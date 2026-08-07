#!/usr/bin/env python3
"""可動軸を**連続**に見る干渉検査（姿勢を組み直さない）。

⚠ 姿勢を数点だけ標本にする検査は穴だらけ。assembly_check の掃引は
  ヨー 60° を 7 点（10° 刻み＝半径 372 で弧 65mm ごと）、仰角 50° を 5 点、
  grab 316mm を 4 点（間隔 105mm）で見ている。部品の寸法（数十 mm）より
  粗い標本なので、
  **両端で当たらないことは途中で当たらないことを意味しない**。
  実際 beltshaft はキャリッジ中央（g212）でしか当たらず、両端では
  何も出なかった。

  リンクの運動は剛体変換なので、1 回組んだソリッドを**変換するだけ**で
  任意の中間姿勢が作れる。組み直し（1 姿勢 100〜290s）が要らないので
  ヨー 0.5° / 直動 2mm 刻みまで細かく回せる。

手順（後段は前段が通った組だけを見る）
  1. 運動が同じソリッドをまとめる（グループ）。
  2. 各ソリッドの**掃引 AABB**（全可動範囲の和）を作る。
     和は AABB なので実体より必ず大きい → 重ならなければ当たらない、が確定。
  3. 掃引 AABB が重なる組だけ、細かい格子で AABB の重なりを最大化する
     標本を探す。AABB の重なりは実体の重なりの**上限**なので、
     最大値が許容以下ならその組は当たらないと**証明できる**（実体計算不要）。
  4. 残った組だけ、その最悪標本で実体のブーリアンを取る。

時間の内訳（測ってから直すこと。`--md` 無しで走らせると標準エラーに出る）
  組立て 24s / 掃引AABB 0.1s / 粗ふるい 0.6s / 格子でAABB最大化 5s /
  **実体判定 2,590s** ← 99% がここ。1 標本 0.9 秒 × 2,865 標本。
  実物のソリッドは面数が多いので、距離（0.87s）もブーリアン（0.94s）も
  同じくらい重い。→ ここだけ子プロセスに分ける（10 並列で 329s、7.9 倍）。
  `-j 1` で逐次に戻せる。出力は逐次と 1 文字も変わらないことを確認済み。
"""
from __future__ import annotations

import argparse
import itertools
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Pos, Rot  # noqa: E402

import assembly_check as AC  # noqa: E402

# 掃引 AABB の行列積を GPU に載せる（無ければ numpy のまま動く）
try:
    import gpu_geom as _GPU  # noqa: E402
    if not _GPU.HAVE_TORCH or os.environ.get("TR_GPU", "1") == "0":
        _GPU = None
except Exception:                             # noqa: BLE001
    _GPU = None
import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402

# 関節原点（tr_assembly.JOINTS と同じもの。ここで独自に持たない）
_JY = A.JOINTS["turret_yaw"]["origin"]
_JP = A.JOINTS["shooter_pitch"]["origin"]
_JF = A.JOINTS["fork_tilt"]["origin"]
PJY = Pos(*_JY)
PJP = Pos(*_JP)
PJF = Pos(*_JF)

# 軸 → (下限, 上限, 細かい刻み)
AXES = {
    "yaw": (-P.YAW_LIMIT, P.YAW_LIMIT, 0.5),
    "pitch": (P.PITCH_MIN, P.PITCH_MAX, 0.5),
    "grab": (0.0, P.GRAB_STROKE, 2.0),
    "press": (0.0, P.PRESS_STROKE, 2.0),
    "tilt": (0.0, P.FORK_TILT_MAX, 0.5),
}
# 掃引 AABB を作るときの粗い刻み。回転は**和が実体を覆う**ように
# 細かめに取る（5° なら半径 400 で矢高 0.38mm、+1mm の膨らましで足りる）。
COARSE = {"yaw": 2.0, "pitch": 2.0, "grab": 20.0, "press": 10.0, "tilt": 2.0}
INFLATE = 1.0            # 掃引 AABB の膨らまし [mm]
GRID_BUDGET = 60_000     # 1 組あたりの標本数の上限
BOOL_PICKS = 8           # 1 組あたり実体計算する標本の数（区間の代表点）


def mkey(name: str, link: str):
    """運動が同じものに同じ鍵を返す。

    ⚠ レールの保持器は段と**速度が違う**（0.25 / 0.75）。link_of は
      どちらも rail_mid に落とすので、鍵は名前から作る。
    """
    if name.startswith("rail_ball_"):
        k = int(name[len("rail_ball_") + 1])
        return ("slide", 0.25 + 0.5 * k)
    if link == "turret_yaw":
        return ("yaw",)
    if link in ("shooter_pitch", "roller_upper", "roller_lower"):
        return ("pitch",)
    if link in ("fork", "fork_tilt"):
        return ("fork",)
    if link == "grabber_press":
        return ("press",)
    if link == "grabber_slide":
        return ("slide", 1.0)
    if link == "rail_mid":
        return ("slide", 0.5)
    return ("fix",)


# 鍵 → その運動に効く軸
KEY_AXES = {"yaw": ("yaw",), "pitch": ("yaw", "pitch"), "fork": ("grab", "tilt"),
            "press": ("grab", "press"), "slide": ("grab",), "fix": ()}


def loc_of(key, s: dict, b: dict):
    """基準姿勢 b から標本 s へ動かす剛体変換（Location）。

    ⚠ 「その姿勢の Location」ではなく「基準姿勢からの**差分**」。
      すでに組んである形状に掛けるので、基準の分を戻してから進める。
    """
    kind = key[0]
    if kind == "fix":
        return Pos(0, 0, 0)
    if kind == "yaw":
        return PJY * Rot(0, 0, s["yaw"] - b["yaw"]) * PJY.inverse()
    if kind == "pitch":
        # X  = PJY・Rz(yaw)・PJP・Ry(-pitch)・C
        # M  = X・X0⁻¹ = PJY・Rz(yaw)・PJP・Ry(-Δpitch)・PJP⁻¹・Rz(-yaw0)・PJY⁻¹
        return (PJY * Rot(0, 0, s["yaw"]) * PJP
                * Rot(0, -(s["pitch"] - b["pitch"]), 0)
                * PJP.inverse() * Rot(0, 0, -b["yaw"]) * PJY.inverse())
    if kind == "fork":
        return (Pos(-s["grab"], 0, 0) * PJF * Rot(0, s["tilt"] - b["tilt"], 0)
                * PJF.inverse() * Pos(b["grab"], 0, 0))
    if kind == "press":
        return Pos(-(s["grab"] - b["grab"]), 0, -(s["press"] - b["press"]))
    return Pos(-key[1] * (s["grab"] - b["grab"]), 0, 0)


def mat_of(loc) -> np.ndarray:
    """Location → 3×4 の同次行列。"""
    t = loc.wrapped.Transformation()
    return np.array([[t.Value(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)])


def corners(bb) -> np.ndarray:
    a, b = bb.min, bb.max
    return np.array([[x, y, z] for x in (a.X, b.X) for y in (a.Y, b.Y)
                     for z in (a.Z, b.Z)])


def feasible(s: dict) -> bool:
    """機構と動作の順序で**あり得る姿勢か**を判定する。

    ⚠ 姿勢の全組合せを回すと、機構的にあり得ない組で不良が出る。
      櫛歯の傾斜は**受動**（引込み端の固定カムが押し下げる）なので
      grab の関数であって、独立に指令できる軸ではない。
      実際、傾斜 8° と grab 133.7 の組（カムから 94mm も離れている）で
      上押さえパッドが櫛歯に 2,874mm³ 入る、という報告が出た。
      そこは **そもそも傾斜しない**。
    ⚠ さらに、傾斜しているあいだに上押さえを降ろすと傾いた櫛歯を潰す。
      これは動作の順序で守る制約（引込み前に上押さえを退避させる）なので、
      ここでも同じ制約を課す。守らない動作を検査しても意味が無い。
    """
    tl = P.FORK_TILT_MAX * max(0.0, 1.0 - s["grab"] / P.FORK_TILT_GRAB)
    if abs(s["tilt"] - tl) > 0.51:
        return False
    return not (s["tilt"] > 0.5 and s["press"] > 0.5)


def grid(axes, base, budget=GRID_BUDGET):
    """軸ごとの標本列。細かい刻みで budget を超えるなら、その分だけ荒くする。

    ⚠ 荒くしたときは**荒くしたことを表に出す**（黙って上限で切ると
      「全部見た」ように読める）。戻り値に実際の刻みを含める。
    """
    if not axes:
        return [dict(base)], {}
    cols, step = [], {}
    for ax in axes:
        lo, hi, st = AXES[ax]
        cols.append((ax, lo, hi, st))
    n = 1
    for _ax, lo, hi, st in cols:
        n *= int(round((hi - lo) / st)) + 1
    scale = 1.0
    if n > budget:
        scale = (n / budget) ** (1.0 / len(cols))
    vals = []
    for ax, lo, hi, st in cols:
        st2 = st * scale
        cnt = max(2, int(round((hi - lo) / st2)) + 1)
        vals.append(np.linspace(lo, hi, cnt))
        step[ax] = (hi - lo) / (cnt - 1)
    out = []
    for combo in itertools.product(*vals):
        s = dict(base)
        for (ax, *_), v in zip(cols, combo):
            s[ax] = float(v)
        # ⚠ 櫛歯の傾斜は grab の従属変数。格子で独立に振ったままにすると
        #   あり得ない姿勢を検査してしまうので、grab から作り直す。
        if "grab" in axes:
            s["tilt"] = P.FORK_TILT_MAX * max(
                0.0, 1.0 - s["grab"] / P.FORK_TILT_GRAB)
        if feasible(s):
            out.append(s)
    if not out:
        out = [dict(base)]
    return out, step


# 変換行列の束は (運動グループ, 格子) だけで決まる。組ごとに作り直すと
# 80,000 標本 × 350 組 の Location 生成でここが全体の律速になる。
_MATS: dict = {}


def mats(key, gid: int, samples, base) -> np.ndarray:
    """標本ごとの 3×4 行列（キャッシュ付き）。戻りは (S,3,4)。"""
    if gid is None:                       # 局所格子は溜めない
        return np.array([mat_of(loc_of(key, s, base)) for s in samples])
    ck = (key, gid)
    ms = _MATS.get(ck)
    if ms is None:
        ms = np.array([mat_of(loc_of(key, s, base)) for s in samples])
        _MATS[ck] = ms
    return ms


def polish(fn, ax, s0: dict, base: dict, steps: dict, passes: int = 3):
    """粗い格子で見つけた最悪標本の**近傍を細かい刻みで詰める**。

    ⚠ 3 軸以上が絡む組は、標本数の上限（GRID_BUDGET）のせいで刻みが
      grab 12.6mm / ヨー 3.2° まで荒くなる。部品の寸法より粗いので、
      「粗い格子で当たらなかった」は「当たらない」を意味しない。
      粗い格子で大域を見たあと、その近傍だけ**軸ごとに順に**細かく
      動かして詰める（軸ごとなら 1 組あたり 150 標本ほどで済む）。
    fn は標本のリストを受けて評価値の配列（大きいほど悪い）を返す。
    """
    best = dict(s0)
    bv = float(fn([best])[0])
    for _ in range(passes):
        moved = False
        for a in ax:
            lo, hi, fine = AXES[a]
            half = steps.get(a, fine)
            if half <= fine * 1.01:
                continue                  # すでに細かい刻みで見ている
            n = int(min(24, max(2, round(2 * half / fine)))) + 1
            col = np.clip(np.linspace(best[a] - half, best[a] + half, n), lo, hi)
            cand_ = [dict(best, **{a: float(v)}) for v in col]
            vals = fn(cand_)
            k = int(np.argmax(vals))
            if vals[k] > bv + 1e-9:
                bv = float(vals[k]); best = cand_[k]; moved = True
        if not moved:
            break
    return best, bv


# ⚠ **ここが連続掃引の律速**。レポートの内訳がそれを言っている:
#     標本 40 万件 / 実体を見た標本 3,621（うち 3,553 は距離が正で
#     ブーリアン不要）。つまり実体計算はほとんど走っておらず、
#     時間は「標本ごとの AABB を作る行列積」に行っている。
#   組ごとに (S,8,3) を 2 回作るので、S が 7 万を超える格子では
#   1 組あたり数千万要素になる。ここは**近似の入らない数値計算**なので、
#   内外判定と違って形の精度を落とさずに移せる。
# ⚠ **float64 のまま計算する。** float32 にすると座標が 1e-4mm ずれ、
#   AABB 重なりの体積が `BOOL_MIN` の境目で反転しうる。行列積の形は
#   (8,3) @ (S,3,3) と小さいので、FP64 が遅い GPU でも numpy より速い。
#   `TR_GPU=0` で numpy に戻せる。
_MATS_T: dict = {}
_COR_T: dict = {}


def aabbs(key, samples, base, cor: np.ndarray, gid: int = 0) -> np.ndarray:
    """標本ごとの AABB。戻りは (S, 2, 3)。"""
    ms = mats(key, gid, samples, base)
    # 標本が少ないと転送とカーネル起動のほうが高くつく（実測: S=1,000 では
    # numpy 1.0ms に対し GPU も同程度、S=20,000 で 16x、S=73,440 で 24x）。
    if _GPU is not None and len(ms) >= 2048:
        t = _GPU.torch
        ck = (key, gid)
        mt = _MATS_T.get(ck) if gid is not None else None
        if mt is None:
            mt = t.as_tensor(ms, device=_GPU.DEVICE, dtype=t.float64)
            if gid is not None:
                _MATS_T[ck] = mt
        ct = _COR_T.get(id(cor))
        if ct is None:
            ct = t.as_tensor(cor, device=_GPU.DEVICE, dtype=t.float64)
            _COR_T[id(cor)] = ct
        pts = ct @ mt[:, :, :3].transpose(1, 2) + mt[:, None, :, 3]
        return t.stack([pts.amin(dim=1), pts.amax(dim=1)], dim=1).cpu().numpy()
    pts = cor @ ms[:, :, :3].transpose(0, 2, 1) + ms[:, None, :, 3]  # (S,8,3)
    return np.stack([pts.min(axis=1), pts.max(axis=1)], axis=1)


def lap_vol(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """AABB どうしの重なり体積。u,v は (S,2,3) か (1,2,3)。"""
    lo = np.maximum(u[:, 0], v[:, 0])
    hi = np.minimum(u[:, 1], v[:, 1])
    return np.prod(np.clip(hi - lo, 0.0, None), axis=1)


def gap(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """AABB どうしの**すきまの下限**。u,v は (S,2,3)。

    ⚠ AABB は実体を必ず包むので、AABB が d 離れていれば実体も d 以上
      離れている。**この向きだけは AABB で証明できる**（逆に AABB が
      重なっていても実体が接しているとは言えない）。
      軸ごとの離れの最大値を返す（ユークリッド距離はこれ以上）。
    """
    d = np.maximum(v[:, 0] - u[:, 1], u[:, 0] - v[:, 1])   # (S,3)
    return np.clip(d, 0.0, None).max(axis=1)


# ---------------------------------------------------------------------------
# 実体の重なりは「まず距離で落とす」
# ---------------------------------------------------------------------------
# ⚠ **距離はブーリアンよりずっと安い。** 実測（面数のある板どうし 16 回）
#     ブーリアン交差  68.9 秒
#     距離（BRepExtrema）1.96 秒        ← 35 分の 1
#   そして「距離 > 0」なら**重なりは 0 と確定する**（触れてすらいない）。
#   AABB が重なる組の大半は実際には離れているので、ここで落とせば
#   ブーリアンを回す回数そのものが減る。
#
# ⚠ 逐次版と結果が変わらないことを確かめること:
#     ・d > 0  … `sa & sb` は None を返し、体積は 0.0。同じ。
#     ・d == 0 … 接触か重なり。ブーリアンを回す（従来どおり）。
#   もとの版も「体積が 1mm³ 以下で宣言が無い」ときに同じ距離を計算して
#   いたので、**先に測るだけ**で計算の中身は増えていない。
#
# ⚠ ここを**プロセスで並列化するのは筋が悪い**（試して駄目だった記録）:
#     ・OCP はブーリアンのあいだ GIL を離さない
#       （8 スレッドで 1.10 倍にしかならない）
#     ・TopoDS_Shape は pickle できないので spawn では形を渡せない
#     ・fork は**デッドロックする**。OCCT はブーリアンを並列で回すために
#       作業スレッドを作る（親は 27 スレッドになっていた）ので、
#       そのあとに fork した子は、存在しないスレッドが握ったままの
#       ミューテックスを待って永久に止まる（実際 8 子とも CPU 0% で
#       futex 待ちのまま 10 分放置になった）。
_COST = {"dist_s": 0.0, "bool_s": 0.0}


def _overlap_or_gap(a_, b_):
    """(重なり体積, 距離) を返す。距離が正なら重なりは 0 と確定する。"""
    t = time.perf_counter()
    d_ = a_.distance_to(b_)
    _COST["dist_s"] += time.perf_counter() - t
    if d_ > 0.0:
        return 0.0, d_
    t = time.perf_counter()
    v_ = AC.overlap_volume(a_, b_)
    _COST["bool_s"] += time.perf_counter() - t
    return v_, 0.0


# ---------------------------------------------------------------------------
# 実体判定をプロセスで分ける（BREP でソリッドを渡す）
# ---------------------------------------------------------------------------
# 段ごとの実測（budget 1500）: 組立て 18.4s / 掃引AABB 0.2s / 粗ふるい 0.6s /
# 格子でAABB最大化 4.0s / **実体判定 約 2,590s**。99% がここ。
# 1 標本あたり約 0.9 秒（距離 0.87s・ブーリアン 0.94s）で、実物のソリッドは
# 面数が多いので**距離もブーリアンとほぼ同じだけ重い**（合成ベンチの 35 倍差は出ない）。
# 減らせる回数は減らしたので、あとは分けて回すしかない。
#
# ⚠ **fork は使えない。** OCCT はブーリアンを並列で回すために作業スレッドを
#   作る（組み上げ後の親は 27 スレッドになっていた）。そのあと fork した子は、
#   存在しないスレッドが握ったままのミューテックスを待って永久に止まる
#   （実際に 8 子とも CPU 0% の futex 待ちで放置になった）。
# ⚠ **spawn でも形は渡せない。** TopoDS_Shape は pickle できない。
# ⚠ **子に組み直させるのも駄目。** 1 プロセス 11.5GB 使うので、10 並列で
#   115GB。機械が落ちる。
#   → 親が **BREP に書き出し**、子は spawn してそれを読む。読み込みは
#     組み直しよりずっと速く、組立ての途中の一時オブジェクトを持たないので
#     メモリも小さい。並び順は BRep_Builder に入れた順のまま出てくる。
_W: dict = {}


def _wrap(topods):
    """TopoDS_Shape を build123d の形に包む。"""
    from build123d import Compound, Solid
    from build123d.topology.shape_core import downcast
    from OCP.TopAbs import TopAbs_ShapeEnum
    sh = downcast(topods)
    if sh.ShapeType() == TopAbs_ShapeEnum.TopAbs_SOLID:
        return Solid(sh)
    return Compound(sh)


def _write_brep(items, path) -> np.ndarray:
    """ソリッドを**並び順のまま** 1 つの BREP に書き出し、bbox の表を返す。"""
    from OCP.BRep import BRep_Builder
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Compound
    comp = TopoDS_Compound()
    bld = BRep_Builder()
    bld.MakeCompound(comp)
    ref = np.empty((len(items), 6))
    for k, (_nm, _key, s, _cor) in enumerate(items):
        bld.Add(comp, s.wrapped)
        bb = s.bounding_box()
        ref[k] = [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z]
    BinTools.Write_s(comp, str(path))
    return ref


def _worker_init(path, keys, base, ref):
    """子プロセスの初期化。BREP を読み、**親と同じ並び**であることを確かめる。

    ⚠ 並びがずれたら組の添字が別の部品を指す。数字は出るが意味が無い
      （この検査でいちばん怖い壊れ方）。bbox を突き合わせて、
      1 つでも違えば**その場で落とす**。
    """
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Iterator, TopoDS_Shape
    sh = TopoDS_Shape()
    BinTools.Read_s(sh, str(path))
    sol = []
    it = TopoDS_Iterator(sh)
    while it.More():
        sol.append(_wrap(it.Value()))
        it.Next()
    if len(sol) != len(keys):
        raise RuntimeError(f"BREP のソリッド数が {len(sol)}、親は {len(keys)}")
    for k, s in enumerate(sol):
        bb = s.bounding_box()
        got = (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)
        if max(abs(a - b) for a, b in zip(got, ref[k])) > 1e-6:
            raise RuntimeError(f"BREP の {k} 番目の bbox が親と違う")
    _W["sol"] = sol
    _W["keys"] = [tuple(k) for k in keys]
    _W["base"] = base


def _worker_task(t):
    """(組, 標本番号, i, j, 姿勢, 距離も要るか) → 体積と距離。"""
    k, si, i, j, s_, _need_dist = t
    try:
        a_ = loc_of(_W["keys"][i], s_, _W["base"]) * _W["sol"][i]
        b_ = loc_of(_W["keys"][j], s_, _W["base"]) * _W["sol"][j]
        v_, d_ = _overlap_or_gap(a_, b_)
    except Exception as e:                            # noqa: BLE001
        return (k, si), (None, None, str(e))
    return (k, si), (v_, d_, None)


def _run_tasks(tasks, items, jobs: int) -> dict:
    """実体判定をまとめて回す。`jobs`>1 なら子プロセスに分ける。

    ⚠ **分けられなかったら黙って諦めて逐次に落ちる。** 検査そのものは
      どちらでも同じ結果になるので、速さのために結果を落としてはいけない。
      落ちたことは標準エラーに出す（黙って遅いのがいちばん困る）。
    """
    def run_here():
        out = {}
        for t in tasks:
            k, si, i, j, s_, _nd = t
            try:
                a_ = loc_of(items[i][1], s_, base_g) * items[i][2]
                b_ = loc_of(items[j][1], s_, base_g) * items[j][2]
                out[(k, si)] = (*_overlap_or_gap(a_, b_), None)
            except Exception as e:                    # noqa: BLE001
                out[(k, si)] = (None, None, str(e))
        return out

    base_g = _BASE_G[0]
    if jobs <= 1 or len(tasks) < 4 * jobs:
        return run_here()

    import multiprocessing as mp
    import tempfile
    tmp = tempfile.mkdtemp(prefix="sweep_fine_")
    path = Path(tmp) / "solids.brep"
    try:
        t0 = time.perf_counter()
        ref = _write_brep(items, path)
        keys = [it[1] for it in items]
        print(f"[時間] BREP 書き出し {time.perf_counter() - t0:.1f}s "
              f"({path.stat().st_size / 2**20:.0f}MB, {len(items)} ソリッド)",
              file=sys.stderr, flush=True)
        # ⚠ spawn を使う（fork は OCCT の作業スレッドのせいで固まる）。
        ctx = mp.get_context("spawn")
        with ctx.Pool(processes=jobs, initializer=_worker_init,
                      initargs=(str(path), keys, base_g, ref.tolist())) as pool:
            got = dict(pool.map(_worker_task, tasks,
                                chunksize=max(1, len(tasks) // (jobs * 8))))
        return got
    except Exception as e:                            # noqa: BLE001
        print(f"[警告] 並列化できなかったので逐次で回す: {e}",
              file=sys.stderr, flush=True)
        return run_here()
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)


# 子へ渡す基準姿勢（main が入れる）
_BASE_G: list = [None]


def verify(base_pose: dict) -> int:
    """変換モデルを**実際の組み直し**と突き合わせる。

    ⚠ この検査の全部が loc_of の正しさに乗っている。行列を 1 つ間違えれば
      「どこも当たらない」と嘘をつく。運動グループごとに 1 部品を選び、
      変換した実体の bbox と、その姿勢で組み直した実体の bbox を比べる。
      **bbox を変換したもの**と比べてはいけない（回転では外接箱が
      膨らむので誤差が出て当然になり、検査にならない）。
    """
    b = {k: base_pose[k] for k in ("yaw", "pitch", "grab", "press", "tilt")}
    items: dict = {}
    for path, s, _bb in V.solids_with_bbox(A.build(base_pose)):
        nm = AC.part_name(path)
        items.setdefault(nm, (mkey(nm, AC.link_of(path)), s))
    tgt = dict(base_pose)
    tgt.update(yaw=13.0, pitch=37.0, grab=131.0, press=41.0, tilt=7.0)
    ref = {}
    for path, _s, bb in V.solids_with_bbox(A.build(tgt)):
        ref.setdefault(AC.part_name(path), bb)
    sm = {k: tgt[k] for k in b}
    per: dict = {}
    for nm, (key, _s) in items.items():
        per.setdefault(key, nm)
    ng = 0
    for key, nm in sorted(per.items()):
        mv = (loc_of(key, sm, b) * items[nm][1]).bounding_box()
        r = ref[nm]
        e = max(abs(mv.min.X - r.min.X), abs(mv.min.Y - r.min.Y),
                abs(mv.min.Z - r.min.Z), abs(mv.max.X - r.max.X),
                abs(mv.max.Y - r.max.Y), abs(mv.max.Z - r.max.Z))
        ng += e > 1e-6
        print(f"  {str(key):18s} {nm:18s} 誤差 {e:.6f}mm"
              + ("  ← 一致しない" if e > 1e-6 else ""))
    print("変換モデルは組み直しと一致" if not ng else f"**{ng} グループが不一致**")
    return ng


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="match", choices=list(AC.ALL_POSES))
    ap.add_argument("--limit", type=int, default=40)
    ap.add_argument("--budget", type=int, default=GRID_BUDGET)
    ap.add_argument("--md", action="store_true",
                    help="check_all が渡す（出力はそのまま Markdown として保存）")
    ap.add_argument("--verify", action="store_true",
                    help="変換モデルを組み直しと突き合わせるだけ")
    # 子は BREP を読むだけなので**軽い**。実測 1 プロセス 0.51GB
    # （組み直させると 11.5GB。ここが BREP 受け渡しにした理由）。
    # この検査は check_all の中でいちばん長く、他が終わったあと 1 本だけ
    # 残るので、コアはほぼ使い切ってよい。
    ap.add_argument("-j", "--jobs", type=int,
                    default=max(1, (os.cpu_count() or 4) - 4),
                    help="実体判定を分けるプロセス数（1 で逐次）")
    args = ap.parse_args()
    if args.verify:
        return verify(dict(AC.ALL_POSES[args.pose]()))

    # 各段の所要時間は**標準エラー**に出す。
    # ⚠ 標準出力は check_all がそのまま out/sweep_fine.md にする（人が読む
    #   資料）ので、時間を混ぜない。どこが重いのかは、混ぜずに見られること。
    _t0 = [time.perf_counter()]

    def lap(label: str) -> None:
        now = time.perf_counter()
        print(f"[時間] {label:26s} {now - _t0[0]:7.1f}s", file=sys.stderr,
              flush=True)
        _t0[0] = now

    pose = dict(AC.ALL_POSES[args.pose]())
    base = {"yaw": pose.get("yaw", 0.0), "pitch": pose.get("pitch", P.PITCH_DEFAULT),
            "grab": pose.get("grab", 0.0), "press": pose.get("press", 0.0),
            "tilt": pose.get("tilt", 0.0)}
    shape = A.build(pose)
    sol = V.solids_with_bbox(shape)
    print(f"基準姿勢 {args.pose} … ソリッド {len(sol)}")

    lap("組立て")
    items = []          # (部品名, 鍵, ソリッド, 8隅)
    for path, s, bb in sol:
        nm = AC.part_name(path)
        items.append((nm, mkey(nm, AC.link_of(path)), s, corners(bb)))

    # --- 1. 掃引 AABB（全可動範囲の和）------------------------------------
    keys = sorted({it[1] for it in items})
    cg = {}
    for key in keys:
        ax = KEY_AXES[key[0]]
        if not ax:
            cg[key] = [dict(base)]
            continue
        cols = [np.arange(AXES[a][0], AXES[a][1] + 1e-9, COARSE[a]) for a in ax]
        cg[key] = [dict(base, **dict(zip(ax, map(float, c))))
                   for c in itertools.product(*cols)]
    swept = []
    for nm, key, s, cor in items:
        bbs = aabbs(key, cg[key], base, cor, id(cg[key]))
        swept.append(np.array([bbs[:, 0].min(axis=0) - INFLATE,
                               bbs[:, 1].max(axis=0) + INFLATE]))
    print(f"運動グループ {len(keys)}: " +
          ", ".join(f"{k[0]}{k[1] if len(k) > 1 else ''}"
                    f"×{sum(1 for it in items if it[1] == k)}" for k in keys))

    lap("1. 掃引AABB")

    # --- 2. 相対運動のある組で、掃引 AABB が重なるものを拾う ---------------
    cand = []
    for i, j in itertools.combinations(range(len(items)), 2):
        if items[i][0] == items[j][0] or items[i][1] == items[j][1]:
            continue        # 同じ部品／相対運動なし（静的検査の担当）
        u, v = swept[i], swept[j]
        lo = np.maximum(u[0], v[0])
        hi = np.minimum(u[1], v[1])
        if np.all(hi > lo) and float(np.prod(hi - lo)) > AC.BOOL_MIN:
            cand.append((i, j))
    print(f"掃引 AABB が重なる組 {len(cand)}")

    lap("2. 総当たりの粗ふるい")

    # --- 3. 組ごとに細かい格子で AABB 重なりを最大化（上限なので証明になる）
    cache: dict = {}
    steps_used: dict = {}
    refine = []
    pol = [0, 0.0]      # 局所詰め込みで悪化を見つけ直した組の数と最大増分
    for i, j in cand:
        ki, kj = items[i][1], items[j][1]
        ax = tuple(sorted(set(KEY_AXES[ki[0]]) | set(KEY_AXES[kj[0]])))
        g = cache.get(ax)
        if g is None:
            g, st = grid(ax, base, args.budget)
            cache[ax] = g
            steps_used[ax] = st
        bi = aabbs(ki, g, base, items[i][3], id(g))
        bj = aabbs(kj, g, base, items[j][3], id(g))
        vv = lap_vol(bi, bj)
        m = int(vv.argmax())
        dec = F.declared(items[i][0], items[j][0])
        ok = dec[1] if dec else 0.0
        s_bad, v_bad = g[m], float(vv[m])
        # 粗い刻みになった軸があれば、その近傍を細かく詰める。
        # ⚠ 「粗い格子で当たらなかった」は「当たらない」ではない。
        if ax and any(steps_used[ax].get(a, 0) > AXES[a][2] * 1.01 for a in ax):
            def _f(ss, ki=ki, kj=kj, i=i, j=j):
                return lap_vol(aabbs(ki, ss, base, items[i][3], None),
                               aabbs(kj, ss, base, items[j][3], None))
            v0 = v_bad
            s_bad, v_bad = polish(_f, ax, s_bad, base, steps_used[ax])
            if v_bad > v0 * 1.001:
                pol[0] += 1
                pol[1] = max(pol[1], v_bad - v0)
        if v_bad <= AC.BOOL_MIN or v_bad <= ok:
            continue        # AABB の上限が許容以下 → 実体計算せずに安全と確定
        # ⚠ **AABB 重なりの最大点＝実体の重なりの最大点ではない。**
        #   bbox が大きい部品（配線は X -356..226 / Y 144..342 / Z 140..876）
        #   では AABB の重なりは実体とほとんど関係が無く、1 点だけ実体計算
        #   すると当たっていても見逃す。実際 cab_turret ↔ lift_pul_L_lo は
        #   ヨー -30° で 1,081mm³ 重なっているのに、AABB の最大点が別の
        #   ヨー角だったため「0 件」と報告していた（validate.py の距離
        #   チェックと突き合わせて発覚）。
        #   → 許容を超える AABB 重なりを持つ標本を**区間に分けて代表点を
        #     取り**、複数点で実体計算する。
        qual = np.flatnonzero(vv > max(ok, AC.BOOL_MIN))
        picks = [s_bad]
        if len(qual) > 1:
            for chunk in np.array_split(qual, min(BOOL_PICKS - 1, len(qual))):
                picks.append(g[int(chunk[np.argmax(vv[chunk])])])
        refine.append((i, j, picks, dec, ok, v_bad))
    for ax, st in sorted(steps_used.items()):
        if st:
            print("格子 " + "×".join(f"{a}{st[a]:.2f}" for a in ax)
                  + f"（{len(cache[ax]):,} 標本）")
    if pol[0]:
        print(f"局所詰め込みで AABB 重なりが増えた組 {pol[0]}"
              f"（最大 +{pol[1]:,.0f}mm³）← 粗い格子だけでは見落とす分")
    print(f"実体計算が要る組 {len(refine)}")

    lap("3. 格子でAABB最大化")
    _BASE_G[0] = base

    # --- 4. 最悪標本で実体のブーリアン ------------------------------------
    # ここが**この検査でいちばん重い**。366 組 × 標本 8 点 ＝ 3,000 回近い。
    # ⚠ ブーリアンの前に距離で落とす（`_overlap_or_gap` の註を見ること）。
    tasks = [(k, si, i, j, s_, dec is None)
             for k, (i, j, picks, dec, _ok, _ub) in enumerate(refine)
             for si, s_ in enumerate(picks)]
    done = _run_tasks(tasks, items, args.jobs)

    hits, rub = [], []
    n_bool = n_skip = 0
    for k, (i, j, picks, dec, ok, ub) in enumerate(refine):
        ni, nj = items[i][0], items[j][0]
        v, s, dmin = -1.0, picks[0], None
        for si, s_ in enumerate(picks):
            v_, d_, err = done[(k, si)]
            if v_ is None:
                print(f"  !! {ni} ↔ {nj}: {err}")
                continue
            n_bool += 1
            n_skip += d_ > 0.0
            # ⚠ 比較は**狭義**。同じ値なら先に出た標本を残す（逐次版と同じ）。
            if v_ > v:
                v, s = v_, s_
            if v_ <= 1.0 and dec is None and (dmin is None or d_ < dmin):
                dmin = d_
        if v < 0.0:
            continue
        if v > max(ok, 1.0):
            hits.append((v, ni, nj, dec[0] if dec else "宣言なし", ok, s, ub))
        elif dec is None and dmin is not None and dmin <= AC.CONTACT_TOL:
            # ⚠ 実体は重なっていなくても、**宣言の無い組が行程の途中で
            #   接している**なら擦れる。姿勢を数点しか見ない検査ではここも
            #   見逃す（両端で離れていても途中で接することはある）。
            rub.append((dmin, ni, nj, s))
    print(f"実体を見た標本 {n_bool}（うち {n_skip} は距離が正なので"
          f"ブーリアン不要）")
    # ⚠ 内訳を出しておく。ここは検査全体でいちばん重い段なので、
    #   「距離が重いのか、ブーリアンが重いのか」を知らずに手を入れると外す。
    #   （子プロセスに分けたときは子の側で数えているので親では 0 になる）
    if _COST["dist_s"] or _COST["bool_s"]:
        print(f"[時間] うち距離 {_COST['dist_s']:.1f}s / "
              f"ブーリアン {_COST['bool_s']:.1f}s", file=sys.stderr, flush=True)

    lap("4. 実体（距離＋ブーリアン）")

    # --- 5. 行程の途中で**離れる**宣言（接触必須の締結）------------------
    # ⚠ ここまでは「食い込み」しか見ていない。**接触が必須の締結が行程の
    #   途中で離れる**のも同じ重さの不良で、姿勢を数点しか見ない検査では
    #   同じように見逃す（両端で接していても途中で離れることはある）。
    #   AABB が離れていれば実体も離れている（gap() の向きは証明になる）
    #   ので、実体計算なしで確定できる分だけ拾う。
    by_name: dict = {}
    for idx, (nm, key, _s, cor) in enumerate(items):
        by_name.setdefault(nm, []).append(idx)
    seen_pair = set()
    apart = []
    for nm, lst in F.FIXINGS.items():
        for tgt_, how, _q, _n in lst:
            if not F.HOW[how][0]:
                continue                  # 接触が必須でない締結は対象外
            pr = tuple(sorted((nm, tgt_)))
            if pr in seen_pair or nm not in by_name or tgt_ not in by_name:
                continue
            seen_pair.add(pr)
            ka = {items[i][1] for i in by_name[nm]}
            kb = {items[i][1] for i in by_name[tgt_]}
            if ka == kb and len(ka) == 1:
                continue                  # 相対運動なし（静的検査の担当）
            ax = tuple(sorted(set().union(
                *[set(KEY_AXES[k[0]]) for k in ka | kb])))
            if not ax:
                continue
            g = cache.get(ax)
            if g is None:
                g, st = grid(ax, base, args.budget)
                cache[ax] = g
                steps_used[ax] = st
            # 部品どうしのすきま = ソリッド対の**最小**（どれか 1 対でも
            # 接していれば部品としては接している）
            def _g(ss, nm=nm, tgt_=tgt_, gid=None):
                bst = None
                for i in by_name[nm]:
                    ba = aabbs(items[i][1], ss, base, items[i][3], gid)
                    for j in by_name[tgt_]:
                        d = gap(ba, aabbs(items[j][1], ss, base,
                                          items[j][3], gid))
                        bst = d if bst is None else np.minimum(bst, d)
                return bst
            best = _g(g, gid=id(g))
            m = int(best.argmax())
            s_bad, d_bad = g[m], float(best[m])
            if any(steps_used[ax].get(a, 0) > AXES[a][2] * 1.01 for a in ax):
                s_bad, d_bad = polish(_g, ax, s_bad, base, steps_used[ax])
            if d_bad > AC.CONTACT_TOL:
                apart.append((d_bad, nm, tgt_, how, s_bad))
    lap("5. 締結の離れ")
    apart.sort(reverse=True)

    hits.sort(reverse=True)
    print()
    if rub:
        rub.sort()
        print(f"### 行程の途中で接する（宣言の無い組）{len(rub)} 件")
        for d, na, nb, s in rub[:args.limit]:
            at = " ".join(f"{k}={s[k]:.1f}" for k in ("yaw", "pitch", "grab",
                                                     "press", "tilt")
                          if abs(s[k] - base[k]) > 1e-9)
            print(f"  {d:8.2f}mm  {na} ↔ {nb}  @ {at or '基準姿勢'}")
        if len(rub) > args.limit:
            print(f"（他 {len(rub) - args.limit} 件）")
        print()
    else:
        print("行程の途中で接する（宣言の無い組）0")
    if apart:
        print(f"### 行程の途中で離れる締結 {len(apart)} 件")
        for d, na, nb, how, s in apart[:args.limit]:
            at = " ".join(f"{k}={s[k]:.1f}" for k in ("yaw", "pitch", "grab",
                                                     "press", "tilt")
                          if abs(s[k] - base[k]) > 1e-9)
            print(f"  {d:8.2f}mm  {na} ↮ {nb}  [{how}]  @ {at or '基準姿勢'}")
        if len(apart) > args.limit:
            print(f"（他 {len(apart) - args.limit} 件）")
        print()
    else:
        print("行程の途中で離れる締結 0")
    if not hits:
        print("行程の途中での食い込み 0")
    else:
        print(f"### 行程の途中での食い込み {len(hits)} 件")
        for v, ni, nj, how, ok, s, ub in hits[:args.limit]:
            at = " ".join(f"{k}={s[k]:.1f}" for k in ("yaw", "pitch", "grab",
                                                      "press", "tilt")
                          if abs(s[k] - base[k]) > 1e-9)
            print(f"  {v:9,.0f}mm³  {ni} ↔ {nj}  [{how} 許容{ok:,.0f}]"
                  f"  @ {at or '基準姿勢'}  (AABB上限 {ub:,.0f})")
        if len(hits) > args.limit:
            print(f"（他 {len(hits) - args.limit} 件）")
    return 1 if (hits or apart or rub) else 0


if __name__ == "__main__":
    raise SystemExit(main())
