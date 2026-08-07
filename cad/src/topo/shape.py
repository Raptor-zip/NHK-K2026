"""密度場 → 切り抜き輪郭 → build123d の面.

`simp.py` が吐いた `TopoResult.dens`（0..1 の格子）は、そのままでは切れない。
ここでやるのは 3 段:

    1. しきい値 `level` の等高線を引いて、閉じた多角形の集まりにする
    2. 外形と穴を判別し、簡略化と角の丸めを掛けて **切れる形**にする
    3. 製造・構造上の不良を数値で検査する（`check`）

⚠ **このモジュールの出力がそのまま切削データになる。** 「だいたい合っている
  形」を返してはいけない。面積・最小部材幅は必ず実測した値を入れる。
  ここで嘘をつくと、質量台帳も強度の見積もりも全部ずれる。

座標系は `core` と同じ **板ローカル mm**（原点＝設計領域の外接矩形の中心）。

キャッシュ
----------
最適化は重い（板 1 枚で数十秒）。STEP を作り直すたびに解き直すのは無駄なので、
`to_json` / `from_json` で `out/topo/<板名>.json` に輪郭だけ落としておき、
STEP 生成はそれを読むだけにする。座標は小数点以下 3 桁に丸める
（⚠ 丸めないと座標 1 個が `-93.37499999999999` のように 20 桁になり、
git の差分が読めなくなる。3 桁＝1μm はレーザーのカーフの 1/200 で十分）。

実測（300x200 のトラス状の板 1 枚）: 輪郭 199 点 / JSON 9.1k 文字 /
STEP 1.9MB / `to_outline` 0.43 秒。
"""

from __future__ import annotations

import json
import math
import warnings
from functools import reduce
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
from matplotlib.figure import Figure
from scipy.ndimage import distance_transform_edt
from shapely import contains_xy, prepare
from shapely.geometry import MultiPolygon, Point as SPoint, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union
from shapely.geometry.polygon import orient

from .core import RIB_MIN, Outline, Region, TopoResult

# --- 検査のしきい値 --------------------------------------------------------
# ⚠ 直径 8mm は「エンドミル φ6 が入って、ひとまわりできる」下限。これより
#   小さい穴は加工の手間だけ増えて軽くならない。等高線の簡略化が作るゴミ
#   （数 mm の三日月）もここで落ちる。
MIN_HOLE_DIA = 8.0        # mm これ未満の穴は捨てる
# 面積 18mm² 未満の島は「リブ 1 本ぶんの正方形の半分」以下＝密度場のノイズ。
# ⚠ ここを大きくすると**本当に分断している板を合格にしてしまう**ので上げない。
MIN_ISLAND_AREA = 0.5 * RIB_MIN ** 2
# 丸め演算の前後で面積・最小幅がこれ以上変わったら警告
SMOOTH_TOL = 0.05         # 5%
# 座面が輪郭からはみ出したとみなす下限 [mm²]。既定の丸め R2 が矩形の角を
# 2 つ落とす量 = 2·(1 − π/4)·2² = 1.72mm²。これ以下は「角が丸まっただけ」。
SEAT_EPS = 2 * (1 - math.pi / 4) * 2.0 ** 2
# ⚠ **細長い座面では、角ではなく縁に沿って落ちる。** 輪郭は `simplify`(0.4mm)
#   で頂点を間引いた多角形なので、長さ 320mm の辺に沿うと平均 0.03mm の
#   ずれでも 8mm² になる。絶対値だけで切ると、この手の座面が永久に
#   「ねじが留められない」と出続ける。
#   座面は**ねじの頭を 10mm 広げた領域**（`topo_opt.SEAT_GROW`）で、頭そのものは
#   その内側の小さな円。縁の 0.5% が欠けても頭には届かない。
# ⚠ 2026-08-05 に 0.005 → 0.02 へ。曲げ品を平板 2 枚へ分けた結果、座面の
#   形が変わって角の欠けが 1.8〜2.8mm²（座面 211mm² の 1.3%）に増え、
#   38 件が「ねじが留められない（致命的）」として残った。
#   ⚠ **座面はねじの頭を 10mm 広げた領域**で、頭そのもの（M5 で φ8.5 =
#     57mm²）は中心にある。縁が 1.3% 欠けても頭には届かない。
#   本物のはみ出しは 6〜80%（座面の大半が輪郭の外）なので 2% で分離できる。
SEAT_REL = 0.02
# 最大内接円が「真向かいの 2 点で境界に触れている」とみなす最小角 [度]。
# ⚠ ここを 130° 以下に下げると、板の縁でぶつ切りにされた腕の**切り口**が
#   部材の幅として数えられて、幅 20mm の X 字が 17.5mm と報告される。
#   150° 以上なら実測 15 形状すべてで正しい値が出た（下の _min_width 参照）。
_ANG_MIN = 150.0
# 丸めの円弧の分割数（4 分円あたり）。8 で R2 の弦誤差 0.0096mm。
_QSEG = 8

# 8 近傍。Zhang-Suen の P2..P9 の順（北から時計回り）に並べてある。
# ⚠ この順番を崩すと交差数 A の計算が壊れて、細線化が形を切り刻む。
_NB8 = [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)]


# ===========================================================================
# 等高線 → shapely の面
# ===========================================================================
def _rings(res: TopoResult, level: float) -> list[np.ndarray]:
    """密度場の `level` 等高線を、閉じた輪の座標列にして返す（板ローカル mm）。

    ⚠ **密度場のまわりをゼロで 1 周ぶん埋めてから等高線を引く。**
      これをやらないと、板の縁まで材料が詰まっている所（拘束の座は必ずそう
      なる）で等高線が格子の端で切れて、閉じない折れ線が出てくる。閉じない
      折れ線は多角形にできないので、外形が丸ごと消える。

    埋める位置は要素中心のさらに 1 ピッチ外側。要素中心は縁から半ピッチ内側に
    あるので、密度 1 の縁セルと密度 0 のパディングの中点＝ちょうど設計領域の
    縁に等高線が乗る（実測: 200x200 の板で外形面積 39998mm² ≒ 40000）。

    ⚠ 設計領域いっぱいまで材料が詰まっている板は、**4 隅が半セルぶん面取り
      される**（marching squares が角のセルを対角に切るため。dx=2 なら 1 辺
      1mm の三角が 4 つ＝2mm²、100 角の板で -0.02%）。質量の突き合わせで
      「わずかに軽い」が出たらこれ。実害は無いので直していない。
    """
    reg = res.region
    nx, ny = reg.grid()
    X, Y = reg.cell_centers()
    x, y = X[0, :], Y[:, 0]
    ddx, ddy = reg.w / nx, reg.h / ny

    d = np.zeros((ny + 2, nx + 2), dtype=float)
    d[1:-1, 1:-1] = np.asarray(res.dens, dtype=float)
    xx = np.concatenate([[x[0] - ddx], x, [x[-1] + ddx]])
    yy = np.concatenate([[y[0] - ddy], y, [y[-1] + ddy]])

    # ⚠ pyplot は使わない。バックエンドの差し替えという**全体に効く副作用**が
    #   あって、同じプロセスで動く draw.py の figure を壊しうる。Figure を
    #   直接作れば描画装置は要らない。
    ax = Figure().add_subplot(111)
    cs = ax.contour(xx, yy, d, levels=[float(level)])

    out: list[np.ndarray] = []
    for p in cs.get_paths():
        # 1 本の Path に複数の輪が MOVETO 区切りで入っている。to_polygons が分ける。
        for q in p.to_polygons(closed_only=False):
            if len(q) >= 4:
                out.append(np.asarray(q, dtype=float))
    return out


def _material(rings: Sequence[np.ndarray], cell: float):
    """輪の集まりから材料の面を作る（外形と穴の判別込み）。

    包含関係を数えるかわりに **対称差の畳み込み**を使う。ある点が材料かどうかは
    「その点を含む輪の数が奇数か」で決まる（外形の中＝1 個、穴の中＝2 個、
    穴の中の島＝3 個）。対称差はこの偶奇そのものなので、入れ子が何段でも正しい。

    ⚠ 「一番大きい輪が外形、残りは全部穴」と決め打ちすると、穴の中に島が
      残った時に島が消えて、面積が実物より小さく出る。
    """
    polys = []
    for q in rings:
        p = Polygon(q)
        if not p.is_valid:
            p = p.buffer(0)        # 自己交差した輪はここで直る
        if p.is_empty:
            continue
        # 1 セル未満の輪は等高線のノイズ。拾うと穴の数が数えられなくなる。
        if p.area < 0.5 * cell * cell:
            continue
        polys.append(p)
    if not polys:
        return Polygon()
    return reduce(lambda a, b: a.symmetric_difference(b), polys)


def _parts(geom) -> list[Polygon]:
    """面を連結成分（Polygon）の並びにばらす。ノイズの島は落とす。"""
    if geom.is_empty:
        return []
    gs = list(geom.geoms) if isinstance(geom, MultiPolygon) else [geom]
    return [g for g in gs if g.area >= MIN_ISLAND_AREA]


# ===========================================================================
# 最小部材幅 — 距離変換 + 細線化
# ===========================================================================
def _px_for(poly) -> float:
    """ラスタのピッチ [mm]。長辺 800px を目安に、0.25〜1.0mm に収める。

    ⚠ 6mm のリブを測るのに 1mm ピッチだと 6 画素しか無く、±17% ぶれる。
      逆に細かくしすぎると細線化の反復回数（≒最大肉厚の画素数の半分）が
      効いて秒単位で遅くなる。実測: 400 角の板で 0.5mm ピッチ 約 2.8 秒。
    """
    x0, y0, x1, y1 = poly.bounds
    return float(np.clip(max(x1 - x0, y1 - y0) / 800.0, 0.25, 1.0))


def _raster(poly, px: float) -> tuple[np.ndarray, float, float]:
    """多角形の内部を真偽値のラスタにする。返り値は (mask, x原点, y原点)。"""
    x0, y0, x1, y1 = poly.bounds
    # ⚠ まわりに 3 画素の余白を残す。余白が無いと np.roll の巻き込みで
    #   配列の左端と右端が繋がり、骨格が板を貫通する。
    x0 -= 3 * px
    y0 -= 3 * px
    nx = int(math.ceil((x1 + 3 * px - x0) / px))
    ny = int(math.ceil((y1 + 3 * px - y0) / px))
    xs = x0 + (np.arange(nx) + 0.5) * px
    ys = y0 + (np.arange(ny) + 0.5) * px
    XX, YY = np.meshgrid(xs, ys)
    prepare(poly)
    m = contains_xy(poly, XX.ravel(), YY.ravel()).reshape(YY.shape)
    return m, x0, y0


def _thin(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen の細線化。太さ 1 画素の骨格を返す。

    skimage が入っていないので自前。⚠ **形を保つ細線化でなければ意味が無い。**
    単なる収縮で細らせると、輪が切れたり枝が消えたりして「骨格上の最小内接円」
    が測れなくなる。Zhang-Suen は交差数 A==1 を条件に入れることで連結性を保つ。
    """
    img = mask.astype(np.uint8)
    for _ in range(10_000):
        changed = False
        for step in (0, 1):
            P = [np.roll(np.roll(img, dy, 0), dx, 1) for dy, dx in _NB8]
            P2, P3, P4, P5, P6, P7, P8, P9 = P
            B = sum(P)                                    # 近傍の材料数
            seq = P + [P2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8)
                    for i in range(8))                    # 0→1 の遷移数
            c = (img == 1) & (B >= 2) & (B <= 6) & (A == 1)
            if step == 0:
                c &= (P2 * P4 * P6 == 0) & (P4 * P6 * P8 == 0)
            else:
                c &= (P2 * P4 * P8 == 0) & (P2 * P6 * P8 == 0)
            if c.any():
                img[c] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


def _min_width(poly, px: float | None = None,
               loc: bool = False) -> float | tuple[float, tuple[float, float] | None]:
    """多角形の最小部材幅 [mm]（骨格上の最大内接円の直径の最小値）。

    骨格の全部を使ってはいけない
    ----------------------------
    細線化が出す骨格は**位相の骨格**で、凸の角があれば必ずそこまで枝が伸びる。
    枝の先の内接円は小さいので、素直に最小値を取ると
    「幅 20mm の X 字の最小幅は 12mm」という嘘が出る（実測でそうなった）。
    腕が板の縁でぶつ切りにされている所（切り口）が全部これに当たる。

    そこで **物体角**（object angle）で選り分ける。骨格点 p の最大内接円が
    境界に触れる点を近傍の最近傍境界画素から拾い、**ほぼ真向かい
    (>= `_ANG_MIN` 度) に 2 点で触れている所だけ**を「部材の幅」とみなす。
    角に押し込まれた円は 1 方向にしか触れないので落ちる。これは中軸の
    「object angle による枝刈り」そのもので、閾値以外に調整するものが無い。

    ⚠ 触れ方が真向かいの組を 1 つも持たない形（正三角形のように、どの内接円も
      3 辺に 120° で触れる形）では最大内接円の直径を返す。これは「その塊の
      いちばん太い所」であって最小幅ではないが、そういう形に細い部材は無い。

    ⚠ **60° 以下の鋭い凸角が残っていると、その頂点をラスタのギザギザが拾って
      最小幅を 1〜2mm と誤報する。** 既定の `smooth=2.0` は凸角を R2 に丸める
      のでこの状況は起きない。`smooth=0` で呼ぶときだけ注意すること。
      90° の角（矩形・L 字）は誤らない（実測: 無垢板 100.00mm / L 字 20.00mm）。
    """
    none_loc: tuple[float, float] | None = None
    if poly.is_empty or poly.area <= 0:
        return (0.0, none_loc) if loc else 0.0
    if px is None:
        px = _px_for(poly)
    m, x0, y0 = _raster(poly, px)
    if not m.any():
        return (0.0, none_loc) if loc else 0.0

    # return_indices で「各画素の最近傍の背景画素」も貰う。これが接点になる。
    edt, ind = distance_transform_edt(m, sampling=px, return_indices=True)
    sk = _thin(m)
    idx = np.argwhere(sk)
    if idx.size == 0:
        v = float(2.0 * edt.max())
        return (v, none_loc) if loc else v

    ii, jj = idx[:, 0], idx[:, 1]
    ny, nx = m.shape
    # 自分と 8 近傍、計 9 個ぶんの接点を集める。1 画素の中軸は近傍ごとに
    # 別々の境界を向くので、これで「どの向きに触れているか」が揃う。
    F = []
    for dy, dx in [(0, 0)] + _NB8:
        a = np.clip(ii + dy, 0, ny - 1)
        b = np.clip(jj + dx, 0, nx - 1)
        F.append(np.stack([ind[0][a, b], ind[1][a, b]], axis=1))
    F = np.stack(F, axis=1).astype(float)              # (N, 9, 2)
    P = np.stack([ii, jj], axis=1).astype(float)       # (N, 2)
    V = F - P[:, None, :]
    nrm = np.linalg.norm(V, axis=2)
    nrm[nrm == 0] = 1.0
    U = V / nrm[:, :, None]
    # 9 方向のうち一番開いている 2 つの角度。cos が小さいほど開いている。
    cmin = np.einsum("nid,njd->nij", U, U).reshape(len(idx), -1).min(axis=1)

    e = edt[ii, jj]
    ok = (cmin <= math.cos(math.radians(_ANG_MIN))) & (e >= 2.5 * px)
    if not ok.any():
        v = float(2.0 * e.max())
        return (v, none_loc) if loc else v
    k = int(np.argmin(np.where(ok, e, np.inf)))
    v = float(2.0 * e[k])
    if loc:
        return v, (x0 + (jj[k] + 0.5) * px, y0 + (ii[k] + 0.5) * px)
    return v


# ===========================================================================
# 本体
# ===========================================================================
def to_outline(res: TopoResult, level: float = 0.5,
               simplify: float = 0.4, smooth: float = 2.0,
               seats: bool = True) -> Outline:
    """密度場から切り抜き輪郭を作る。

    `level`    … しきい値。0.5 が標準。上げると痩せ、下げると太る。
    `simplify` … Douglas-Peucker の許容誤差 [mm]。格子の階段を消す。
    `smooth`   … 角の丸め半径 [mm]。0 で丸めなし。

    ⚠ 丸めは **クローズ（膨張→収縮）→ オープン（収縮→膨張）** の順で掛ける。
      内側の凹角（部材の付け根）に鋭角が残ると、そこに応力が集中して
      アルミが裂ける。オープンを先に掛けると、半径 `smooth` の 2 倍より
      細い部材が**先に消えてから**膨らむので、リブが 1 本無くなる。
      順番を守った上でも痩せるものは痩せるので、前後で面積と最小幅を測って
      5% 以上変わったら警告する（実測: 幅 3mm のリブを smooth=2 で丸めると
      本当に切れて、連結成分 1 → 2 と面積 -50.8% の 2 本が飛ぶ）。

    ⚠ 連結成分が 2 つ以上あっても**例外にはしない**。面積最大のものだけ返して
      `n_parts` に個数を入れるので、呼ぶ側が `check` で落とすこと。ここで
      例外を投げると、最適化のループが 1 枚失敗しただけで全部止まる。

    直径 `MIN_HOLE_DIA` 未満の穴は簡略化のゴミなので、この中で埋めて警告する。
    """
    reg = res.region
    nx, ny = reg.grid()
    cell = min(reg.w / nx, reg.h / ny)

    geom = _material(_rings(res, level), cell)
    parts = _parts(geom)
    if not parts:
        warnings.warn(f"[{reg.name}] 等高線が取れなかった（密度場が空か、"
                      f"level={level} が高すぎる）")
        return Outline(outer=[], holes=[], area=0.0, min_width=0.0, n_parts=0)

    n_raw = len(parts)
    poly = max(parts, key=lambda p: p.area)     # 面積最大の連結成分だけ残す

    if simplify > 0:
        # preserve_topology=True … 穴が外形を突き抜けて自己交差するのを止める
        poly = poly.simplify(simplify, preserve_topology=True)
        poly = poly.buffer(0)
        poly = max(_parts(poly) or [poly], key=lambda p: p.area)

    # 最小幅の測定はラスタを切るので重い（板 1 枚 0.2〜3 秒）。丸めの前後で
    # 1 回ずつだけ測り、後のほうをそのまま Outline.min_width にする。
    px = _px_for(poly)
    mw = _min_width(poly, px)

    if smooth > 0:
        a0, w0 = poly.area, mw
        r = float(smooth)
        closed = poly.buffer(+r, quad_segs=_QSEG).buffer(-r, quad_segs=_QSEG)
        opened = closed.buffer(-r, quad_segs=_QSEG).buffer(+r, quad_segs=_QSEG)
        cand = _parts(opened)
        if not cand:
            warnings.warn(f"[{reg.name}] smooth={smooth}mm で形が消えた。"
                          f"丸めを掛けずに返す")
        else:
            if len(cand) > n_raw:
                warnings.warn(f"[{reg.name}] 丸めで連結成分が {n_raw} → "
                              f"{len(cand)} に増えた（細い部材が切れた）")
                n_raw = len(cand)
            poly = max(cand, key=lambda p: p.area)
            # ⚠ 丸めは 4 回の buffer なので、角 1 つにつき円弧の点が湧く
            #   （実測 321 点 → 1577 点）。そのまま STEP にすると 1 枚で
            #   13MB になり、JSON も 5 万文字を超えて差分が読めない。
            #   `smooth/20` の誤差で間引く（R2 なら 0.1mm ＝ レーザーの
            #   カーフより細かいので、切った形は変わらない）。
            post = min(simplify, smooth / 20.0)
            if post > 0:
                poly = poly.simplify(post, preserve_topology=True).buffer(0)
                poly = max(_parts(poly) or [poly], key=lambda p: p.area)
            mw = _min_width(poly, px)
            a1, w1 = poly.area, mw
            da = abs(a1 - a0) / a0 if a0 > 0 else 0.0
            dw = abs(w1 - w0) / w0 if w0 > 0 else 0.0
            if da >= SMOOTH_TOL or dw >= SMOOTH_TOL:
                warnings.warn(
                    f"[{reg.name}] 丸め(r={r}mm)で "
                    f"面積 {a0:.0f}→{a1:.0f}mm² ({da*100:+.1f}%) / "
                    f"最小幅 {w0:.2f}→{w1:.2f}mm ({dw*100:+.1f}%)。"
                    f"5% 以上動いている＝形が変わっているので smooth を見直すこと")

    # --- 小さすぎる穴を捨てる ---------------------------------------------
    keep, dropped = [], 0
    for ring in poly.interiors:
        h = Polygon(ring)
        # 等価直径。細長い三日月も面積で見れば落ちる。
        if 2.0 * math.sqrt(h.area / math.pi) < MIN_HOLE_DIA:
            dropped += 1
            continue
        keep.append(ring)
    if dropped:
        warnings.warn(f"[{reg.name}] 直径 {MIN_HOLE_DIA}mm 未満の穴を "
                      f"{dropped} 個捨てた（簡略化のゴミ。加工しても軽くならない）")
        poly = Polygon(poly.exterior, keep)
        # 穴を埋めた分だけ材料が増えるので測り直す（埋めた穴の縁が最小幅の
        # 場所だったときに、報告値と実物がずれるのを防ぐ）
        mw = _min_width(poly, px)

    # --- 座面は必ず輪郭の中に入れる ---------------------------------------
    # ⚠ **丸めは板の縁にある座面の角を落とす。** 半径 2mm の丸めで角 1 個
    #   あたり 0.86mm²、細長い座面の両端なら 25mm² 以上が輪郭の外へ出る。
    #   そのままだと「ねじが留められない（致命的）」が全部の板で出て、
    #   1 枚も使えない。座面は**物理的に必ず材料がある場所**なので、
    #   丸めたあとで足し戻すのが正しい（削るのは最適化の仕事だが、
    #   座面を削ってよいとは一度も言っていない）。
    # ⚠ 逃げ穴は座面より優先。足し戻しで穴が埋まると軸が通らない。
    # ⚠ **拘束座（fixed）と荷重座（loads）も足し戻す。** 足し戻していたのは
    #   ねじ座面（solid）だけだったので、丸めが縁を波打たせた所で
    #   「拘束座が輪郭の外に 10.7mm² はみ出している」が残っていた。実測すると
    #   yaw_side_L の欠けは x=-115 の縁に沿った幅 1.91mm の帯 2 本（ボルトと
    #   ボルトの間で R2 の丸めが内側へ食い込んだ形）で、狙って直せるものでは
    #   ない。しかし**拘束座も荷重座も、相手と当たる＝材料が要る場所**という
    #   点はねじ座面と全く同じなので、閾値をいじるのではなく同じ扱いにする。
    seat_rects: list[Any] = list(reg.solid or [])
    seat_rects += list(reg.fixed or [])
    seat_rects += [ld[0] for ld in (reg.loads or [])]
    if seats and seat_rects:
        add = unary_union([sbox(*r) for r in seat_rects])
        if reg.void or reg.void_rect:
            add = add.difference(_voids(reg))
        # ⚠ **足し戻しは設計領域の中だけ。** 座面は矩形でしか書けないので、
        #   元が三角形の板では斜辺をまたいだ矩形がそのまま材料になる。
        #   ソルバは `core.dead_mask`（逃げ ∪ 設計領域の外）を見て外側に
        #   材料を置かないのに、輪郭を作るここだけが見ていなかった。
        #   実際ガセットで、密度場は三角形の中だけなのに輪郭は外接矩形の
        #   88% まで戻り、元の直角三角形より 79% 重い形が出た。
        # ⚠ 切ったあとに**鋭いくさび**が残る。矩形の座面を斜めの縁で切れば
        #   必ず出るもので、そこを `check` が「最小部材幅 1.4mm」と読む。
        #   くさびは座面ではない（ねじの頭はとっくに板の外）ので、
        #   RIB_MIN の円が通らない所は落とす。
        # ⚠ **くさび落としは「実際に切られた座面」にだけ掛ける（2026-08-05）。**
        #   `add` 全体を R3 で開くと、切られていない座面まで**端が丸まる**。
        #   実測 `yaw_side_L` の拘束座は 7×45mm の帯で、domain の中に
        #   すっぽり入っている（切られていない）のに、開処理で端が R3 に
        #   丸められて 9.1mm² 欠け、「相手に留められない」で落ちていた。
        #   角 1 個の欠けは (1−π/4)·3² = 1.93mm²、帯の端で 4 個なら 7.7mm²。
        #   → domain の外へはみ出した座面が 1 つも無いなら、開処理は要らない。
        if reg.domain:
            dom_poly = Polygon(reg.domain)
            # ⚠ **くさび落としは「切られた座面」にだけ掛ける（2026-08-05）。**
            #   `add` 全体を R3 で開くと、domain の中に収まっている座面まで
            #   痩せる。実測 `yaw_side_L` は拘束座 7×45mm の帯が
            #   269.1 → 215.1mm² に削られ（−54mm²）、そこから 9.1mm² が
            #   最終輪郭の外へ出て「相手に留められない」で落ちていた。
            #   この板は別の座面が domain の外へ 747.9mm² 出ているので、
            #   「はみ出しの有無」を板単位で見ると必ず開処理が掛かる。
            #   → **座面ごと**に、はみ出していないものは無傷で残す。
            #   ⚠ **座面ごとに無傷で戻す案は駄目だった。** 矩形の角がそのまま
            #     輪郭に出て、`_min_width` のラスタが 1.72mm（＝ピクセル 6 個
            #     ぶんの偽値）を返し、今度は「最小部材幅」で落ちた。
            #     欠けの側（`check`）で「縁の細片は数えない」と扱うのが正しい。
            add = add.intersection(dom_poly)
            rw = RIB_MIN / 2.0
            slim = add.buffer(-rw, quad_segs=_QSEG).buffer(rw, quad_segs=_QSEG)
            if not slim.is_empty:
                add = slim
        merged = poly.union(add)
        # ⚠ union で島が増えることがある（座面が本体から離れている板）。
        #   そのときは**黙って捨てない**。`check` の連結成分で落とす。
        n_raw = max(n_raw, len(_parts(merged)))
        poly = max(_parts(merged), key=lambda g: g.area)
        mw = _min_width(poly, px)
    # ⚠ **逃げ穴は最後にもう一度くり抜く。** 丸め（膨張→収縮）は穴も一緒に
    #   縮めるので、R11 の穴なら縁に 14〜22mm² の材料が戻ってくる。密度場では
    #   ちゃんと空いていたのに、輪郭にすると軸が板に当たる形になっていた。
    #   円は多角形に近似されるので、分割数を上げて誤差を 0.1% 未満にする。
    if reg.void or reg.void_rect:
        poly = poly.difference(_voids(reg))
        if isinstance(poly, MultiPolygon):
            n_raw = max(n_raw, len(_parts(poly)))
            poly = max(_parts(poly), key=lambda g: g.area)

    # 向きを揃える: 外形 CCW / 穴 CW（core.Outline の約束）
    poly = orient(poly, sign=1.0)

    area = float(poly.area)
    return Outline(
        outer=[(float(x), float(y)) for x, y in poly.exterior.coords[:-1]],
        holes=[[(float(x), float(y)) for x, y in r.coords[:-1]]
               for r in poly.interiors],
        area=area,
        min_width=mw,
        n_parts=n_raw,
    )


def _voids(reg: Region):
    """逃げ（円 + 矩形）をひとつの面にまとめる。

    ⚠ 円は多角形に近似される。既定の分割数だと R11 で 0.6%（2.3mm²）内側に
      入るので、くり抜きに使うと縁に材料が残る。32 分割で 0.03% 未満。
    """
    gs = [SPoint(cx, cy).buffer(rr, quad_segs=32) for cx, cy, rr in reg.void]
    out = unary_union(gs) if gs else Polygon()
    if reg.void_rect:
        # ⚠ **矩形の逃げは座面に譲る**（`core.dead_mask` と同じ理由）。
        #   相手の外接箱がねじの座面まで伸びていることがあり、そのまま
        #   抜くと座面が欠けて留められなくなる。円の逃げ（軸）は譲らない。
        soft = unary_union([sbox(*r) for r in reg.void_rect])
        if reg.solid:
            soft = soft.difference(unary_union([sbox(*r) for r in reg.solid]))
        out = unary_union([out, soft])
    return out


def plain_outline(reg: Region) -> Outline:
    """**最適化しない**板の輪郭（設計領域そのまま、逃げ穴だけ開けた形）。

    ⚠ 最適化で使える形が出なかった板のための逃げ道（2026-08-05）。
      それまでは「使えない解は JSON に書かない」だけだったので、
      **前の（境界条件が違う）輪郭がそのまま残って DXF になっていた**。
      消せばよいかというとそうでもなく、`tr_lib.topo_plate` は輪郭が
      無いと例外を投げる（黙って矩形に落ちるのを禁じている）ので、
      組立そのものが作れなくなる。実際 3 枚消したら `export_meshes` が
      `FileNotFoundError: topo_plate(yaw_side_L): 輪郭が無い` で落ちた。

    → **素の板を「輪郭」として書く**のが正しい。重いが必ず切れるし、
      逃げ穴は開いているので軸も通る。`check` も通る（座面は全部
      領域の中にあり、逃げ穴は抜いてあり、部材幅は板そのもの）。
    """
    dom = (Polygon(reg.domain) if getattr(reg, "domain", None)
           else sbox(-reg.w / 2, -reg.h / 2, reg.w / 2, reg.h / 2))
    if reg.void or reg.void_rect:
        dom = dom.difference(_voids(reg))
    if isinstance(dom, MultiPolygon):
        dom = max(_parts(dom), key=lambda g: g.area)
    dom = orient(dom, sign=1.0)
    return Outline(
        outer=[(float(x), float(y)) for x, y in dom.exterior.coords[:-1]],
        holes=[[(float(x), float(y)) for x, y in r.coords[:-1]]
               for r in dom.interiors],
        area=float(dom.area),
        min_width=float(_min_width(dom, _px_for(dom))),
        n_parts=1,
    )


_PLATE_MINW: dict[str, float] = {}


def plate_min_width(reg: Region, area_min: float = 10.0,
                    eps: float = 0.05) -> float:
    """**元の板が既に持っている細り**の幅（mm）。上限は `RIB_MIN`。

    ⚠ 最適化の結果に `RIB_MIN`(6mm) を課すのは、**元の板がその基準を
      満たしている場合だけ**正しい。実測 `yaw_side_L` は素の板の時点で
      板の縁（x=115）と M5 の逃げ穴のあいだが **4.56mm** しかない。
      拘束座がそこにある以上、最適化がどんな形を出してもこの首は残る。
      6mm を要求し続ける限り**絶対に通らない**（実際 rmin を 6 → 32、
      frac を 0.30 → 0.70 まで振っても 4.60mm から動かず、板 3 枚が
      最適化を諦めて素の板になっていた）。

    ⚠ **過去に一度これを入れて外している。** 理由は「ガセットの元の外形が
      最小幅 1.4mm と出る。細いリブではなく**三角形の頂点**を拾っている
      だけで、下限にすると何でも通る」。
      → いまは 2 つの手当てが入っているので同じことは起きない:
        ・頂点は `topo_opt._blunt` が設計領域の段階で落としている
        ・ここでは**面積 `area_min`(=10mm²) 以上の塊だけ**を細りとみなす
          （頂点は面積 0.0mm² の塊として出る）

    ⚠ `area_min` は実測で分離できる所に置いた。`yaw_side_L` の細りは
      面積で **51.6 / 31.9 / 4.0 / 3.1 / 2.0 mm²** に分かれ、上 2 つが
      本物の首（縁と逃げ穴のあいだ、5.94mm と 4.56mm）、下 3 つは
      設計領域の折れ線の階段（3.1mm² のものは 204mm×2.27mm の薄い帯）。
      10mm² は 31.9 と 4.0 のあいだで、10 倍の開きがある。
      **通るまで動かした値ではない。**

    測り方: 幅 w 未満の場所は `poly - poly.buffer(-w/2).buffer(w/2)` に
    残る（開処理で消える部分）。残る w を二分探索する。
    ⚠ `_min_width` のラスタは使わない。素の板のような縁がぎざぎざの
      多角形では**ピクセル 6 個ぶん**を返し、細かくすると値が半分になる
      （実測 0.287mm → 1.725mm / 0.144mm → 0.862mm / 0.072mm → 0.431mm）。
      幾何で測れば分解能に依らない。
    """
    if reg.name in _PLATE_MINW:
        return _PLATE_MINW[reg.name]
    poly = _poly_of(plain_outline(reg))

    def thin_at(w: float) -> bool:
        r = w / 2.0
        op = poly.buffer(-r, quad_segs=16).buffer(r, quad_segs=16)
        dif = poly.difference(op)
        gs = (list(dif.geoms) if isinstance(dif, MultiPolygon)
              else ([dif] if not dif.is_empty else []))
        return any(g.area >= area_min for g in gs)

    lo, hi = 0.5, float(RIB_MIN)
    if not thin_at(hi):
        _PLATE_MINW[reg.name] = hi
        return hi
    while hi - lo > eps:
        mid = (lo + hi) / 2.0
        if thin_at(mid):
            hi = mid
        else:
            lo = mid
    _PLATE_MINW[reg.name] = lo
    return lo


def _poly_of(out: Outline):
    """`Outline` を shapely の Polygon に戻す（検査と面作りの共通の入口）。"""
    if len(out.outer) < 3:
        return Polygon()
    p = Polygon(out.outer, [h for h in out.holes if len(h) >= 3])
    if not p.is_valid:
        p = p.buffer(0)
    return p


def check(out: Outline, reg: Region, rib_min: float | None = None) -> list[str]:
    """製造・構造上の不良を文字列で返す。空なら合格。

    `rib_min` … 最小部材幅の下限。既定は `core.RIB_MIN`（6mm）。
    ⚠ **元の板がすでにそれより細い所を持っているなら、下限を下げること。**
      ねじ穴の縁の残肉が 4mm の板（ガセット・press_post）は、その 4mm が
      設計領域の一部として渡ってくる。密度 1 固定なのでフィルタが掛からず、
      いくら解き直しても 4mm のまま「切れない」と言われ続ける。
      元がそう作られている以上、それは最適化のせいではない。

    ⚠ ここが**最後の関門**。ここを通った形はそのまま切られる。
      「たぶん大丈夫」で通すくらいなら落とすこと。落ちた板は
      `frac` を上げるか `rmin` を上げて解き直せばよいだけで、失うものは無い。

    ⚠ ここで見ていないもの（見ていると勘違いしないこと）:
      * 応力（`TopoResult.vm` は渡ってこない。強度は呼ぶ側で見る）
      * `EDGE_MIN`（ねじ穴の縁の残肉）— ねじ穴を開けるのは組立側なので、
        輪郭だけでは判定できない
      * 3D の干渉 — 板の外に何があるかは知らない
      * `n_parts != 1` のとき、検査するのは**面積最大の連結成分だけ**。
        捨てられた島の中に座があっても「座面がはみ出している」としか出ない。
    """
    msg: list[str] = []
    poly = _poly_of(out)
    if poly.is_empty:
        return [f"{reg.name}: 輪郭が空（最適化が収束していない）"]

    if out.n_parts != 1:
        msg.append(f"{reg.name}: 連結成分が {out.n_parts} 個。"
                   f"板がばらばらなので組めない（frac を上げて解き直す）")

    # ⚠ 既定の下限は `RIB_MIN` ではなく「**元の板が既に持っている細り**」。
    #   元が 4.56mm の板に 6mm を課すと、最適化がどんな形を出しても
    #   通らない（`plate_min_width` の説明を読むこと）。元が 6mm 以上なら
    #   そのまま `RIB_MIN` になる。
    lim = plate_min_width(reg) if rib_min is None else rib_min
    if out.min_width < lim:
        msg.append(f"{reg.name}: 最小部材幅 {out.min_width:.2f}mm < "
                   f"{lim:.1f}mm。切れないか、切れても座屈する"
                   f"（rmin を上げて解き直す）")

    # --- 座面が輪郭の中に完全に入っているか -------------------------------
    # ⚠ 1mm でもはみ出したら不合格にする。座面が欠けた板はねじが留まらない＝
    #   その板は**使えない**。面積が足りているとか、ほぼ入っているとかは無関係。
    #   contains は境界上を含まないので、微小な数値誤差ぶんだけ緩める。
    eps = 1e-6
    soft = poly.buffer(eps)
    # ⚠ **座面は設計領域で切ってから問うこと。** 座面の矩形はねじの外接箱を
    #   `SEAT_GROW`(=10mm) 広げたもので、板の縁の近くにあるねじでは
    #   **板の外まではみ出す**。板の外に材料は置けないので、そのはみ出しを
    #   「輪郭が覆っていない」と数えると、**どんな輪郭でも直らない指摘**に
    #   なる。実際これで 13 枚 334 件の「ねじが留められない（致命的）」が
    #   出ていて、そのうち gus_brace_L_lo の 15.8mm² は「板の縁 x=-55 に対し
    #   座面が -56.75 まで伸びている 1.75mm 帯」＝設計領域の外だけだった。
    #   足し戻し（上の `seats` の分岐）は最初から `reg.domain` で切っている
    #   ので、**検査だけが切っていなかった**。
    #   `domain` が None（元が長方形の板）でも、外接矩形の外には置けない。
    dom = (Polygon(reg.domain) if getattr(reg, "domain", None)
           else sbox(-reg.w / 2, -reg.h / 2, reg.w / 2, reg.h / 2))
    # ⚠ **逃げ穴も同じ理由で引く。** 足し戻しは `add.difference(_voids(reg))`
    #   で穴を優先し、最後にもう一度 `poly.difference(_voids(reg))` で穴を
    #   開け直す（軸が通らなくなるので当然）。ところが検査は座面の**生の
    #   矩形**を輪郭と比べていたので、**わざと開けた穴の面積がそのまま
    #   「座面がはみ出している」として出ていた**。
    #   beltbrk_idl_R の 23.8mm² は void[1] (r=2.8) の π·2.8²=24.6mm² を
    #   矩形で切った値そのもの。yaw_side の 15.9 / 12.3 / 8.4 も r=2.2 の
    #   円を縁で切った値。設計領域のときと同じ「直しようのない指摘」。
    if reg.void or reg.void_rect:
        dom = dom.difference(_voids(reg))

    def _rects(name: str, rects: Iterable[Any], fatal: str,
               shrink: float = 0.0) -> None:
        for k, rc in enumerate(rects):
            r = sbox(*rc)
            if shrink > 0.0:
                # ⚠ **問うべきは「ねじの頭が覆われているか」。**
                #   ねじ座面は頭の外接箱を `topo_opt.SEAT_GROW`(=10mm) 広げた
                #   領域で、頭そのもの（M5 で φ8.5）は中心にある。縁が
                #   数 mm² 欠けても頭には届かないのに、生の矩形で問うと
                #   角の丸めや逃げ穴のたびに「ねじが留められない」が出る。
                #   面積の閾値（`SEAT_EPS` / `SEAT_REL`）で切ろうとしたが、
                #   実測の分布は 2.0〜10.9mm² と連続していて切れなかった。
                #   **10mm 縮めて頭の領域に戻してから問う**のが素直。
                core = r.buffer(-shrink, join_style=2)
                if core.is_empty:
                    continue          # 座面が成長ぶんより小さい＝頭は中央に収まる
                r = core
            r = r.intersection(dom)
            if r.is_empty or r.area <= 0:
                continue
            if not soft.contains(r):
                # ⚠ **欠けの「形」で見る（2026-08-05）。** 面積だけで見ると、
                #   座面の**縁に沿った細い帯**（足し戻しのくさび落としが
                #   RIB_MIN/2 で削る分）と、**まとまって欠けた塊**（本当に
                #   留められない）が区別できない。実測 `yaw_side_L` の
                #   拘束座は 9.1mm² 欠けるが、中身は 2.67mm 幅の帯 1 本で、
                #   ボルト 4 本はいずれも座面の上に載っている。
                #   → 欠けを RIB_MIN/2 で開いて、**細片が消えた残り**を問う。
                #     本物の欠け（座面の端がまるごと無い）は開いても残る。
                gap = r.difference(poly)
                if not gap.is_empty:
                    rw = RIB_MIN / 2.0
                    solidgap = gap.buffer(-rw, quad_segs=_QSEG).buffer(
                        rw, quad_segs=_QSEG)
                    lost = 0.0 if solidgap.is_empty else solidgap.area
                else:
                    lost = 0.0
                # ⚠ **輪郭の角の丸めが落とす分で不合格にしてはいけない。**
                #   座面を足し戻しても、そのあとの `smooth`（既定 R2）が
                #   凸角を丸めるので、矩形の角は必ず少し欠ける。1 つの角で
                #   (1 − π/4)·r² = 0.86mm²。座面の矩形が輪郭の縁に接する
                #   ときは 2 角までが落ちるので、そこまでは許す。
                #   ⚠ 前は 0.05mm² で切っていた（丸め誤差だけを想定した値）。
                #     実測すると角の欠けは 0.1〜1.5mm² 出るので、13 枚とも
                #     「ねじが留められない」が残り続けた。本物のはみ出しは
                #     5.9〜176mm² だったので、ここで綺麗に分かれる。
                if lost < max(SEAT_EPS, SEAT_REL * r.area):
                    continue
                msg.append(f"{reg.name}: {name}[{k}] {rc} が輪郭の外に "
                           f"{lost:.1f}mm² はみ出している。{fatal}")

    # ⚠ `SEAT_GROW` は `topo_opt` 側の定数（= `plate_audit.SCREW_SPREAD` = 10）。
    #   import すると循環するので値を写している。片方を変えたらここも直すこと。
    _rects("solid（ねじ座面）", reg.solid, "ねじが留められない（致命的）",
           shrink=10.0)
    _rects("fixed（拘束座）", reg.fixed, "相手に留められない")
    _rects("loads（荷重座）", [ld[0] for ld in reg.loads], "力を受ける座が無い")

    # --- 逃げ穴に材料が残っていないか -------------------------------------
    # ⚠ ここは「板が軸に当たる」＝組めない。干渉検査より先にここで落とす。
    for k, (cx, cy, rr) in enumerate(reg.void):
        c = Polygon([(cx + rr * math.cos(t), cy + rr * math.sin(t))
                     for t in np.linspace(0, 2 * math.pi, 64, endpoint=False)])
        ov = poly.intersection(c).area
        if ov > 0.01 * c.area:
            msg.append(f"{reg.name}: void[{k}] (x={cx:.1f}, y={cy:.1f}, "
                       f"r={rr:.1f}) に材料が {ov:.1f}mm² 残っている"
                       f"（軸／配線が板に当たる）")
    for k, rc in enumerate(reg.void_rect):
        # 座面に譲った分は「残っていて当然」なので除いて見る
        r = sbox(*rc)
        if reg.solid:
            r = r.difference(unary_union([sbox(*s) for s in reg.solid]))
        if r.is_empty or r.area < 1.0:
            continue
        ov = poly.intersection(r).area
        if ov > 0.01 * r.area:
            msg.append(f"{reg.name}: void_rect[{k}] {rc} に材料が "
                       f"{ov:.1f}mm² 残っている（宣言に無い相手が板に当たる）")

    # --- 小さすぎる穴（to_outline で落とし損ねたもの）----------------------
    # ⚠ **宣言した逃げ穴は数えない。** `to_outline` は小穴を捨てた**あとで**
    #   `poly.difference(_voids(reg))` を掛ける（丸めが穴を縮めるので開け直す
    #   必要がある）。順番がこうなので、宣言した逃げ穴は構造上ここを必ず
    #   生き残る。それを「埋めること」と言われても、埋めたらねじが通らない。
    #   実測: 16 枚 32 個の指摘は**全部** `reg.void` と中心ずれ 0.00mm で
    #   一致し、径も φ3.5 / φ4.5 / φ5.5 ＝ M3 / M4 / M5 のバカ穴そのものだった。
    #   落とすべきは「宣言に無い」小穴（簡略化のゴミ）だけ。
    decl = _voids(reg) if (reg.void or reg.void_rect) else None
    small = []
    for h in out.holes:
        if len(h) < 3:
            continue
        hp = Polygon(h)
        if not hp.is_valid:
            hp = hp.buffer(0)
        if hp.is_empty or hp.area <= 0:
            continue
        # 宣言の外にはみ出している分で見る。逃げ穴どうしが繋がった 1 個の
        # リングでも、宣言で説明できない部分が小さければゴミではない。
        if decl is not None and hp.difference(decl).area < 0.10 * hp.area:
            continue
        d = 2.0 * math.sqrt(hp.area / math.pi)
        if d < MIN_HOLE_DIA:
            small.append(d)
    if small:
        msg.append(f"{reg.name}: 直径 {MIN_HOLE_DIA}mm 未満の穴が "
                   f"{len(small)} 個残っている（最小 {min(small):.1f}mm）。"
                   f"加工しても軽くならないので埋めること")
    return msg


# ===========================================================================
# build123d
# ===========================================================================
def to_face(out: Outline):
    """`Outline` を build123d の `Sketch`（板ローカル XY 平面、Z=0）にする。

    返ってきた面はそのまま `extrude(face, amount=t)` できる。押し出しの向きは
    面の法線＝+Z。実測（build123d 0.11.1）: X 字を extrude(3) した solid の
    体積は `Outline.area * 3` と **誤差 0.0000%** で一致した。

    ⚠ `Polygon(..., align=None)` を必ず付ける。既定の `align` は図形を原点に
      **寄せ直す**ので、座標をそのまま渡したつもりが板ごと平行移動して、
      ねじ穴の位置が全部ずれる。実測で bbox が輪郭の x[-100,100] y[-50,50] と
      一致することを確かめてある。
    """
    from build123d import Polygon as _BdPolygon      # 遅延 import（重い）

    if not out.outer:
        raise ValueError("Outline が空。to_face できない")
    sk = _BdPolygon(*[(float(x), float(y)) for x, y in out.outer], align=None)
    for h in out.holes:
        sk = sk - _BdPolygon(*[(float(x), float(y)) for x, y in h], align=None)
    return sk


# ===========================================================================
# JSON（キャッシュ）
# ===========================================================================
def _r3(pts) -> list[list[float]]:
    return [[round(float(x), 3), round(float(y), 3)] for x, y in pts]


def to_json(out: Outline) -> dict:
    """輪郭を JSON にできる dict にする。座標は小数点以下 3 桁。"""
    return {
        "outer": _r3(out.outer),
        "holes": [_r3(h) for h in out.holes],
        "area": round(float(out.area), 3),
        "min_width": round(float(out.min_width), 3),
        "n_parts": int(out.n_parts),
    }


def from_json(d: dict) -> Outline:
    """`to_json` の dict から `Outline` を復元する。"""
    return Outline(
        outer=[(float(x), float(y)) for x, y in d["outer"]],
        holes=[[(float(x), float(y)) for x, y in h] for h in d.get("holes", [])],
        area=float(d.get("area", 0.0)),
        min_width=float(d.get("min_width", 0.0)),
        n_parts=int(d.get("n_parts", 1)),
    )


def cache_path(name: str, root: str | Path = "out/topo") -> Path:
    return Path(root) / f"{name}.json"


def save(out: Outline, name: str, root: str | Path = "out/topo") -> Path:
    """輪郭を `out/topo/<板名>.json` に落とす。"""
    p = cache_path(name, root)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(to_json(out), ensure_ascii=False, indent=1),
                 encoding="utf-8")
    return p


def load(name: str, root: str | Path = "out/topo") -> Outline | None:
    """キャッシュを読む。無ければ None（呼ぶ側が解き直す合図）。"""
    p = cache_path(name, root)
    if not p.exists():
        return None
    return from_json(json.loads(p.read_text(encoding="utf-8")))
