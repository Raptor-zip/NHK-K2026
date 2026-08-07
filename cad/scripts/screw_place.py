"""宣言されたねじの**置き場所**を幾何から決めて凍結する.

    python scripts/screw_place.py [--md] [--only <部品名>]

なぜ要るか
-----------
`src/tr_fix.py` の固定宣言は 326 組あり、そのうち **220 組・622 本のねじが
「留まっている」と書いてあるだけで図に無かった**（`scripts/fastener_bom.py`
の「未実体」）。買う数と質量は宣言から見積もれていたが、

  * **組めるかどうかが分からない。** ねじの頭が座る面が本当にあるのか、
    工具が入るのか、相手を貫いていないかは、実体を置かないと言えない
  * 図を見た人には「留まっていない」ように見える

ここでやること
---------------
組んだ実体（`validate.solids_with_bbox`）から、宣言された 2 部品の
**接触面**と、その面で**両方に材料がある領域**を求め、ねじを通せる点を選ぶ。

  1. bbox の重なりが最も薄い軸を接触の法線とする
  2. 法線方向に ±0.4mm ずらした 2 点が、それぞれの実体の**内部**にあるか
     を格子で判定する（`BRepClass3d_SolidClassifier`）。
     ⚠ **bbox の重なり矩形をそのまま使ってはいけない。** トポロジー
       最適化した板は外形が矩形と全く違うので、隅にねじを置くと空中に浮く。
  3. 頭の座面ぶん（頭半径 + 1mm）だけ有効域を**収縮**する
  4. 残った点から、互いに最も離れる n 点を貪欲に選ぶ（両端・四隅に散る）

結果は `out/screws.json` に凍結する。組立側（`tr_assembly.py`）はこれを
読んで置くだけ。**組立 → 位置計算 → 組立** の循環を断つのが凍結の目的で、
`out/topo/` の輪郭と同じ考え方。

⚠ 位置は部品の**形と場所**にしか依らない。ねじを足したことで形は変わらない
  ので、凍結した位置は次の組立でもそのまま使える。部品を動かしたら
  `scripts/screw_check.py` が指紋の違いで気づく。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time as _time

import numpy as np

_T0 = _time.time()

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

OUT = os.path.join(ROOT, "out", "screws.json")

# 格子ピッチ [mm]。細かくすると内点判定の回数が二乗で増える。
# 2mm あれば M3 の頭（φ5.5）でも座面の判定に足りる。
GRID = 2.0
# 接触面から実体の中へ探りを入れる深さ [mm]。板厚 2mm の部材でも
# 中に入る値にする（薄すぎると面の公差で外れ、厚すぎると薄板を突き抜ける）。
PROBE = 0.4
# 接触とみなす bbox の重なり [mm]。負（離れている）側にも公差を見る。
TOUCH_TOL = 1.0


def _bb(b):
    """bbox を（合成できるように）素の値で持ち直す。"""
    from types import SimpleNamespace as NS
    return NS(min=NS(X=b.min.X, Y=b.min.Y, Z=b.min.Z),
              max=NS(X=b.max.X, Y=b.max.Y, Z=b.max.Z))


def _merge_bb(a, b):
    from types import SimpleNamespace as NS
    return NS(min=NS(X=min(a.min.X, b.min.X), Y=min(a.min.Y, b.min.Y),
                     Z=min(a.min.Z, b.min.Z)),
              max=NS(X=max(a.max.X, b.max.X), Y=max(a.max.Y, b.max.Y),
                     Z=max(a.max.Z, b.max.Z)))


def _axis_names(b):
    return ((b.min.X, b.max.X), (b.min.Y, b.max.Y), (b.min.Z, b.max.Z))


# ラベルパス → リンク名。**順序が意味を持つ**（pitch は turret の中、
# press / fork は grabber の中にあるので、内側から先に判定する）。
LINK_BY_PATH = (
    ("/pitch_group/", "pitch"), ("/shooter_pitch/", "pitch"),
    ("/roller/", "pitch"),
    ("/turret/", "turret"), ("/turret_yaw/", "turret"),
    ("/singulator/", "singulator"),
    ("/grabber_press/", "press"), ("/fork/", "fork"), ("/fork_tilt/", "fork"),
    ("/grabber_slide/", "carriage"), ("/grabber/", "carriage"),
    ("/slide_rails", "rails"),
)


def link_of(path: str) -> str:
    for w in ("fl", "fr", "rl", "rr"):
        if f"/wheel_{w}/" in path:
            return f"wheel_{w}"
    for key, lk in LINK_BY_PATH:
        if key in path:
            return lk
    return "base"


def overlaps(ba, bb):
    """各軸の bbox 重なり量。負なら離れている。"""
    A, B = _axis_names(ba), _axis_names(bb)
    return [min(A[i][1], B[i][1]) - max(A[i][0], B[i][0]) for i in range(3)]


def face_candidates(ba, bb, k: int):
    """軸 `k` を法線としたときの、**面座標の候補**と a が +側かの並び。

    ⚠ 突き合わせ（板と板）なら境界は 1 つだが、**嵌合**（軸受が板に入る、
      ハブが車輪に入る）では 3 軸とも厚く重なり、境界の中点は材の中に来る。
      この場合ねじが通るのは相手へ**差し込んだ側の端面**なので、
      a の端面も候補にする（実際、ハブ ↔ 車輪の 4-M5 はここでしか拾えない）。
    """
    A, B = _axis_names(ba), _axis_names(bb)
    plus = A[k][0] + A[k][1] > B[k][0] + B[k][1]
    out = [((A[k][0] + B[k][1]) / 2 if plus else (A[k][1] + B[k][0]) / 2, plus)]
    if min(A[k][1], B[k][1]) - max(A[k][0], B[k][0]) > 1.0:
        out.append((A[k][0], False))      # a の −側の端面（a は +側にある）
        out.append((A[k][1], True))       # a の +側の端面
    return out


class Inside:
    """ソリッドの内部判定（点が中にあるか）。

    `strict=True` なら**面の上は数えない**。ねじの頭が隣の部品に「触れて
    いる」のと「めり込んでいる」のは別で、触れるだけなら組める。
    ⚠ 面も内部として数えて頭の当たりを見たら、置けるねじが 544 → 425 本に
      減った。座面の隣に別の部品がある箇所は普通にあり、そこを全部
      「置けない」にするのは行きすぎ。
    ⚠ **1 つの部品が複数のソリッドで出来ていることがある。** 代表の 1 個
      だけ見ていたので、スカート（前後で別ソリッド）に 24mm³ 食い込んで
      いるのを拾えなかった。部品まるごとを 1 つの判定にする。
    """

    def __init__(self, solids, strict: bool = False):
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        if not isinstance(solids, (list, tuple)):
            solids = [solids]
        self.cls = [BRepClass3d_SolidClassifier(s.wrapped) for s in solids]
        self.strict = strict

    def __call__(self, p) -> bool:
        return self.state(p) == 2 if self.strict else self.state(p) > 0

    def many(self, pts):
        """(N,3) をまとめて。GPU 版と同じ API にするための受け皿。"""
        import numpy as _np
        return _np.fromiter((self(list(map(float, q))) for q in pts),
                            dtype=bool, count=len(pts))

    def state_many(self, pts):
        import numpy as _np
        return _np.fromiter((self.state(list(map(float, q))) for q in pts),
                            dtype=_np.int8, count=len(pts))

    def state(self, p) -> int:
        """0=外 / 1=面の上 / 2=中（複数ソリッドのうち最も強いもの）。"""
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON
        pt = gp_Pnt(*p)
        out = 0
        for c in self.cls:
            c.Perform(pt, 1e-7)
            st = c.State()
            if st == TopAbs_IN:
                return 2
            if st == TopAbs_ON:
                out = 1
        return out


# --- 内外判定の実装を選ぶ -------------------------------------------------
# ⚠ **OCC の分類は 1 点ごとの C++ 呼び出し**（実測 50µs/点）。ここは
#   数十万回呼ばれるので、配置時間の大半を占める。形は判定の途中で
#   変わらないので、三角メッシュにして GPU でまとめて解く
#   （`src/gpu_geom.py`。実測 9µs/点、点をまとめるほど下がる）。
#   `TR_GPU=0` で OCC に戻せる。**答えが変わっていないかは
#   `scripts/gpu_bench.py` で見ること**（実測 0.011% / すべて境界の揺れ）。
USE_GPU = os.environ.get("TR_GPU", "1") != "0"
try:
    if USE_GPU:
        sys.path.insert(0, os.path.join(ROOT, "src"))
        import gpu_geom as _G
        USE_GPU = _G.HAVE_TORCH
except Exception:                        # noqa: BLE001
    USE_GPU = False

_INS_CACHE: dict = {}


def inside_of(ss, strict: bool = False):
    """部品のソリッド列 → 内外判定。**同じリストなら作り直さない。**

    ⚠ GPU 版は三角メッシュを作るので、`blocked()` のように部品ごとに
      毎回作ると、メッシュ化だけで潰れる。リストの同一性でキャッシュする
      （`id` の使い回しを踏まないよう、リスト自身も持っておく）。
    """
    key = (id(ss), strict)
    hit = _INS_CACHE.get(key)
    if hit is not None and hit[0] is ss:
        return hit[1]
    ins = _G.GpuInside(ss, strict) if USE_GPU else Inside(ss, strict)
    _INS_CACHE[key] = (ss, ins)
    return ins


def _seg_dist(a0, a1, b0, b1) -> float:
    """線分どうしの最短距離。ねじの軸が交差していないかを見るのに使う。"""
    u = [a1[i] - a0[i] for i in range(3)]
    v = [b1[i] - b0[i] for i in range(3)]
    w = [a0[i] - b0[i] for i in range(3)]
    dot = lambda p, q: sum(p[i] * q[i] for i in range(3))    # noqa: E731
    a, b, c = dot(u, u), dot(u, v), dot(v, v)
    d, e = dot(u, w), dot(v, w)
    den = a * c - b * b
    if den < 1e-9:
        s, t = 0.0, (e / c if c > 1e-9 else 0.0)
    else:
        s = max(0.0, min(1.0, (b * e - c * d) / den))
        t = max(0.0, min(1.0, (a * e - b * d) / den))
    return math.dist([a0[i] + s * u[i] for i in range(3)],
                     [b0[i] + t * v[i] for i in range(3)])


def _center_line(ok, nx, ny):
    """有効域から**中心線**だけを残す。

    ⚠ **収縮しきった帯で縁を残してはいけない。** 幅が座面より狭い帯
      （耳・リブ・小口）は「線」でしか置けないのに、以前は収縮前の格子を
      そのまま返していた。`_spread()` は互いに最も離れる点を選ぶので縁が
      当たり、**ねじの穴が材料の側面を破る**（`car_beam_eL` は t6 の耳に
      M4 の芯が 1mm 寄って、φ4.5 の下穴が側面を 0.25mm 突き抜けていた）。
      押出材の溝を「線」として扱う `slot_of()` と同じ考え方で、狭いほうの
      軸は中央に寄せる。
    """
    out = [[False] * ny for _ in range(nx)]
    # 各行・各列で連続している長さの最大値を比べ、**狭いほうの軸**を畳む
    span_i = max((sum(1 for i in range(nx) if ok[i][j]) for j in range(ny)),
                 default=0)
    span_j = max((sum(1 for j in range(ny) if ok[i][j]) for i in range(nx)),
                 default=0)
    if span_j <= span_i:
        for i in range(nx):
            js = [j for j in range(ny) if ok[i][j]]
            if js:
                out[i][js[len(js) // 2]] = True
    else:
        for j in range(ny):
            iss = [i for i in range(nx) if ok[i][j]]
            if iss:
                out[iss[len(iss) // 2]][j] = True
    return out


def _erode(ok, nx, ny, times: int):
    """有効格子を `times` 回収縮する（縁から離す）。"""
    for _ in range(times):
        nxt = [[False] * ny for _ in range(nx)]
        for i in range(nx):
            for j in range(ny):
                if not ok[i][j]:
                    continue
                if (i and j and i + 1 < nx and j + 1 < ny
                        and ok[i - 1][j] and ok[i + 1][j]
                        and ok[i][j - 1] and ok[i][j + 1]
                        and ok[i - 1][j - 1] and ok[i + 1][j + 1]
                        and ok[i - 1][j + 1] and ok[i + 1][j - 1]):
                    nxt[i][j] = True
        if not any(any(r) for r in nxt):
            return _center_line(ok, nx, ny)   # 収縮しきったら中心線に寄せる
        ok = nxt
    return ok


def _spread(pts, n: int):
    """点群から互いに最も離れる n 点を選ぶ（貪欲・最遠点)。

    ⚠ 同じ点が 2 度選ばれないようにすること。候補が少ないと最遠点の
      評価が同点になり、**同じ座標のねじが 2 本**出る（頭が丸ごと重なって
      531mm³ の食い込みになった）。選んだ点は候補から外す。
    """
    if len(pts) <= n:
        return list(pts)
    rest = list(pts)
    cx = sum(p[0] for p in rest) / len(rest)
    cy = sum(p[1] for p in rest) / len(rest)
    first = max(rest, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
    out = [first]
    rest.remove(first)
    while len(out) < n and rest:
        nxt = max(rest, key=lambda p: min((p[0] - q[0]) ** 2
                                          + (p[1] - q[1]) ** 2 for q in out))
        out.append(nxt)
        rest.remove(nxt)
    return out


def depth_into(ins, k, c, sgn, ax, u, v, maxd: float = 30.0, step: float = 0.5):
    """接触面から相手の中へ、材料が続いている深さ [mm]。

    ⚠ **これを測らずに長さを決めると、ねじが相手を突き抜けて向こう側の
      部品に刺さる。** 41 本で試した時点で 19 件の食い込みが出て、その
      大半が「長すぎるねじが 3 つ目の部品を貫いた」ものだった。
    """
    # ⚠ **1 点ずつ聞かない。** 打ち切りがあるので逐次に見えるが、
    #   最大まで一括で聞いて「先頭から続く True の数」を数えれば同じ答えで、
    #   GPU なら 1 回の呼び出しで済む。
    n = int(maxd / step)
    if n <= 0:
        return 0.0
    q = np.zeros((n, 3), dtype=np.float64)
    q[:, ax[0]] = u
    q[:, ax[1]] = v
    q[:, k] = c - sgn * (np.arange(n) * step + step / 2)
    r = np.asarray(ins.many(q))
    off = np.nonzero(~r)[0]
    return float((off[0] if len(off) else n) * step)


# 実体を持たない「空間の予約」。ここを貫いても組めなくはならない。
# ⚠ マスコットの入る空間（CONTAIN）を実体と同じに扱うと、椅子まわりの
#   ねじが 6 本とも「置けない」になった。雑巾やマスコットは物であって
#   構造ではないので、ねじの通り道の判定からは外す。
VIRTUAL = ("mascot", "rag_", "bucket_water")


def blocked(sol, keep, p0, dvec, length: float, rad: float, step: float = 1.5):
    """`p0` から `dvec` の逆向きに `length` 進む軸が、`keep` 以外の材に当たるか。

    ⚠ 粗ふるいは bbox（速い）、確定は内点判定（正確）。全部品に内点判定を
      掛けると 600 × 点数で保たない。
    """
    lo = [min(p0[i], p0[i] - dvec[i] * length) - rad for i in range(3)]
    hi = [max(p0[i], p0[i] - dvec[i] * length) + rad for i in range(3)]
    for nm, (ss, b) in sol.items():
        if (nm in keep or nm.startswith("scr_") or nm.startswith("cab_")
                or nm.startswith(VIRTUAL)):
            continue
        if (b.max.X < lo[0] or b.min.X > hi[0] or b.max.Y < lo[1]
                or b.min.Y > hi[1] or b.max.Z < lo[2] or b.min.Z > hi[2]):
            continue
        ts = np.arange(step, length, step)
        if not len(ts):
            continue
        q = np.asarray(p0, dtype=np.float64)[None, :] \
            - np.asarray(dvec, dtype=np.float64)[None, :] * ts[:, None]
        if np.asarray(inside_of(ss).many(q)).any():
            return nm
    return None


def _basis(dvec):
    """`dvec` を +Z とする正規直交基底 (dz, ux, uy)。

    ⚠ 軸に平行なときだけを想定して「成分が 0 の軸」を横方向にしていたが、
      **旋回した姿勢では方向が斜めになる**（成分が 0 の軸が無くなり
      IndexError で落ちた）。外積で作れば向きに依らない。
    """
    n = math.sqrt(sum(v * v for v in dvec)) or 1.0
    dz = [v / n for v in dvec]
    t = [1.0, 0.0, 0.0] if abs(dz[0]) < 0.9 else [0.0, 1.0, 0.0]
    ux = [t[1] * dz[2] - t[2] * dz[1], t[2] * dz[0] - t[0] * dz[2],
          t[0] * dz[1] - t[1] * dz[0]]
    m = math.sqrt(sum(v * v for v in ux)) or 1.0
    ux = [v / m for v in ux]
    uy = [dz[1] * ux[2] - dz[2] * ux[1], dz[2] * ux[0] - dz[0] * ux[2],
          dz[0] * ux[1] - dz[1] * ux[0]]
    return dz, ux, uy


def head_hits(sol, keep, p, dvec, head_r: float, head_h: float):
    """ねじの**頭**が `keep` 以外の部品に当たるなら、その名前を返す。

    ⚠ 軸の通り道だけ見ていたので、頭が隣の部品に**触れている**組が 61 件
      出た（「未宣言接触」）。座金や頭は軸より太いので、外周も見る。
    """
    dz, ux, uy = _basis(dvec)
    r = max(1.0, head_r - 0.3)            # 面で触れるぶんは見ない
    pts = []
    for t in (0.5, head_h * 0.5, head_h):
        for a in range(8):
            c, s = math.cos(a * math.pi / 4), math.sin(a * math.pi / 4)
            pts.append([p[i] + dz[i] * t + r * (c * ux[i] + s * uy[i])
                        for i in range(3)])
    lo = [min(q[i] for q in pts) for i in range(3)]
    hi = [max(q[i] for q in pts) for i in range(3)]
    for nm, (ss, b) in sol.items():
        if (nm in keep or nm.startswith("scr_") or nm.startswith("cab_")
                or nm.startswith(VIRTUAL)):
            continue
        if (b.max.X < lo[0] or b.min.X > hi[0] or b.max.Y < lo[1]
                or b.min.Y > hi[1] or b.max.Z < lo[2] or b.min.Z > hi[2]):
            continue
        if (np.asarray(inside_of(ss, True).state_many(pts)) == 2).any():
            return nm
    return None


def _disc(r: float, step: float = 1.0):
    """半径 `r` の円板を `step` 格子で埋めた (du, dv) の並び（中心を含む）。"""
    out = [(0.0, 0.0)]
    n = int(r / step)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            if i or j:
                if (i * step) ** 2 + (j * step) ** 2 <= r * r:
                    out.append((i * step, j * step))
    if r > 0:                              # 縁そのものも 8 方向で押さえる
        for a in range(8):
            out.append((r * math.cos(a * math.pi / 4),
                        r * math.sin(a * math.pi / 4)))
    return out


def neighbors(sol, keep, p, dvec, d: float, length: float, head_r: float,
              head_h: float, r_sh: float | None = None):
    """このねじが (めり込む相手, 面で触れる相手) を返す。

    ⚠ **弾ききれなかったぶんは、隠さず宣言に出す。** 候補を全部試しても
      3 つ目の部品に当たる箇所は残る（柱の角をかすめる・既にあるねじと
      交差する）。そこを黙って置くと「宣言のない接触」「食い込み」として
      検査に出るだけで、何が起きているか分からない。**貫くなら貫くと
      宣言する**（実機ではその部材に穴が要る、という意味になる）。
    """
    ax = [i for i in range(3) if dvec[i] == 0]
    pts = []
    t = 0.0
    # ⚠ 軸は**中心線だけでは足りない**。側面が相手の面に接している組が
    #   30 件残った（中心は相手から d/2 離れているので、どのサンプル点も
    #   相手の外側に出る）。外周も一緒に見る。
    # ⚠ **接している相手は「ねじの表面より少し外」でないと拾えない。**
    #   表面ちょうどの点で見ていたので、0.1〜0.44mm 離れた（＝検査では
    #   接触と数える）相手が 20 件ぶん宣言から漏れた。内側の点で「貫く」を、
    #   外側の点で「触れる」を見る。NEAR は assembly_check の接触公差と同じ。
    NEAR = 0.7
    # ⚠ **溝ナット付きは末端が φ11 に太る。** 軸の呼び径で見ていたので、
    #   ナットの縁がブラケットに食い込む 23〜26mm³ が 14 件、宣言から
    #   漏れていた。太いほうで探る。
    rs = max((d / 2 if d > 1.0 else 0.0), (r_sh or 0.0)) - 0.1
    outer = []
    # ⚠ **円周を何本引いても薄板の縁は捉えられない。** t0.8 のスカートに
    #   頭が 24mm³ 食い込んでいるのを、半径を 3 段にしても見逃した
    #   （どの半径の円周も、板の厚み 0.8mm の帯を跨いでしまう）。
    #   断面を **1mm 格子で塗りつぶす**。ここは採用が決まったねじにしか
    #   走らないので、点数を増やしても全体の時間には効かない。
    while t <= length:                    # 軸（座面から先端へ）
        q0 = [p[i] - dvec[i] * t for i in range(3)]
        for du, dv in _disc(rs):
            q = list(q0)
            q[ax[0]] += du
            q[ax[1]] += dv
            pts.append(q)
        for du, dv in _disc(rs + NEAR):
            q = list(q0)
            q[ax[0]] += du
            q[ax[1]] += dv
            outer.append(q)
        t += 1.0
    for t in (0.5, head_h * 0.5, head_h) if head_h else ():
        q0 = [p[i] + dvec[i] * t for i in range(3)]
        for r, dst in ((head_r - 0.2, pts), (head_r + NEAR, outer)):
            for du, dv in _disc(r):
                q = list(q0)
                q[ax[0]] += du
                q[ax[1]] += dv
                dst.append(q)
    outer.append([p[i] - dvec[i] * (length + NEAR) for i in range(3)])  # 先端
    outer.append([p[i] + dvec[i] * (head_h + NEAR) for i in range(3)])  # 頭の上
    lo = [min(q[i] for q in pts + outer) - 1.0 for i in range(3)]
    hi = [max(q[i] for q in pts + outer) + 1.0 for i in range(3)]
    thru, touch = [], []
    # ⚠ ここでは **scr_ を除外しない**。弾く判定（blocked）では無視してよい
    #   ものでも、実体として重なっているなら宣言は要る。実際、既にあるねじ
    #   との重なりが「宣言なしの食い込み」として残っていた。
    # ⚠ VIRTUAL（マスコット・雑巾）は除く。**後から載せる物**で、しかも
    #   発泡と布なので、ねじ頭が沈むぶんはそのまま逃げになる。
    #   ここを見ていた頃は、椅子まわりのねじ 1 本ごとにマスコットの
    #   曲面ソリッドで内点判定が走り、配置に 35 分かかっていた。
    for nm, (ss, b) in sol.items():
        if nm in keep or nm.startswith(VIRTUAL):
            continue
        if (b.max.X < lo[0] or b.min.X > hi[0] or b.max.Y < lo[1]
                or b.min.Y > hi[1] or b.max.Z < lo[2] or b.min.Z > hi[2]):
            continue
        ins = inside_of(ss)
        st = np.asarray(ins.state_many(pts + outer))
        if (st[:len(pts)] == 2).any():
            thru.append(nm)
        elif (st > 0).any():
            touch.append(nm)
    return thru, touch


def _to_pose(p, dvec, org0, loc):
    """関節 0 の世界座標 `p`（向き `dvec`）を、別姿勢の同じリンク上へ移す。

    ⚠ **引き算だけでは移せない。** 関節 0 ではどのリンクも並進だけなので
      `p − org` がそのままリンクローカルになるが、旋回した姿勢では回転が
      入る。ローカルへ戻してから Location を掛ける（向きも同じだけ回る）。
      位置だけ足し引きしていたので、旋回体のねじを「ヨー ±30° でも見る」と
      書きながら**実際には見ていなかった**（台座リングへの 135mm³ が
      連続掃引にだけ出続けた）。
    """
    from build123d import Pos as _Pos
    pl = [p[i] - org0[i] for i in range(3)]
    a = (loc * _Pos(*pl)).position
    b = (loc * _Pos(pl[0] + dvec[0], pl[1] + dvec[1],
                    pl[2] + dvec[2])).position
    return [a.X, a.Y, a.Z], [b.X - a.X, b.Y - a.Y, b.Z - a.Z]


def ext_axes(b, slot_bottom: float, w_nom: float):
    """押出材の bbox から (長手軸, 断面中心, 溝底の深さ) を返す。

    ⚠ **アルミフレームは面のどこにでもねじを打てるわけではない。**
      HFS5-2020 は 4 面それぞれに溝が **1 条だけ**、面の**中心線**に通って
      いる。留められるのはその線の上だけで、線から外れた場所はただの
      平らな押出材の肌（下穴が無い）。
      どちらが長手かは bbox の辺の長さで決まる（断面 20 × 長さ L）。

    ⚠ **斜材には使えない。** 傾いた押出材は bbox が断面より太くなるので、
      bbox の中心線は溝の中心線ではない（そこを溝だと思って置くと、
      いまより悪い場所にねじが並ぶ）。判定できないので None を返し、
      呼ぶ側は従来どおり面で散らす。斜材は 2 本だけで、ガセットは
      そもそも面ではなく**角**に当たっている。
    """
    d = [b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z]
    la = max(range(3), key=lambda i: d[i])
    if max(d[i] for i in range(3) if i != la) > w_nom + 1.5:
        return None
    ctr = [(b.min.X + b.max.X) / 2, (b.min.Y + b.max.Y) / 2,
           (b.min.Z + b.max.Z) / 2]
    return la, ctr, slot_bottom


def slot_of(ext, k: int, ax):
    """法線 `k` の面で、ねじが乗れる線を `ax` 上の拘束 [(index, 座標)] で返す。

    側面（`k` != 長手軸）… 溝は 1 条なので、**幅方向は中心に固定**する。
      並べられるのは長手方向だけ。ここを自由にすると「溝が 2 条あって
      2 列に打てる」という、実物には無いフレームを描くことになる。
    端面（`k` == 長手軸）… 溝は無い。あるのは中心の下穴（φ4.3、M5 用）
      **1 つだけ**なので、断面の 2 軸とも中心に固定する。
    """
    la, ctr, _ = ext
    if k == la:
        return [(0, ctr[ax[0]]), (1, ctr[ax[1]])]
    i_ = 0 if ax[0] != la else 1
    return [(i_, ctr[ax[i_]])]


def place_pair(sa, sb, ba, bb, n: int, head_r: float, ext=None,
               d_nom: float = 4.0):
    """向きと面を試して、**ねじを通せる点が最も多い**置き場所を返す。

    ⚠ 薄い軸から試し、十分な候補が出たら打ち切る。3 軸 × 面候補を毎回
      全部見ると内点判定が 5 倍になり、220 組で保たない。

    `ext` … 留める相手が押出材のときの `ext_axes()`。溝の上にしか置けない。
    """
    ov = overlaps(ba, bb)
    if min(ov) < -TOUCH_TOL:
        return None                       # どこかの軸で離れている＝接していない
    best = None
    order = sorted(range(3), key=lambda i: ov[i])
    if ext is not None:
        # ⚠ 端面は中心穴 1 つしか無い。**側面（溝のある面）を先に試す。**
        order.sort(key=lambda i: i == ext[0])
    for k in order:
        for c, plus in face_candidates(ba, bb, k):
            slot = slot_of(ext, k, [i for i in range(3) if i != k]) \
                if ext is not None else None
            got = _place_axis(sa, sb, ba, bb, n, head_r, k, c, plus, slot,
                              ext[2] if ext is not None else 0.0, d_nom)
            if got and (best is None or len(got[3]) > len(best[3])):
                best = got
            if best and len(best[3]) >= n * 4:
                return best
    return best


def _place_axis(sa, sb, ba, bb, n: int, head_r: float, k: int, c: float,
                plus: bool, slot=None, slot_bottom: float = 0.0,
                d_nom: float = 4.0):
    """法線 `k`・面 `c` での (k, c, plus_a, 候補 [(u,v)], 相手の内点判定)。

    `d_nom` … ねじの呼び径。**b 側に要るのは下穴 + 壁**で、頭の径ではない。
    """
    ax = [i for i in range(3) if i != k]
    A, B = _axis_names(ba), _axis_names(bb)
    lo = [max(A[i][0], B[i][0]) for i in ax]
    hi = [min(A[i][1], B[i][1]) for i in ax]
    if hi[0] - lo[0] < 1.0 or hi[1] - lo[1] < 1.0:
        return None                       # 線でしか当たっていない
    for i_, sc in (slot or []):
        # 溝（または端面の下穴）が重なり範囲の外＝この面では留められない
        if not (lo[i_] - 1.0 <= sc <= hi[i_] + 1.0):
            return None
        lo[i_] = hi[i_] = sc
    # ⚠ **格子は必ず中心を含むこと（＝分割数を奇数にする）。** 幅 6mm を
    #   GRID 2mm で割ると点は 4 つ（両端 + 2）で、**帯の中心が候補に無い**。
    #   耳のような狭い相手では、中心に置けないぶんが丸ごと壁の薄さになる。
    nx = 1 if hi[0] <= lo[0] else max(3, int((hi[0] - lo[0]) / GRID) + 1)
    ny = 1 if hi[1] <= lo[1] else max(3, int((hi[1] - lo[1]) / GRID) + 1)
    nx += (nx > 1 and nx % 2 == 0)
    ny += (ny > 1 and ny % 2 == 0)
    if nx * ny > 40_000:                  # 大きな面は粗く見る（内点判定が律速）
        step = math.ceil(math.sqrt(nx * ny / 40_000.0))
        nx, ny = max(3, nx // step), max(3, ny // step)
        nx += (nx % 2 == 0)               # 粗くしても中心は残す
        ny += (ny % 2 == 0)
    du = (hi[0] - lo[0]) / (nx - 1) if nx > 1 else 0.0
    dv = (hi[1] - lo[1]) / (ny - 1) if ny > 1 else 0.0
    ia, ib = inside_of(sa), inside_of(sb)
    sgn = 1.0 if plus else -1.0
    # ⚠ 押出材側は**溝の底より深いところ**で材料を見る。表面から 1.9mm は
    #   溝の開口で、そこは空。表面すぐを探ると「材料が無い」と出て、
    #   **溝の真上だけが候補から落ちる**（＝溝を外した場所にねじが並ぶ）。
    pb = (slot_bottom + 0.5) if slot else PROBE
    # 格子は**まとめて**引く（GPU なら 1 回、OCC でも呼び出しの形は同じ）
    gi, gj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    qa = np.zeros((nx * ny, 3), dtype=np.float64)
    qa[:, ax[0]] = lo[0] + gi.ravel() * du
    qa[:, ax[1]] = lo[1] + gj.ravel() * dv
    qb = qa.copy()
    qa[:, k] = c + sgn * PROBE
    qb[:, k] = c - sgn * pb
    ra = np.asarray(ia.many(qa)).reshape(nx, ny)
    rb = np.asarray(ib.many(qb)).reshape(nx, ny) & ra
    ok_a = [[bool(ra[i][j]) for j in range(ny)] for i in range(nx)]
    ok_b = [[bool(rb[i][j]) for j in range(ny)] for i in range(nx)]
    # ⚠ **収縮は a 側と b 側で別々にかける。** 以前は「両方に材料がある
    #   領域」をまとめて頭の座面ぶん収縮していたが、座面が要るのは**頭を
    #   入れる a 側だけ**。b 側に要るのは「下穴 + 壁」で、頭の径は関係ない。
    #   共通領域で収縮すると、a に十分な座面があっても b が細い（耳・リブ・
    #   小口）だけで候補が全滅し、フォールバックで縁に落ちていた。
    raw_a, raw_b = ok_a, ok_b
    ok_a = _erode(ok_a, nx, ny, max(1, int(round((head_r + 1.0) / GRID))))
    ok_b = _erode(ok_b, nx, ny, max(1, int(round((d_nom / 2 + 0.8) / GRID))))

    def _and(u, v):
        return [[u[i][j] and v[i][j] for j in range(ny)] for i in range(nx)]

    ok = _and(ok_a, ok_b)
    if not any(any(r) for r in ok):
        # ⚠ **座面は諦めても「頭を入れる側に材料がある」ことは譲れない。**
        #   ここを `ok_b` だけにしていたら、相手（椅子の座板のような広い面）
        #   に材料さえあれば通ってしまい、**枠のくり抜きの真上**にねじが
        #   並んだ（`chair_mount_*` の 4 本が肉抜き穴の上）。座面の広さは
        #   座金で稼げるが、材料が無い場所は座金でも埋まらない。
        ok = _and(raw_a, ok_b)
    if not any(any(r) for r in ok):
        # それでも無いなら、穴が開けられる側を優先する（壁の無い穴は
        # どうにもならない）。ここへ落ちた組は `screw_seat_check` に出る。
        ok = ok_b
    pts = [(lo[0] + i * du, lo[1] + j * dv)
           for i in range(nx) for j in range(ny) if ok[i][j]]
    if slot:
        # ⚠ 溝の上は**線**なので、格子の収縮（3×3 の近傍を要求する）は
        #   1 回で全部消える。線の両端から座面ぶんを手で削る。
        keep = [(u, v) for u, v in pts
                if (nx == 1 or lo[0] + head_r + 1.0 <= u <= hi[0] - head_r - 1.0)
                and (ny == 1 or lo[1] + head_r + 1.0 <= v <= hi[1] - head_r - 1.0)]
        pts = keep or pts
    if not pts:
        return None
    # ⚠ **候補は多めに返す。** ちょうど n 点だけ選ぶと、そのうち 1 点でも
    #   軸が 3 つ目の部品を貫いていたときに打つ手が無くなる（実際
    #   beltbrk ↔ rail_plate の 4 本がそれで置けなかった）。
    #   採否は呼ぶ側が軸の通り道を見て決める。
    # ⚠ 候補を n×6 にしていたら、頭の当たり・ねじ同士の重なりを見るように
    #   した途端に置ける本数が 544 → 430 に落ちた。判定は正しいので、
    #   **候補のほうを厚くして**逃げ場を作る。
    return k, c, plus, _spread(pts, n * 15), ib


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    import tr_assembly as A
    import tr_lib as L
    import tr_params as P
    import validate as V

    # ⚠ **ねじを置かない状態**で組む。置いてから位置を測ると、
    #   前回のねじが接触面の判定に混ざる（組立 → 位置 → 組立 の循環）。
    A.AUTO_SCREWS = False
    # ⚠ **全関節 0 の姿勢で測る。** 仰角が付いていると射出リンクの部品は
    #   傾き、bbox（軸平行）が実体からずれて接触面が読めない。関節が 0 なら
    #   どのリンクも並進だけなので、世界座標から**引き算だけ**でリンク
    #   ローカルに直せる。凍結する位置はリンクローカルでなければならない
    #   （そうでないと姿勢を変えたときねじだけ取り残される）。
    shape = A.build(dict(yaw=0.0, pitch=0.0, grab=0.0, press=0.0, tilt=0.0))
    # ⚠ 部品名は**ラベルパスの末尾ではなく、宣言にある名前**で引く。
    #   末尾で引くと、部品の中の無名ソリッドが別部品として辞書に入り、
    #   本体（skirt_front など）が丸ごと見えなくなる。
    import tr_fix as F
    known = set(F.BOX)
    sol, lk_of = {}, {}
    for n, sd, b in V.solids_with_bbox(shape):
        seg = [q.split("#")[0] for q in n.split("/")]
        nm = next((q for q in reversed(seg) if q in known), seg[-1])
        # ⚠ bbox は **`solids_with_bbox` のもの（世界座標）**を合成する。
        #   `tr_fix.BOX` は put した時点の座標なので、可動リンクの中の部品は
        #   リンクローカルで入っている。混ぜると接触面がまるごとずれる。
        if nm in sol:
            ss, bb0 = sol[nm]
            sol[nm] = (ss + [sd], _merge_bb(bb0, b))
        else:
            sol[nm] = ([sd], _bb(b))
        lk_of[nm] = link_of(n)
    org = {k: tuple(v.position) for k, v in A.LINK_LOC.items()}

    # ⚠ **姿勢を変えると当たる相手がいる。** 装填姿勢（グラバー伸長）では
    #   キャリッジのねじがレールのボール保持器に 143mm³ 食い込んでいた。
    #   関節 0 の 1 姿勢だけで場所を決めると、動かした先で当たる。
    # ⚠ 装填だけでは足りない。**旋回（ヨー ±30°）も見る。** 溝の上に
    #   移したねじが、旋回リングに 135mm³ 食い込んだ（連続掃引が拾った）。
    #   旋回体は base から見て 30° 回るので、関節 0 と装填はどちらも
    #   ヨー 0 ＝ その運動をまるごと見ていなかった。
    others = []            # [(その姿勢の実体, リンク原点)]
    for pose in (P.POSE_LOADING,
                 dict(yaw=+P.YAW_LIMIT, pitch=0.0, grab=0.0, press=0.0, tilt=0.0),
                 dict(yaw=-P.YAW_LIMIT, pitch=0.0, grab=0.0, press=0.0, tilt=0.0)):
        A._STATIC = None
        sh = A.build(pose)
        s2 = {}
        for n, sd, b in V.solids_with_bbox(sh):
            seg = [q.split("#")[0] for q in n.split("/")]
            nm = next((q for q in reversed(seg) if q in known), seg[-1])
            if nm in s2:
                ss, bb0 = s2[nm]
                s2[nm] = (ss + [sd], _merge_bb(bb0, b))
            else:
                s2[nm] = ([sd], _bb(b))
        # ⚠ **Location をそのまま持つ。** 位置だけ持って引き算で移すやり方は
        #   関節 0（並進だけ）でしか成り立たない。旋回した姿勢では回転が
        #   入るので、ローカル座標に Location を掛けて世界へ戻す。
        others.append((s2, dict(A.LINK_LOC)))
    # 位置計算そのものは関節 0 の組立で行う（bbox が軸平行になる姿勢）
    A._STATIC = None
    A.build(dict(yaw=0.0, pitch=0.0, grab=0.0, press=0.0, tilt=0.0))

    # ⚠ **内訳を出す。** どこが遅いかが分からないと、GPU にするべき場所も
    #   決められない（組み立て＝OCC のブーリアンは GPU では速くならない）。
    print(f"[時間] 組立 4 姿勢 {_time.time() - _T0:6.1f}s  "
          f"（判定 {'GPU' if USE_GPU else 'OCC'}）", file=sys.stderr)
    _t_build = _time.time()
    _j, _p, missing, _s, _sm = L.fastener_joints()
    rows, fails = [], []
    placed_pts: list[tuple[list[float], float]] = []   # (世界座標, 頭半径)
    # ⚠ **組立側で手置きしたねじも避ける。** モーターのボルト円のように
    #   「買った部品が位置を決める」締結は `tr_assembly` が直接置いている。
    #   `blocked()` は名前が `scr_` で始まるものを素通りさせるので、ここで
    #   避け先に入れておかないと、自動配置のねじが手置きのねじに重なる。
    for _nm, (_k0, _d0, _ln0, _ex0) in F.FASTENER.items():
        _tl = F.TOOL.get(_nm)
        if _tl is None:
            continue
        _p0, _dv = list(_tl[:3]), list(_tl[3:])
        _hh = 0.0 if _k0 in ("FLAT", "RIVET_FLAT") else (
            1.2 if _k0 == "RIVET" else L.SCREW_HEAD[int(_d0)][1])
        # ⚠ 半径の出し方は下の本体と**同じ式**にすること（RIVET だけ /2 しない）。
        _hr = max(2.0, _d0) if _k0 == "RIVET" else (
            L.rivet_flat_head(_d0) if _k0 == "RIVET_FLAT" else
            L.FLAT_HEAD_DIA[int(_d0)] if _k0 == "FLAT" else
            L.SCREW_HEAD[int(_d0)][0]) / 2
        _rs = max(_d0 / 2, L.POST_NUT_DIA / 2
                  if any(e[0] == "TNUT" for e in _ex0) else 0.0)
        placed_pts.append((_nm, (
            (_p0, [_p0[i] + _dv[i] * max(_hh, 1.0) for i in range(3)], _hr),
            (_p0, [_p0[i] - _dv[i] * _ln0 for i in range(3)], _rs))))
    # 組立の段数。工具が入るかを見るとき、**そのねじより後に組む部品**は
    # まだ存在しないので通す（`scripts/assembly_check.py` の D4 と同じ考え方）。
    dep = F.depth_of(set(sol))
    seq = 0
    for a, b, how, note, n, size, rivet, flush in missing:
        if args.only and args.only not in (a, b):
            continue
        if a not in sol or b not in sol:
            fails.append((a, b, "実体が見つからない（Compound のまま？）"))
            continue
        lk = lk_of[a]
        if lk_of[a] != lk_of[b]:
            # ⚠ リンクをまたぐ締結は、関節そのものを固めることになる。
            #   摺動レールのように「動く側と固定側」で名前空間が分かれて
            #   いるだけの組もあるので、**動く側**に属させる（そちらに
            #   付いていれば姿勢を変えても一緒に動く）。
            lk = lk_of[a] if lk_of[a] != "base" else lk_of[b]
        if lk not in org:
            fails.append((a, b, f"リンク {lk} の変換が分からない"))
            continue
        kind, d, ln, extras = L.estimate_fastener(a, b, how, size, rivet, flush)
        # 頭を入れる側 … 押出材へは溝ナットなので**板側から**入れる。
        # どちらも板なら薄いほうから（頭が長く飛び出さない）。
        ta, tb = L.part_thickness(a), L.part_thickness(b)
        head_a = not L.is_extrusion(a) if L.is_extrusion(b) != L.is_extrusion(a) \
            else (ta <= tb)
        x, y = (a, b) if head_a else (b, a)
        sa, ba = sol[x]
        sb, bb = sol[y]
        head_r = max(2.0, d) if kind == "RIVET" else (
            L.rivet_flat_head(d) if kind == "RIVET_FLAT"
            else L.FLAT_HEAD_DIA[int(d)] if kind == "FLAT"
            else L.SCREW_HEAD[int(d)][0]) / 2
        # ⚠ 相手がアルミフレームなら、置けるのは**溝の上だけ**。
        #   面のどこにでも打てるつもりで散らすと、溝から外れたねじが並ぶ
        #   （下穴の無い肌に頭だけ乗っている状態）。
        # ⚠ **リベットは溝を使わない。** 押出材に打つときは面に下穴を開ける
        #   ので、溝の中心線に縛ると「溝の空を狙って打つ」ことになる。
        #   実際それで、材まで届かない長さになったリベット 27 本が
        #   押出材から浮いた。溝に縛るのは溝ナットで留めるものだけ。
        ext = ext_axes(bb, L.EXT_SLOT_BOTTOM, P.EXT_W) \
            if (L.is_extrusion(y) and not kind.startswith("RIVET")) else None
        got = place_pair(sa, sb, ba, bb, n, head_r, ext, d)
        if got is None:
            fails.append((a, b, "接触面に両方の材料がある場所が無い"
                                if ext is None else
                                "アルミフレームの溝の上に置ける場所が無い"))
            continue
        k, c, plus, cand, ib = got
        ax = [i for i in range(3) if i != k]
        sgn = 1.0 if plus else -1.0
        # 頭の座面 = 頭を入れる部品の材料が、接触面から**続いている端**。
        # ⚠ **bbox の外面を座面にしてはいけない。** 曲げ板・L 形・長い板は
        #   bbox が板厚よりずっと厚いので、頭が材料から離れて宙に浮く。
        #   以前は「ねじ 1 本ぶんの掴み代（12mm）で頭打ち」にして誤魔化して
        #   いたが、それでは t4 の耳に打つねじの頭が 8mm 浮いたままになる
        #   （ニップ入口ガイドの継ぎ耳がまさにそれだった。bbox は Y に
        #   200mm 伸びているので `abs(seat - c)` は 200 を返す）。
        #   相手側の深さと同じように、**点ごとに材料を実測する**。
        ia_ = inside_of(sa)
        o = org[lk]
        # ⚠ **後から組む部品は工具を塞がない。** 椅子の座板・ホッパーの側壁の
        #   ように「その締結を締めてから載せる」部品を数えると、実機では
        #   普通に組める締結が「レンチが入らない」で落ちる（`chair_mount_*`
        #   4 組がそれだった）。段数（締結の連鎖の深さ）で近似する。
        _my_d = max(dep.get(a, 0), dep.get(b, 0))
        tool_keep = {x, y} | {nm_ for nm_, dd_ in dep.items() if dd_ > _my_d}
        took, why = 0, ""
        for u, v in cand:
            if took >= n:
                break
            grip = min(max(1.0, depth_into(ia_, k, c, -sgn, ax, u, v,
                                           maxd=13.0)), 12.0)
            p = [0.0, 0.0, 0.0]
            p[ax[0]], p[ax[1]] = u, v
            p[k] = c + sgn * grip
            dvec = [0, 0, 0]
            dvec[k] = int(sgn)
            depth = depth_into(ib, k, c, sgn, ax, u, v)
            # ⚠ **長さは相手の実測厚みから決める。** 見積もりの長さを
            #   そのまま使うと、薄い相手を突き抜けて 3 つ目の部品に刺さる。
            if kind.startswith("RIVET"):
                ln2, how_ab, ex2 = max(4.0, round(grip + depth + 1.0)), \
                    ("RIVET", "RIVET"), []
                # ⚠ **買える長さを超えたら置かない。** リベットは掴み代に
                #   上限がある（φ3.2 で全長 25mm ほど）。ここに上限が無いと、
                #   接触の法線を取り違えた組から **φ3.2×43mm** のような
                #   買えないリベットが生まれ、その軸が別の部品を削る
                #   （`brk_lift ↔ lift_guide`。斜面で接しているので
                #   「bbox の重なりが薄い軸」では法線が取れない）。
                #   置かずに落とせば「置けなかった組」として表に出る。
                if ln2 > L.rivet_len_max(d):
                    why = (f"リベットが φ{d:g}×{ln2:.0f}mm になる"
                           f"（買える上限 {L.rivet_len_max(d):.0f}mm）。"
                           "斜めに接しているか、板の縁に打とうとしている")
                    continue
            elif L.is_extrusion(y):
                ln2, how_ab = L.screw_len(grip, 6.0), ("THRU", "TNUT")
                ex2 = [["TNUT", int(d)]]
            elif depth >= L.TAP_MIN.get(int(d), 6.0):
                # タップが立つ厚みがある → ねじ込み（2d 噛ませる）
                ln2 = L.screw_len(grip, min(depth - 1.0, 2.0 * d))
                # ⚠ **呼び長さへの切り上げで相手を突き抜けることがある。**
                #   `screw_len` は実在する長さへ切り上げるので、掴み代 +
                #   かみ合いが 17mm なら 18mm が返る。旋回プーリのねじが
                #   台座リングへ 6.9mm 出て、**旋回すると 135mm³ 削って
                #   いた**（連続掃引が拾った。ヨー 0 では当たらないので、
                #   置くときの通り道の判定では見えない）。
                #   突き抜けるなら 1 段短い規格長へ落とす（かみ合い 1d は残す）。
                if ln2 > grip + depth:
                    fit = [v for v in L.SCREW_LENGTHS
                           if grip + d <= v <= grip + depth]
                    if fit:
                        ln2 = float(fit[-1])
                how_ab, ex2 = ("THRU", "SCREW_IN"), [["WASHER", int(d)]]
            else:
                # 薄い → 貫通させてナット止め
                ln2 = L.screw_len(grip + depth, 3.0)
                how_ab, ex2 = ("THRU", "THRU"), [["HEXNUT", int(d)],
                                                 ["WASHER", int(d)]]
            # ⚠ **既に置いたねじと当たらないこと。** 同じ組の 2 本目だけで
            #   なく、隣の組のねじとも当たる（頭が丸ごと重なった 531mm³ の
            #   食い込みが 53 件出た）。頭の中心距離だけで見ると、**向きの
            #   違うねじが軸で交差する**のを見逃すので、軸を線分として
            #   最短距離を測る。
            # ⚠ **頭と軸は太さが違う。** 頭の半径（M5 で 4.25）を軸の判定に
            #   まで使うと、隣り合う穴に入るはずのねじが弾かれて置ける本数が
            #   443 → 392 に落ちた。頭は頭どうし、軸は軸どうしで測る。
            #   ⚠ **片方の頭ともう片方の軸**も当たる。ねじを「頭の円柱」と
            #     「軸の円柱」の 2 本の線分として、4 通りとも見る。
            # ⚠ 皿（FLAT / RIVET_FLAT）の頭は**板に沈む**ので、面から出る高さは 0。
            #   ここを 1.2 のままにすると、面一のはずの皿リベットが
            #   「1.2mm 出ている」ものとして隣のねじを弾く。
            kind2 = kind
            hh = 1.2 if kind == "RIVET" else (
                0.0 if kind in ("FLAT", "RIVET_FLAT") else L.SCREW_HEAD[int(d)][1])
            tail = [p[i] - dvec[i] * ln2 for i in range(3)]
            htop = [p[i] + dvec[i] * max(hh, 1.0) for i in range(3)]
            # ⚠ 溝ナット付きは末端が φ11 に太る。軸の半径で見ていたので、
            #   ナットどうしが 56mm³ 重なる組が残った。太いほうで測る。
            r_sh = max(d / 2, L.POST_NUT_DIA / 2
                       if any(e[0] == "TNUT" for e in ex2) else 0.0)
            me = ((p, htop, head_r), (p, tail, r_sh))
            # ⚠ **食い込むものだけ弾き、触れるだけのものは宣言に回す。**
            #   「触れたら置かない」にすると 424 → 394 本まで減る。実機では
            #   隣り合うねじの頭が当たっているのは普通のことなので、
            #   落とすのは体積が重なる場合だけにして、接触は TOUCH_OK と
            #   書いて残す（宣言のない接触は検査に出る、という筋は保つ）。
            near = [nm for nm, prev in placed_pts
                    if any(_seg_dist(s0, s1, t0, t1) < r0 + r1 + 0.6
                           for s0, s1, r0 in me for t0, t1, r1 in prev)]
            # ⚠ 線分＋半径はねじの外形の**近似**なので、ぴったり 0 で切ると
            #   実体では 53mm³ 重なる組が残る（頭の面取りぶん外形が張り出す）。
            #   0.2mm 余分に離す。
            if any(_seg_dist(s0, s1, t0, t1) < r0 + r1 + 0.2
                   for s0, s1, r0 in me
                   for nm, prev in placed_pts for t0, t1, r1 in prev):
                why = "既に置いたねじと食い込む"
                continue
            # 軸が a・b 以外の材を貫かないこと（3 つ目の部品への食い込み）
            hit = blocked(sol, {x, y}, p, dvec, ln2, d / 2)
            for s2, o2 in others:
                # 装填・旋回 ±30° でも同じ場所が空いていること
                if hit:
                    break
                if lk not in o2:
                    continue
                p2, d2 = _to_pose(p, dvec, o, o2[lk])
                hit = blocked(s2, {x, y}, p2, d2, ln2, d / 2)
            if hit:
                why = f"ねじの軸が {hit} を貫く（M{d:.0f}×{ln2:.0f}）"
                continue
            # 工具が入る空間（頭の外へ 18mm、六角レンチの太さぶん）
            # ⚠ 25mm × φ10 で見ていたら配線サドル 8 個が全部落ちた。
            #   短柄・ボール先のレンチは斜めからでも入るので、まっすぐの
            #   空間としてはこのくらいで足りる。厳しすぎる判定は
            #   「置けるはずのねじを置かない」という別の誤りになる。
            # ⚠ 軸線だけ見ていたので、レンチの**外周**が当たる 5 本が
            #   すり抜けた（インナーレールの縁）。中心と外周の 4 点で見る。
            def tool_hit(ss, pp, dv=None):
                dv = dvec if dv is None else dv
                _dz, _ux, _uy = _basis(dv)
                for ang in (None, 0, 1, 2, 3):
                    q0 = [pp[i] + _dz[i] * 18.0 for i in range(3)]
                    if ang is not None:
                        c = 6.0 * math.cos(ang * math.pi / 2)
                        s = 6.0 * math.sin(ang * math.pi / 2)
                        q0 = [q0[i] + c * _ux[i] + s * _uy[i] for i in range(3)]
                    h = blocked(ss, tool_keep, q0, [-q for q in _dz], 17.0, 3.5)
                    if h:
                        return h
                return None

            tool = tool_hit(sol, p)
            # ⚠ **工具は「どこかの姿勢で入れば」入る。** 軸の通り道と違って、
            #   レンチが要るのは組むときだけ。塞いでいるのが可動部なら
            #   そこをどけて締めればよい。1 姿勢だけで「入らない」と決めると、
            #   実機では普通に組める場所まで落ちる（キャリッジが伸びた姿勢なら
            #   空くのに、関節 0 で見てレール取付板の 12 本が全部落ちた）。
            for s2, o2 in others:
                if not tool or lk not in o2:
                    break
                p2, d2 = _to_pose(p, dvec, o, o2[lk])
                if not tool_hit(s2, p2, d2):
                    tool = None
            if tool:
                why = f"頭の外が {tool} に塞がれて工具が入らない"
                continue
            # ⚠ **頭は「留める相手」の中に埋まってもいけない。** keep から
            #   y を外す。嵌合の組では a の外面が b の内側にあることがあり、
            #   そこを座面にすると頭ごと b の中に入る（表示器の LED で
            #   515mm³ 埋まっていた）。座面が当たるのは a だけ。
            hit2 = head_hits(sol, {x}, p, dvec, head_r, hh) if hh else None
            for s2, o2 in others:
                if hit2 or not hh:
                    break
                if lk not in o2:
                    continue
                p2, d2 = _to_pose(p, dvec, o, o2[lk])
                hit2 = head_hits(s2, {x}, p2, d2, head_r, hh)
            if hit2 and kind2 == "CAP":
                # ⚠ **頭が当たるのは「置けない」ではなく「六角穴付きでは
                #   置けない」。** 座面のすぐ上に別の部品が載る継手
                #   （椅子の座板・斜路の側壁・フォークの櫛歯）は実機では
                #   皿ねじで頭を沈める。板厚が皿もみのぶんあるなら、
                #   ここで皿へ切り替えれば継手はそのまま成立する。
                sink = (L.FLAT_HEAD_DIA[int(d)] - d) / 2
                if grip >= sink + 1.0:
                    kind2, hh, hit2 = "FLAT", 0.0, None
            if hit2:
                why = f"頭が {hit2} に当たる"
                continue
            seq += 1
            took += 1
            nm_new = f"scr_a{seq:04d}"
            placed_pts.append((nm_new, ((list(p), list(htop), head_r),
                                        (list(p), list(tail), r_sh))))
            thru, touch = neighbors(sol, {x, y}, p, dvec, d, ln2, head_r, hh,
                                    r_sh)
            touch += near                 # 隣のねじと触れているぶん
            rows.append(dict(name=f"scr_a{seq:04d}", a=x, b=y, link=lk,
                             how=how, how_ab=list(how_ab),
                             kind=kind2, size=float(d), length=float(ln2),
                             extras=ex2,
                             pos=[round(p[i] - o[i], 3) for i in range(3)],
                             dir=dvec, grip=round(grip, 2),
                             depth=round(depth, 2), thru=thru, touch=touch,
                             note=note))
        if took < n:
            fails.append((a, b, f"{n} 本中 {took} 本しか置けない … {why}"))
        if args.limit and seq >= args.limit:
            break

    print(f"[時間] ねじの配置 {_time.time() - _t_build:6.1f}s "
          f"（{len(rows)} 本 / {len(missing)} 組）", file=sys.stderr)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fp:
        json.dump({"bc": bc_hash(), "screws": rows}, fp,
                  ensure_ascii=False, indent=1)

    if args.md:
        print("# ねじの自動配置\n")
        print(f"- 置いた {len(rows)} 本 / 未実体だった組 {len(missing)}")
        if fails:
            print(f"\n**置けなかった {len(fails)} 組**\n")
            for a, b, why in fails:
                print(f"- `{a}` ↔ `{b}` … {why}")
    else:
        print(f"置いた {len(rows)} 本 / 組 {len(missing) - len(fails)}"
              f" （置けなかった組 {len(fails)}）")
        for a, b, why in fails[:40]:
            print(f"  ⚠ {a} ↔ {b}: {why}")
    return 0


def bc_hash() -> str:
    """位置が依存しているもの（部品の bbox）の指紋。

    ⚠ `tr_fix.BOX` は **build() の後**でなければ埋まらない。呼ぶ側は
      組んでから呼ぶこと。
    """
    import hashlib
    import tr_fix as F
    h = hashlib.sha256()
    for nm in sorted(F.BOX):
        if nm.startswith("scr_"):
            continue                      # ねじ自身は指紋に含めない（循環する）
        # ⚠ VIRTUAL（マスコット・雑巾）も含めない。**位置の判定に使って
        #   いない**ものが指紋に入ると、飾りの形を直しただけで「凍結が
        #   古い」になり、15 分かけて同じ答えを出し直すことになる。
        if nm.startswith(VIRTUAL):
            continue
        b = F.BOX[nm]
        h.update(f"{nm}|{b.min.X:.2f},{b.min.Y:.2f},{b.min.Z:.2f},"
                 f"{b.max.X:.2f},{b.max.Y:.2f},{b.max.Z:.2f}".encode())
    return h.hexdigest()[:16]


if __name__ == "__main__":
    raise SystemExit(main())
