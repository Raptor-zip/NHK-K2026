"""板材のトポロジー最適化 — 「長方形から丸穴」をやめて、荷重の通る形に切る.

    python scripts/topo_opt.py --list            # 対象板と境界条件だけ（速い・解かない）
    python scripts/topo_opt.py                   # 全板を解いて out/topo/ に画像と JSON
    python scripts/topo_opt.py --only yaw_side_R # 1 枚だけ
    python scripts/topo_opt.py --md              # レビュー用 Markdown（画像へのリンクつき）

何が変わったか
--------------
これまでの `L.lighten` / `L.lighten_path` は、板の**外形を長方形のまま**にして
中に円を並べる方式だった。荷重が絶対に通らない**隅**が必ず残るので、
`plate_audit` の使用率は yaw_side で 24%、pitch_side で 46〜52% にとどまっていた。
4 分の 3 が「そこに板がある」以外の理由を持たない材料だった。

ここでは逆に解く。板の占めうる領域を全部材料で埋め、荷重を支えるのに
寄与しない要素から順に消す（SIMP + OC 法）。残るのは荷重経路そのものの形で、
隅は最初から残らない。**外形を切り直すのが目的**なので、丸穴は使わない。

境界条件はどこから来るか
------------------------
手で書かない。**`src/tr_fix.py` の固定宣言そのもの**から作る。しかも
**宣言の向き**をそのまま使う。

  固定 … `FIXINGS[板]` に載っている相手。＝「この板が留まっている先」。
         板から見れば、そこから先は動かない
  荷重 … `to=板` と宣言した相手。＝「この板にぶら下がっているもの」。
         その相手に**ぶら下がる質量**が、そのまま力になる
  座面 … ねじの当たり。絶対に消させない（消えたら締結できない）
  逃げ … 板を**貫通する**相手（THRU / ROTATE / SLIDE / PRESS）。必ず抜く

⚠ 最初これを `depth_of`（根からの段数）で分けようとして失敗した。段数は
  固定連鎖の木での深さで、**枝分かれの向きとは別物**。実際 pitch_side は
  段数で見ると「自分より浅い相手ゼロ」になり、支持が 1 つも無い板として
  落ちていた。宣言の向きはもともと「何に留めるか」を書いたものなので、
  そのまま荷重の上流／下流になる。
⚠ 手で座標を書くと、相手を動かしたときに境界条件だけ取り残されて、
  「もう力が来ていない場所に材料を残す」形が出てくる。宣言から作れば追従する。

扱えない板・入れていない荷重（承知の上で使う）
----------------------------------------------
⚠ **平面応力（面内）でしか解いていない。** 水平な板（厚み軸が Z）は重力を
  面外曲げで受けるので、面内 SIMP にかけると「荷重ゼロ」と判定されて
  ほぼ全部消える。そういう板は対象から外して理由を出す（`--list` で見える）。
  面外曲げの板をやるには Kirchhoff 板要素のソルバが別に要る。
⚠ **荷重は重力（動的倍率 3）と横加速度（1G）だけ。** 射出の反力、砲塔を
  振ったときの慣性トルク、モーターの反力トルクは入れていない。これは
  `plate_audit` / `fasteners` と**同じ荷重の考え方に揃える**ため。ここだけ
  別の荷重を入れると、「板厚は足りるが形が持たない」といった食い違いが
  検査の間で出る。射出反力を入れるなら全部の検査で同時にやること。
  → いま出る形は、**射出系の板（pitch_side）では非保守側**になりうる。
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

import plate_audit as PA  # noqa: E402
import topo_cache  # noqa: E402
from topo.core import (RIB_MIN, Region, dead_mask,  # noqa: E402
                       domain_mask, rect_mask)

OUT = os.path.join(ROOT, "out", "topo")

G = 9.80665

# ⚠ `plate_audit` と**同じ動的倍率**を使う。ここだけ別の数字にすると、
#   「板厚は足りているのに形が持たない」といった食い違いが出る。
DYN = 3.0            # 走行の突き上げ・射出反力
ACC_LAT = 1.0        # G 旋回・衝突の横加速度（面内に効く。板は横力に弱い）

# 板とみなす条件。plate_audit と揃える（別基準にすると対象がずれる）
PLATE_T_MAX = PA.PLATE_T_MAX
PLATE_MIN_SIDE = PA.PLATE_MIN_SIDE
PLATE_AREA_MIN = PA.PLATE_AREA_MIN
PLATE_FILL_MIN = PA.PLATE_FILL_MIN

# 板を**貫通する**宣言。ここは必ず抜く（軸・レール・通し穴）
THRU_HOW = ("THRU", "ROTATE", "SLIDE", "PRESS", "SHAFT")
# 荷重を渡す締結。⚠ `plate_audit.STRUCT` に **ROTATE / SLIDE を足す**。
#   軸受と摺動面は「留め方」としては動くが、**板から見れば支持点**。
#   実際 pitch_side は ROTATE のピボット 1 本で吊られていて、これを
#   支持に数えないと「支持ゼロの板」になる。
# ⚠ **SIT と CLAMP も荷重を渡す。** 「載せてある」「貼ってある」だけの
#   相手でも、重さは受け板にそのまま来る。ここに無かったので、
#   バケツ座（バケツ 10L）と上押さえ板（両面テープ貼りのスポンジ）が
#   「この板は何も支えていない」として最適化から静かに外れていた。
#   留め方が何であれ、上に乗っているなら荷重である。
STRUCT = PA.STRUCT + ("ROTATE", "SLIDE", "SIT", "CLAMP")
NO_LOAD = PA.NO_LOAD

# 固定領域がこれより小さくまとまっていると、面内の回転が止まらない。
# ⚠ 「固定の宣言が 2 か所あるか」で見てはいけない。1 か所でも**面で**
#   留まっていれば節点が複数拘束されるので回転は止まる。実際 yaw_arm は
#   旋回リングに 1 宣言で留まる 640mm のアームで、2 か所要求で落ちていた。
FIX_SPAN_MIN = 20.0   # mm 固定領域の対角長

# 質量がこれ未満の相手は「荷重」ではなく「座面」として扱う。
# ⚠ 小さいブラケットまで荷重点にすると、板中に力の湧き出しが散らばって
#   密度場が斑になる。力にならないものは、消させない印だけ付ければよい。
LOAD_MASS_MIN = 0.010   # kg

# ⚠ 座面は**ねじの頭の直径だけでは足りない**。荷重は板の中で 45° に広がるので、
#   頭の縁から板厚ぶん外まで効く。plate_audit の SCREW_SPREAD と同じ理由。
SEAT_GROW = PA.SCREW_SPREAD

# 面内成分がこの割合を下回る板は、面内 SIMP では解けない（面外曲げが本体）
INPLANE_MIN = 0.20

# 既定の目標体積率。⚠ 低くしすぎると解が細い糸になり、フィルタでも救えない。
# 0.30 は「元の板の 7 割を捨てる」で、使用率 24% の板でもまだ保守側。
FRAC_DEFAULT = 0.30
DX_DEFAULT = 2.0
# 座面（消させない領域）に対して、部材を繋ぐために上乗せする体積率
FRAC_MARGIN = 0.12
# 座面をこの幅まで広げる。⚠ **リブ幅そのものでは足りない。** 輪郭は最後に
# 半径 2mm で丸められるので、幅 6mm の帯は角が落ちて 4.2mm まで細る。
# 実際 pitch_side / press_post / yaw_arm はこれで「最小部材幅 4.2mm」と
# 判定され、rmin をいくら上げても（座面は密度 1 固定なので）動かなかった。
SEAT_W = RIB_MIN * 1.5


# ---------------------------------------------------------------------------
# 板ごとの境界条件を宣言から組む
# ---------------------------------------------------------------------------
def _plan(box, k: int):
    """厚み軸 k を除いた 2 軸の (lo, hi) と、その軸番号。"""
    ax = [i for i in range(3) if i != k]
    return [(box[2 * i], box[2 * i + 1]) for i in ax], ax


def _rect_local(box, k: int, ax, ctr) -> tuple[float, float, float, float] | None:
    """相手の bbox を板ローカルの矩形 (x0,y0,x1,y1) にする。範囲外なら None。"""
    r = []
    for n, i in enumerate(ax):
        lo, hi = box[2 * i] - ctr[n], box[2 * i + 1] - ctr[n]
        r.append((lo, hi))
    return (r[0][0], r[1][0], r[0][1], r[1][1])


def _blunt(ring, r: float):
    """多角形の**鋭角の頂点**を、直径 2r の円が通れるところまで落とす（開き）。

    ⚠ 「細い部材」と「形状の角」は幅の測り方では区別できない。角のほうは
      最適化で作った部材ではないので、設計領域を渡す前にここで丸めておく。
      丸めないと `solve_auto` が角の 1.4mm を部材の細りと読んで、
      rmin と frac を上限まで上げ続ける（3 回とも空振りする）。
    """
    from shapely.geometry import Polygon
    p = Polygon(ring).buffer(-r, quad_segs=16).buffer(r, quad_segs=16)
    if p.is_empty:
        return ring
    if p.geom_type == "MultiPolygon":
        p = max(p.geoms, key=lambda g: g.area)
    return [(round(float(x), 2), round(float(y), 2))
            for x, y in p.exterior.coords]


def _screw_seats(plate, tgt, children, fixings, info, k, ax, ctr, w, h, t):
    """`plate` と `tgt` をつないでいる**ねじの座面**を板ローカルの矩形で返す。

    ⚠ 相手の bbox を座面にしてよいのは「相手が板の上に載っている」ときだけ。
      斜めに寝た押出材は bbox が実体よりずっと太く、斜材の bbox は
      110×140 のガセットの 60% を覆う。荷重が板に入ってくるのは実際には
      **ねじの座面**なので、実体のねじがあるならそこへ載せる。
      実体が無ければ空を返す（呼ぶ側は従来どおり相手の bbox に戻る）。
    """
    out = []
    for own, _how in children.get(plate, ()):
        if not own.startswith("scr") or own not in info:
            continue
        if tgt not in {a for a, _h, _q in fixings.get(own, ())}:
            continue
        r = _clip(_rect_local(info[own]["box"], k, ax, ctr), w, h,
                  edge=max(SEAT_GROW, 2.0 * t))
        if r is None:
            continue
        out.append((r[0] - SEAT_GROW, r[1] - SEAT_GROW,
                    r[2] + SEAT_GROW, r[3] + SEAT_GROW))
    return out


def _flow(fixings, adj, mass, roots):
    """荷重を葉から根へ流して、**辺ごとの負担 [kg]** を出す。

    返り値は `carry[(子, 親)] = kg`。板 `nm` の荷重点 `tgt` に載せる力は
    `carry[(tgt, nm)] * g * 動的倍率`。

    ⚠ **「その板を抜いたら何が落ちるか」で測ってはいけない。** 構造は
      たいてい冗長で、yaw_side は左右 2 枚で砲塔を支えている。片方を抜いても
      もう片方が支えるので「落ちるもの＝無し」になり、**荷重ゼロの板**が
      できあがる（実際そうなった）。
    ⚠ **木で足し上げるのも駄目。** `plate_audit.subtree_mass` は根から BFS で
      張った木なので、複数経路で支持される部品は**先に見つけた親にだけ**
      計上される。同じ形の yaw_arm_r / yaw_arm_f が 1N と 44N になった。

    ここでは根からの距離が遠い順に、各部品の荷重（自分の質量＋子から
    受け取った分）を**親の数で等分**して親へ渡す。冗長支持は分担になり、
    経路が何本あっても合計は保存される。
    """
    dist: dict[str, int] = {r: 0 for r in roots}
    frontier = list(roots)
    while frontier:
        nxt = []
        for cur in frontier:
            for n in adj.get(cur, ()):
                if n not in dist:
                    dist[n] = dist[cur] + 1
                    nxt.append(n)
        frontier = nxt
    far = max(dist.values(), default=0) + 1
    order = sorted(mass, key=lambda x: -dist.get(x, far))

    flow = dict(mass)
    carry: dict[tuple[str, str], float] = {}
    for x in order:
        ps = [t for t, h, _q in fixings.get(x, ())
              if h not in NO_LOAD and t in mass]
        # ⚠ 親のうち**自分より根に近いものだけ**へ流す。距離を見ないと、
        #   同じ深さの部品どうしで荷重を回し合って合計が合わなくなる。
        up = [p for p in ps if dist.get(p, far) < dist.get(x, far)]
        up = up or ps
        if not up:
            continue
        share = flow[x] / len(up)
        for p in up:
            flow[p] = flow.get(p, 0.0) + share
            carry[(x, p)] = carry.get((x, p), 0.0) + share
    return carry


def _bridge_seats(rects, gap: float):
    """`gap` 未満しか離れていない座面の組に、あいだを埋める矩形を足す。

    ⚠ 座面は密度 1 で固定されるので、座面と座面に挟まれた首には
      **フィルタが効かない**（rmin を上げても細いまま）。ねじ 2 本の
      あいだの肉は最適化が作った細いリブではなく必ずある材料なので、
      「そこは繋がっている」と入口で教える。
    ⚠ **重なりのある軸でだけ橋を架ける。** 斜めにずれた座面どうしを
      繋ぐと、板の中を斜めに走る帯が湧いて元より重くなる。
      軸に沿って向かい合っている（もう一方の軸で重なっている）組だけ。
    """
    out = list(rects)
    n = len(rects)
    for i in range(n):
        ax0, ay0, ax1, ay1 = rects[i]
        for j in range(i + 1, n):
            bx0, by0, bx1, by1 = rects[j]
            # X で向かい合う（Y が重なる）
            oy0, oy1 = max(ay0, by0), min(ay1, by1)
            if oy1 - oy0 > 0:
                d = max(bx0 - ax1, ax0 - bx1)
                if 0 < d < gap:
                    x0, x1 = (ax1, bx0) if bx0 > ax1 else (bx1, ax0)
                    out.append((x0, oy0, x1, oy1))
            # Y で向かい合う（X が重なる）
            ox0, ox1 = max(ax0, bx0), min(ax1, bx1)
            if ox1 - ox0 > 0:
                d = max(by0 - ay1, ay0 - by1)
                if 0 < d < gap:
                    y0, y1 = (ay1, by0) if by0 > ay1 else (by1, ay0)
                    out.append((ox0, y0, ox1, y1))
    return out


def _widen(rect, minw: float, w: float, h: float):
    """細すぎる座面を最小リブ幅まで膨らませる（板の外へは出さない）。

    ⚠ **座面が細いと、最小部材幅はそこで頭打ちになる。** 座面は密度 1 固定
      なので、フィルタ半径をいくら上げても形が変わらない。実際 gus_brace は
      rmin を 6 → 16.5 まで上げても最小幅 4.0mm のままだった（ねじ穴の縁の
      残肉が 4mm だから）。フィルタではなく座面そのものを直すのが正しい。
    """
    x0, y0, x1, y1 = rect
    if x1 - x0 < minw:
        c = (x0 + x1) / 2
        x0, x1 = max(-w / 2, c - minw / 2), min(w / 2, c + minw / 2)
    if y1 - y0 < minw:
        c = (y0 + y1) / 2
        y0, y1 = max(-h / 2, c - minw / 2), min(h / 2, c + minw / 2)
    return (x0, y0, x1, y1)


def _clip(rect, w: float, h: float, edge: float = 0.0):
    """板の外接矩形で切る。完全に外なら None。

    ⚠ **交差が線になる場合を捨ててはいけない。** 板の**端面**で相手に
      載っている締結（柱の下端がブラケットの上面に立つ、など）は、平面図に
      投影すると幅ゼロの線になる。これを None にすると「留まっていない板」に
      なる。実際 press_post は car_brk の上面に立っているだけで、
      「固定の宣言が無い」と判定されて対象から落ちていた。
      線になった軸は、板の**内側へ** `edge` だけ帯を作る（そこが座面）。
    """
    x0, x1 = max(rect[0], -w / 2), min(rect[2], w / 2)
    y0, y1 = max(rect[1], -h / 2), min(rect[3], h / 2)
    if x1 < x0 - 1.0 or y1 < y0 - 1.0:
        return None                       # そもそも重なっていない
    if x1 - x0 < 1e-6:
        if edge <= 0.0:
            return None
        x0, x1 = (x0, x0 + edge) if x0 <= -w / 2 + 1e-6 else (x1 - edge, x1)
    if y1 - y0 < 1e-6:
        if edge <= 0.0:
            return None
        y0, y1 = (y0, y0 + edge) if y0 <= -h / 2 + 1e-6 else (y1 - edge, y1)
    return (x0, y0, x1, y1)


def collect(frac: float = FRAC_DEFAULT, dx: float = DX_DEFAULT):
    """(対象の Region 一覧, 外した板と理由の一覧) を返す。

    組立は作らない。`scripts/topo_cache.py` が落とした JSON を読む。
    """
    cache = topo_cache.load()
    info = {nm: {"box": d["box"], "vol": d["vol"], "mat": d["mat"],
                 "rho": d["rho"], "mass": d["mass"]}
            for nm, d in cache["parts"].items()}
    fixings = cache["fixings"]
    # bbox が実体を表していない相手（斜めに寝た押出材）
    sloped = set(cache.get("slot_axis", {}))
    # ねじの呼び径。板を貫くねじの逃げ穴を bbox ではなくバカ穴の径で開ける
    screw_dia = {nm: d[1] for nm, d in cache.get("fastener", {}).items()}
    # 「自分に留まっている相手」を引けるように逆引きを作る
    children: dict[str, list[tuple[str, str]]] = {}
    for owner, lst in fixings.items():
        for tgt, how, _q in lst:
            children.setdefault(tgt, []).append((owner, how))
    # 荷重を渡す締結だけの無向グラフ。「この板を抜いたら何が落ちるか」に使う
    adj: dict[str, set[str]] = {nm: set() for nm in info}
    for owner, lst in fixings.items():
        for tgt, how, _q in lst:
            if how in NO_LOAD or owner not in info or tgt not in info:
                continue
            adj[owner].add(tgt)
            adj[tgt].add(owner)
    mass = {nm: d["mass"] for nm, d in info.items()}
    roots = [r for r in cache["roots"] if r in info]
    carry = _flow(fixings, adj, mass, roots)

    regions: list[Region] = []
    skipped: list[tuple[str, str]] = []
    outside: list[tuple[str, str]] = []

    for nm, d in sorted(info.items()):
        if nm in PA.SKIP or nm.startswith(tuple(PA.SKIP_PREFIX)):
            outside.append((nm, PA.SKIP.get(nm) or next(
                (v for k, v in PA.SKIP_PREFIX.items() if nm.startswith(k)),
                "板の検査から外してある部品")))
            continue
        if nm in SKIP_PLATE:
            skipped.append((nm, SKIP_PLATE[nm]))
            continue
        b = d["box"]
        ext = [b[1] - b[0], b[3] - b[2], b[5] - b[4]]
        k = min(range(3), key=lambda i: ext[i])
        t = ext[k]
        plan, ax = _plan(b, k)
        w = plan[0][1] - plan[0][0]
        h = plan[1][1] - plan[1][0]
        area = w * h
        # ⚠ **ここで黙って落とすと「まだやっていない板」が見えない。**
        #   板でないもの（押出材・軸・小物）も理由つきで残す。数えられれば
        #   「対象 9 枚」が全体の何割なのかが言える。
        if t > PLATE_T_MAX:
            outside.append((nm, f"厚み {t:.0f}mm。板ではない（押出材・軸・ブロック）"))
            continue
        if min(w, h) < PLATE_MIN_SIDE:
            outside.append((nm, f"短辺 {min(w, h):.0f}mm。棒・小片"))
            continue
        if area < PLATE_AREA_MIN:
            outside.append((nm, f"投影 {area / 100:.0f}cm²。"
                                "削れる量が切り抜きの手間に見合わない"))
            continue
        # ⚠ t2 未満は**曲げ加工の板金**。切り抜き形状を最適化しても、
        #   剛性を出しているのは曲げのフランジであって面内の材料ではない。
        #   実際 ramp_side（t1.0・実肉率 28%）が対象に入ってきた。
        if t < 2.0:
            skipped.append((nm, f"t{t:.1f}。曲げ加工の板金で、面内では効いていない"))
            continue
        fill = d["vol"] / (area * t)
        if fill < PLATE_FILL_MIN:
            skipped.append((nm, f"実肉率 {fill:.0%}。平板ではない（L 字・曲げ板）"))
            continue
        # ⚠ **3Dプリント（PETG）も対象に入れる。** 切削と違って外形は
        #   最初から自由なので、「長方形に印刷して穴を開ける」理由が無い。
        #   最小部材幅（RIB_MIN=6mm）は壁 3 本ぶんあるので、この寸法なら
        #   積層でも成立する。中空板（TEKCELL / プラダン）は別で、切ると
        #   芯が縁に出るので外形は変えない。
        if d["mat"] not in ("A5052", "SUS304", "A2017", "PETG"):
            skipped.append((nm, f"材質 {d['mat']}。切削アルミでも 3Dプリントでもない"))
            continue

        ctr = [(plan[0][0] + plan[0][1]) / 2, (plan[1][0] + plan[1][1]) / 2]

        fixed: list[tuple] = []
        loads: list[tuple] = []
        solid: list[tuple] = []
        void: list[tuple] = []
        void_rect: list[tuple] = []
        f_in = 0.0        # 面内に乗った力の合計
        f_all = 0.0       # 全方向の力の合計

        # (相手, 方法, 上流か) — 上流 True = この板が留まっている先 = 固定側
        partners = [(t, h, True) for t, h, _q in fixings.get(nm, ())]
        partners += [(o, h, False) for o, h in children.get(nm, ())]

        for tgt, how, upstream in partners:
            if how in NO_LOAD or tgt not in info:
                continue
            ob = info[tgt]["box"]
            # 厚み方向で届いていなければ、宣言だけで実体は触れていない
            if ob[2 * k] > b[2 * k + 1] + 1.0 or ob[2 * k + 1] < b[2 * k] - 1.0:
                continue
            # 端面で載っているだけの相手は、板の内側へ座面ぶんの帯を作る
            r = _clip(_rect_local(ob, k, ax, ctr), w, h,
                      edge=max(SEAT_GROW, 2.0 * t))
            if r is None:
                continue

            # --- 板を貫通する相手: 必ず抜く ------------------------------
            # ⚠ ねじより先に見ること。通しボルトは `scr` で始まるうえに
            #   板を貫くので、座面だけ置いて穴を開けないと軸が材料の中を通る。
            if how in THRU_HOW and (ob[2 * k] <= b[2 * k] + 0.5
                                    and ob[2 * k + 1] >= b[2 * k + 1] - 0.5):
                cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
                rad = max(r[2] - r[0], r[3] - r[1]) / 2
                # ⚠ **板を貫くのはねじの軸であって、溝ナットではない。**
                #   ナットは相手の溝の中＝板の反対側にいる。ねじの実体の
                #   bbox（M5 + 溝ナットで φ11）をそのまま逃げ穴にすると、
                #   穴が呼び径の 2 倍になり、縁の残肉が 4.5mm しかない板が
                #   「最小部材幅 4.0mm。切れない」で落ちる。実際 gus_brace は
                #   これで rmin も frac も上限まで空振りしていた（穴を
                #   小さくしても直らないので、原因が穴だと気づけない）。
                #   ねじならバカ穴（呼び径 + 0.5）で抜く。
                if screw_dia.get(tgt):
                    rad = min(rad, screw_dia[tgt] / 2.0 + 0.25)
                void.append((cx, cy, rad))
                # 穴の**まわりの帯**。矩形しか書けないので上下左右 4 本に割る。
                # ⚠ 外接矩形 1 枚にすると、穴の中（密度 0 の要素に囲まれた
                #   節点）にも力や拘束が載る。そこは剛性がほとんど無いので
                #   変位が発散する。
                g = max(SEAT_W, 0.25 * rad)
                ring = [(cx - rad - g, cy + rad, cx + rad + g, cy + rad + g),
                        (cx - rad - g, cy - rad - g, cx + rad + g, cy - rad),
                        (cx - rad - g, cy - rad, cx - rad, cy + rad),
                        (cx + rad, cy - rad, cx + rad + g, cy + rad)]
                # ⚠ 逃げ穴の**まわりに座面の輪を置いてはいけない**。座面は
                #   矩形でしか書けないので、輪のつもりで置いた矩形が穴を
                #   丸ごと覆い、`solid`（密度 1 固定）が `void`（密度 0 固定）と
                #   ぶつかる。実際 pitch_side の逃げ穴 φ22 に材料が
                #   17〜25mm² 残った。穴の縁は SIMP が勝手に残す（縁に応力が
                #   集まるので）。細くなったら `shape.check` の最小部材幅で落ちる。
                if upstream and how in STRUCT:
                    # 軸受は**この板の支持そのもの**。ROTATE のピボットで
                    # 吊られた側板は、ここを拘束しないと支持ゼロになる。
                    fixed.extend(ring)
                    continue
                # ⚠ **貫通する相手を荷重から外してはいけない。** 軸受は穴を
                #   通るので THRU 側で処理して `continue` していたが、
                #   射出ローラーは軸受を介して側板にぶら下がっている。
                #   結果 pitch_side の荷重が全部 1N 以下になり（合計 12N）、
                #   「ほとんど力の掛からない板」として解かれていた。
                #   軸受は穴の**縁**で力を伝える。4 本の帯に等分して載せる。
                m = carry.get((tgt, nm), 0.0)
                if m < LOAD_MASS_MIN:
                    solid.extend(ring)
                    continue
                fw = [0.0, 0.0, -m * G * DYN]
                lat = m * G * ACC_LAT
                for i in ax:
                    if i != 2:
                        fw[i] += lat
                        break
                fx, fy = fw[ax[0]] / 4.0, fw[ax[1]] / 4.0
                f_in += 4 * math.hypot(fx, fy)
                f_all += 4 * math.sqrt(sum((v / 4) ** 2 for v in fw))
                loads.extend((rr, fx, fy) for rr in ring)
                continue

            # --- ねじ: 座面として必ず残す -------------------------------
            # ⚠ ねじを荷重点に数えてはいけない。ねじ自身の質量が板を
            #   引っ張ることになって、締結の周りだけ材料が集まる形が出る。
            if tgt.startswith("scr"):
                solid.append((r[0] - SEAT_GROW, r[1] - SEAT_GROW,
                              r[2] + SEAT_GROW, r[3] + SEAT_GROW))
                continue

            if how not in STRUCT:
                continue

            if upstream:
                fixed.append(r)          # この板が留まっている先＝動かない
                continue

            # この板にぶら下がっている相手。⚠ その相手 1 個の質量ではなく、
            #   **その相手を通ってこの板に降りてくる荷重**。ブラケット 1 個は
            #   30g でも、その先に 5kg の砲塔がぶら下がっている。
            # ⚠ **相手の bbox を座面にしてよいのは、相手が板の上に載って
            #   いるときだけ。** 斜めに寝た押出材は bbox が実体よりずっと
            #   大きく、斜材の bbox は 110×140 のガセットの 60% を覆う。
            #   そのまま座面（密度 1 固定）にすると、`shape.to_outline` の
            #   足し戻しで板が外接矩形いっぱいに戻り、最適化なのに元の
            #   直角三角形より 79% 重い形が出る（実際に出た）。
            #   荷重が板に入るのは**その相手を留めているねじの座面**なので、
            #   実体のねじがあるならそちらに載せる。
            seats = _screw_seats(nm, tgt, children, fixings, info,
                                 k, ax, ctr, w, h, t) \
                if tgt in sloped else []
            m = carry.get((tgt, nm), info[tgt]["mass"])
            if seats:
                if m < LOAD_MASS_MIN:
                    solid.extend(seats)
                    continue
                fw = [0.0, 0.0, -m * G * DYN]
                lat = m * G * ACC_LAT
                for i in ax:
                    if i != 2:
                        fw[i] += lat
                        break
                fx, fy = fw[ax[0]] / len(seats), fw[ax[1]] / len(seats)
                f_in += len(seats) * math.hypot(fx, fy)
                f_all += len(seats) * math.sqrt(
                    sum((v / len(seats)) ** 2 for v in fw))
                loads.extend((rr, fx, fy) for rr in seats)
                continue
            if m < LOAD_MASS_MIN:
                solid.append(r)          # 力にならない。座面としてだけ残す
                continue
            # 世界座標での力: 重力（-Z, 動的倍率つき）＋ 横加速度
            fw = [0.0, 0.0, -m * G * DYN]
            lat = m * G * ACC_LAT
            # ⚠ 横加速度の向きは**面内に乗る方**へ全部載せる（保守側）。
            #   前後・左右のどちらで曲がるかは走行次第なので、板ごとに
            #   厳しいほうを取る。ここを「常に世界 X」にすると、
            #   厚み軸が X の板だけ横力ゼロになって、実機で先に裂ける。
            for i in ax:
                if i != 2:
                    fw[i] += lat
                    break
            fx, fy = fw[ax[0]], fw[ax[1]]
            f_in += math.hypot(fx, fy)
            f_all += math.sqrt(sum(v * v for v in fw))
            loads.append((r, fx, fy))

        # --- 宣言に無いのに板の場所を占める相手 -------------------------
        # ⚠ **締結の宣言だけ見ていると穴が足りない。** 仰角モーターは
        #   pitch_side に留まっていない（ウォームブラケットに留まる）ので
        #   partners に出てこないが、板と同じ場所を占める。元のコードは
        #   ここに手で逃げ穴を開けていた。外形を切り直したらそれが消えて、
        #   「未宣言接触 pitch_side_R ↔ pitch_motor 0.00mm」が出た。
        #   相手が板の厚みを跨いでいるなら、そこは必ず抜く。
        near = {t for t, _h, _q in fixings.get(nm, ())}
        near |= {o for o, _h in children.get(nm, ())}
        for other, od in info.items():
            if other == nm or other in near:
                continue
            if other.startswith(("scr", "cab_", "wire_", "rag_")):
                continue
            ob = od["box"]
            # 厚み方向で**重なっている**こと（面で接するだけなら穴は要らない）
            if (ob[2 * k] >= b[2 * k + 1] - 0.5
                    or ob[2 * k + 1] <= b[2 * k] + 0.5):
                continue
            r = _clip(_rect_local(ob, k, ax, ctr), w, h)
            if r is None or (r[2] - r[0]) * (r[3] - r[1]) < 4.0:
                continue
            void_rect.append(r)

        # 固定領域の広がり。1 点留めだと面内の回転が止まらない
        span = 0.0
        if fixed:
            fx0 = min(r[0] for r in fixed); fx1 = max(r[2] for r in fixed)
            fy0 = min(r[1] for r in fixed); fy1 = max(r[3] for r in fixed)
            span = math.hypot(fx1 - fx0, fy1 - fy0)

        why = None
        if not fixed:
            why = "固定の宣言が無い（この板は何にも留まっていない）"
        elif span < FIX_SPAN_MIN:
            why = (f"固定領域の広がりが {span:.0f}mm しかない"
                   f"（{FIX_SPAN_MIN:.0f}mm 必要）。面内の回転が止まらない")
        elif not loads:
            why = "荷重を渡す相手が宣言に無い（この板は何も支えていない）"
        elif f_all > 0 and f_in / f_all < INPLANE_MIN:
            why = (f"荷重の面内成分が {f_in / f_all:.0%}。"
                   f"面外曲げが本体なので平面応力では解けない（厚み軸 "
                   f"{'XYZ'[k]}）")
        if why:
            skipped.append((nm, why))
            continue

        # ⚠ **拘束座と荷重座は必ず「消させない」領域にも入れる。**
        #   境界条件を置いただけでは、SIMP はそこを削ってよいと判断する
        #   （削っても解けてしまう。力は節点に載るので材料が要らない）。
        #   実際これを忘れて解いたら、yaw_side_R の拘束座 2 か所が
        #   合わせて 912mm² 輪郭の外へ出た＝**相手に留められない板**が出た。
        solid = [_widen(r, SEAT_W, w, h)
                 for r in solid + fixed + [r for r, _fx, _fy in loads]]
        # ⚠ **近すぎる座面のあいだは埋める（2026-08-05）。** 座面は
        #   `SEAT_W`(=9mm) 角で置くので、ねじ 2 本が 13.6mm 離れていると
        #   あいだが 4.6mm の首になる。密度 1 の座面どうしに挟まれた首は
        #   **フィルタが効かない**（実測 `yaw_side_L` は rmin を 6 → 32 まで
        #   上げても最小幅 4.60mm のまま。frac も 0.70 まで上げて動かず）。
        #   その結果 `shape.check` が「最小部材幅 < 6mm」で落とし、板 3 枚が
        #   最適化を諦めて素の板になっていた。
        #   ⚠ **これは判定を緩める話ではない。** ねじ 2 本のあいだの肉は
        #     最適化が作った細いリブではなく、**物理的に必ずある材料**。
        #     入口（境界条件）の側で「ここは繋がっている」と教えるのが正しい。
        #     `RIB_MIN` 未満しか離れていない座面の組に、橋を 1 枚足す。
        solid = _bridge_seats(solid, RIB_MIN)
        # ⚠ 投影外形は**穴を埋めたもの**なので、機能上どうしても開いて
        #   いなければならない穴は教えないと埋まった形が出る。埋めた形を
        #   後からくり抜くと、そこに材料がある前提で解いた結果とずれる。
        void = void + VOID_EXTRA.get(nm, [])

        # ⚠ フィルタ半径は「最小部材の**幅**」で置く。半幅（RIB_MIN/2）に
        #   すると、円錐フィルタが作る帯は半径と同程度の幅にしかならず、
        #   6mm 狙いで 5.3mm の部材が出た（`shape.check` で落ちた）。
        reg = Region(
            name=nm, w=w, h=h, t=t,
            fixed=fixed, loads=loads, solid=solid, void=void,
            void_rect=void_rect,
            frac=frac, rmin=RIB_MIN, dx=dx,
            mat=d["mat"], rho_mat=d["rho"] / 1000.0, mass0=d["mass"],
            note=f"厚み軸 {'XYZ'[k]} / 面内 {'XYZ'[ax[0]]}{'XYZ'[ax[1]]} / "
                 f"実肉率 {fill:.0%}")

        # ⚠ **目標体積率は座面より下には置けない。** 座面（消させない領域）
        #   だけで 64% を占める板に frac=0.30 を渡すと、SIMP は「制約が
        #   矛盾している」で例外になる。実際 car_side / gus_brace / yaw_arm_f /
        #   yaw_motor_deck の 7 枚がここで落ちた。
        #   板ごとに「座面 + 繋ぐぶん」を下限として置き直す。
        # 材料を置いてよい範囲を、板の**元の外形**（穴は埋めたもの）に限る。
        # ⚠ 無ければ外接矩形の全面になり、元が三角形の板では改悪になる。
        reg.domain = cache.get("outlines", {}).get(nm)
        if reg.domain is None:
            reg.note += " / 投影外形が取れず外接矩形で解く（重くなりうる）"
        else:
            # ⚠ **設計領域そのものに RIB_MIN より細い所を残さない。** 元が
            #   直角三角形のガセットは、鋭角の頂点で幅が 0 に落ちる。そこに
            #   材料が残ると `shape.check` は「最小部材幅 1.4mm」と読むが、
            #   これは**部材の細り**ではなく形状の角で、rmin でも frac でも
            #   直らない。実際 solve_auto が 3 回とも空振りして frac だけ
            #   上限まで上がり、最後は「目標体積率が置けない」で落ちた。
            #   頂点は最適化した板の部材ではないので、領域の段階で落とす。
            reg.domain = _blunt(reg.domain, RIB_MIN / 2.0)

        keep = rect_mask(reg, reg.solid) & ~dead_mask(reg)
        floor = float(keep.mean()) + FRAC_MARGIN
        # ⚠ 体積率は**外接矩形に対する比**なので、設計領域そのものの比が
        #   上限になる。三角形のガセット（矩形の 50%）に frac 0.85 を渡すと
        #   「置く場所が無い」で例外になる。上限は領域比の 95%。
        reg.frac_max = float(domain_mask(reg).mean()) * 0.95
        if floor > reg.frac:
            reg.note += f" / 座面が {keep.mean():.0%} を占めるので frac を上げた"
        reg.frac = min(max(reg.frac, floor), reg.frac_max)
        regions.append(reg)
    return regions, skipped, outside


# 輪郭を使ってよいかを止める不良。⚠ ここに無いものは「見て直すべきだが、
# 組めなくはない」。座面の角が丸めで 1mm² 落ちるのと、板が 2 つに割れて
# いるのを同じ扱いにすると、後者を見落とす。
# ⚠ **「逃げ穴に材料が残っている」「拘束座／荷重座が輪郭の外」も足した
#   （2026-08-05）。** 軸が通らない板・相手に留められない板を JSON に書けば、
#   そのまま DXF になって実物が組めない。座面の角が丸めで 1mm² 欠ける件は
#   `shape.check` 側を直した（ねじ座面は頭の領域で問う／宣言した逃げ穴は
#   数えない）ので、ここを厳しくしても全滅はしない。
#   ⚠ 書かない＝その板は**素の板**に戻る。重くなるが必ず切れる。
FATAL = ("連結成分", "最小部材幅", "より重い", "に材料が残っている",
         "相手に留められない", "力を受ける座が無い")


def fatal(bad) -> list[str]:
    return [m for m in bad if any(k in m for k in FATAL)]


# 「前の輪郭を据え置いてよいか」を決める語。⚠ `FATAL` より広い。
# `FATAL` は**輪郭を JSON に書かない**ための語で、そこに「ねじが留められない」
# を入れると、座面の角が丸めで 1mm² 欠けただけで 1 枚も書けなくなる。
# 一方**据え置き**は「前の形で組立が成立している」が前提なので、ねじが
# 留まらない形を残す理由は無い。ここを分けずに `fatal` だけで判定したため、
# 「前より太い」で据え置かれ続け、**解き直しても直らない**状態が続いていた。
# ⚠ **「逃げ穴に材料が残っている」と「座が輪郭の外」もここに入れる
#   （2026-08-05）。** 前は「致命的」（＝ねじ座面）だけを見ていたので、
#   軸が板に当たる形・拘束座が板の外に出た形が「前より太い」を理由に
#   据え置かれ続けていた。実測 `beltbrk_idl_L` は void[1]/void[2] に
#   19.8mm² ずつ材料が残ったまま据え置かれ、`topo_check` が 51 件の
#   「軸／配線が板に当たる」を出していた。**軸が通らない板は、軽くても
#   使えない。**
UNUSABLE = FATAL + ("致命的", "に材料が残っている",
                    "相手に留められない", "力を受ける座が無い")


def unusable(bad) -> list[str]:
    return [m for m in bad if any(k in m for k in UNUSABLE)]


def bc_hash(reg) -> str:
    """境界条件の指紋。輪郭が古いかどうかを**形に関わる変更だけ**で見る。

    ⚠ ファイルの更新時刻で比べると、`src/` を 1 文字直しただけで全部の輪郭が
      「古い」になる。実際 `tr_lib.py` に sys.path を 3 行足しただけで
      9 枚とも警告が出た。板の形に関係ない変更で警告が出続けると、
      **本当に古くなったときに気づけなくなる**。
    ⚠ `frac` / `rmin` は入れない。自動リトライが解いている最中に書き換える
      ので、入れると同じ板でも回すたびに指紋が変わる。
    """
    import hashlib

    def r2(v):
        return round(float(v), 2)

    d = {
        "wht": [r2(reg.w), r2(reg.h), r2(reg.t)],
        "fixed": sorted([r2(x) for x in r] for r in reg.fixed),
        "loads": sorted([*[r2(x) for x in r], r2(fx), r2(fy)]
                        for r, fx, fy in reg.loads),
        "solid": sorted([r2(x) for x in r] for r in reg.solid),
        "void": sorted([r2(x) for x in c] for c in reg.void),
        "void_rect": sorted([r2(x) for x in r] for r in reg.void_rect),
        "domain": [[r2(x), r2(y)] for x, y in (reg.domain or ())],
    }
    return hashlib.sha256(
        json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]


def solve_auto(reg, iters: int, tries: int = 10):
    """成立するまで frac / rmin を上げて解き直す。

    返り値は (結果, 輪郭, 残った不良, 何をしたかの記録)。

    ⚠ ここは**機械が回すループ**（分断していたら太らせる、細すぎたら
      フィルタを広げる）。人（Claude）が図を見て回すループは別で、
      境界条件そのものや目標体積率の狙いを直す。混ぜないこと。
    """
    from topo import shape, simp

    log: list[str] = []
    res = out = None
    bad: list[str] = []
    prev_w = -1.0
    # ⚠ 「元の板がすでに細いなら下限を下げる」を一度入れて**外した**。
    #   ガセットの元の外形は最小幅 1.4mm と出る。細いリブがあるのではなく
    #   **三角形の頂点**を拾っているだけで、これを下限にすると何でも通る。
    #   形状の角と部材の細りは、幅の測り方では区別できない。下限は
    #   `RIB_MIN` のまま置き、通らない板は「この方式では作れない」と出す。
    # ⚠ 下限は「**元の板が既に持っている細り**」（`shape.plate_min_width`）。
    #   元が 4.56mm の板に 6mm を課しても永久に通らない。元が 6mm 以上の
    #   板ではこれまでどおり `RIB_MIN` になる。
    rib = shape.plate_min_width(reg)
    for i in range(tries):
        res = simp.solve(reg, iters=iters)
        out = shape.to_outline(res)
        bad = shape.check(out, reg, rib_min=rib)
        split = [m for m in bad if "連結成分" in m]
        thin = [m for m in bad if "最小部材幅" in m]
        if not split and not thin:
            break
        if i == tries - 1:
            break
        if split:
            # ばらばら＝材料が足りない。太らせる。⚠ 設計領域の比を超えては
            #   太らせられない（置く場所が無い）。頭打ちなら打ち切る。
            top = getattr(reg, "frac_max", 0.90)
            if reg.frac >= top - 1e-3:
                log.append(f"frac {reg.frac:.2f} が設計領域の上限。"
                           f"これ以上太らせられない。打ち切り")
                break
            reg.frac = min(top, reg.frac + 0.12)
            # ⚠ 分断は「材料が足りない」だけでなく「部材が細くて切れた」でも
            #   起きる。frac だけ上げると、離れた島がそれぞれ太るだけで
            #   繋がらないことがある。フィルタも少し広げて橋を架けさせる。
            reg.rmin *= 1.15
            log.append(f"分断 {out.n_parts} 個 → frac {reg.frac:.2f} / "
                       f"rmin {reg.rmin:.1f} へ")
        elif thin:
            # 細すぎ＝フィルタが狭い。⚠ frac を上げても細い部材は太らない
            #   （全体が太るだけで、最小幅を決めるのはフィルタ半径）。
            # ⚠ **効かないリトライは打ち切る。** 座面そのものが細い板では
            #   rmin をいくら上げても最小幅が動かない（密度 1 固定なので
            #   フィルタが掛からない）。実際 gus_brace は 6 → 16.5 まで
            #   上げて 3 回とも 4.0mm のままだった。3 回無駄に解いている。
            if out.min_width <= prev_w + 0.1:
                # ⚠ **ここで打ち切ってはいけない。** rmin が効かないのは
                #   「座面が細い」ときだけではない。材料そのものが足りなくて
                #   部材が細く広がっているなら、frac を上げれば太る。実際
                #   pitch_side_R と yaw_arm_f は逃げ穴を足したとたん 4.1mm に
                #   なり、rmin では戻らないのに frac では戻った。
                top = getattr(reg, "frac_max", 0.90)
                if reg.frac >= top - 1e-3:
                    log.append(f"rmin も frac も上限。最小幅 "
                               f"{out.min_width:.1f}mm のまま。打ち切り")
                    break
                reg.frac = min(top, reg.frac + 0.10)
                prev_w = -1.0            # rmin の履歴はここで切る
                log.append(f"rmin が効かない → frac {reg.frac:.2f} へ")
                continue
            prev_w = out.min_width
            reg.rmin *= 1.4
            log.append(f"最小幅 {out.min_width:.1f}mm → rmin {reg.rmin:.1f} へ")
    # ⚠ **重くなる結果を採用してはいけない。** 設計領域は板の外接矩形なので、
    #   元が三角形の板（ガセット）では、最適化の結果が元より大きくなりうる。
    #   実際 gus_brace は 62g → 105g と増えた。増えたら「元の形のほうが良い」。
    if res is not None and reg.mass0 > 0 and res.mass() >= reg.mass0:
        bad = bad + [f"{reg.name}: 最適化しても {res.mass() * 1000:.0f}g で、"
                     f"元の {reg.mass0 * 1000:.0f}g より重い。"
                     f"設計領域（外接矩形）が元の外形より大きい板なので、"
                     f"この方式では改善しない"]
    return res, out, bad, log


# ---------------------------------------------------------------------------
# 上書き（板ごとに手で決めたいパラメータ）
# ---------------------------------------------------------------------------
# ⚠ ここに書くのは**最後の手段**。既定値で出た形を見てから、その板だけ
#   動かす。理由を必ず添えること（添えないと、次に見た人が戻せない）。
OVERRIDE: dict[str, dict] = {
    # 左右対称の板なのに L だけ 4.2mm の細りが出る（R は 7.5mm で通る）。
    # 荷重点が 20 対 15 で L のほうが多く、材料が薄く広がって細る。
    # dx を 1.5 に細かくしても 4.6mm までしか戻らなかった（格子の粗さでは
    # ない）。frac を 0.30 → 0.36 にすると 9.8mm。削減は 238g → 151g に
    # 落ちるが、**切れない形を採るよりは軽さを譲る**。
    "pitch_side_L": {"frac": 0.36},
}

# 対象から外す板と、その理由。⚠ **「効かなかった」も結果なので消さずに残す。**
#   消すと、次に見た人が同じ板でまた同じことを試す。
# 宣言からは出てこない逃げ穴 (cx, cy, r)。板の**局所座標**（中心が原点、
# 面内 2 軸の順は `--list` の「面内」の並び）。
# ⚠ 「板を貫く相手」は宣言から自動で拾える（THRU / ROTATE / SLIDE / …）。
#   ここに書くのは**相手が貫かないのに開いている穴**だけ。
VOID_EXTRA = {
    # ヨー駆動の延長軸 φ18（Z 844.5..872.7）が板（Z 862..868）を**貫く**。
    # ⚠ 宣言に無い相手なので `void_rect`（外接箱）としては拾えているが、
    #   矩形の逃げは**座面に譲る**規則がある（`core.dead_mask`）。ここは
    #   隣のねじの座面と重なるので材料が残り、軸に 7mm³ 食い込んだ。
    #   軸は本当に板を貫くので、譲らない**円の逃げ**として書く。
    # ⚠ 半径は**軸の半径ぴったりにしない**。9.5（φ19）だと φ18 の軸との
    #   すきまが 0.5mm で、`assembly_check` が「宣言のない接触」に数える。
    #   回る軸なので実機でも 0.5mm は擦れる。片側 2mm 空ける。
    "yaw_arm_f": [(0.0, 190.0, 11.0)],
}

SKIP_PLATE = {
    # ⚠ 台座リングプレートは**梁の真上だけを残した円弧**（`_pedestal_group`）。
    #   板のほぼ全面が梁に載っているので、拘束座（梁）と荷重座（V リング）が
    #   同じ場所になり、`solve_initial` が
    #   「荷重が全部 fixed 節点に載っている（自由 dof に力が無い）」で落ちる。
    #   ⚠ **落ちるのは正しい。** 拘束と荷重が重なった状態で解いても、
    #     意味のない密度場が出るだけで、その形で板を切ることになる。
    #   そもそも削る余地が無い形（布の通り道を空けるために既に円弧 2 枚まで
    #   削ってある）。最適化の対象から外す。
    #   ⚠ 2026-08-05 に `pedestal_ring` を 2 部品へ分けた（1 部品 2 ソリッドは
    #     製作データが「平板 1 枚で表せない」と落とすため）。分けたことで
    #     ここの対象に入ってしまった。分割前は 2 ソリッドだったので
    #     投影外形が作れず、静かに対象外になっていた。
    "pedestal_ring0": "梁の真上だけの円弧。拘束座と荷重座が重なる（削る余地も無い）",
    "pedestal_ring1": "梁の真上だけの円弧。拘束座と荷重座が重なる（削る余地も無い）",
    # ⚠ **斜材ガセット（直角三角形）4 枚はここから外した（DESIGN.md §42）。**
    #   前は「元が直角三角形。削れて +2〜3g」で外していたが、削れないので
    #   はなく**境界条件のほうが間違っていた**。直したのは 5 つ:
    #     ・斜材へのボルトが溝（＝軸線）から 3.0〜6.1mm 外れていた
    #       → `tr_assembly._brace_bolt_holes` で軸線に載せた
    #     ・斜材の bbox（板の 60%）を座面にしていた → `_screw_seats`
    #     ・座面の足し戻しが設計領域の外にも材料を置いていた → `shape`
    #     ・三角形の鋭角の頂点を「細い部材」と読んでいた → `_blunt`
    #     ・貫くねじの逃げを実体の bbox（φ11）で開けていた → バカ穴 φ5.5
    #   結果 4 枚で −61.4g。⚠ **「解いたら重くなった」を理由に外すときは、
    #   境界条件が正しいかを先に見ること。** 5 つとも、外した形（＝結果）
    #   ではなく入口（＝境界条件）の誤りだった。
    # ⚠ 50×131 の板に座面が 42% ある。繋ぐと元の柱より重くなる。
    "press_post_L": "元が細い柱。最適化すると逆に 15g 重くなる",
    "press_post_R": "元が細い柱。最適化すると逆に 15g 重くなる",
    # ⚠ ここに書くのは「**解いた上で**割に合わなかった」板だけにすること。
    #   「たぶん削れない」で足すと、やっていない板が理由つきで隠れる。
    # ⚠ 350×60 に締結が 5 か所。座面だけで 64% を占め、削る場所が無い。
    #   この板の余肉は形ではなく**板厚**（t5 → t2 で 129g、plate_audit）。
    # --- 以下は 2026-08-02 に**実際に解いた上で**外したもの ---------------
    # ⚠ 「座面が 64% だから形では削れない」と解かずに書いてあったので、
    #   解いてみた。結果は下のとおりで、外す判断そのものは変わらないが、
    #   根拠が見立てから実測になった。
    "car_side_L": "解いた: 連結成分 2 個のまま frac 0.95（設計領域の上限）"
                  "で打ち切り。−37g だが形が成立しない",
    "car_side_R": "解いた: −1g。座面（拘束座と同じ帯）が板を横断していて"
                  "削る余地が無い",
    "press_plate": "解いた: 426g で元の 374g より重い。設計領域（外接矩形）"
                   "が元の外形より大きい板なので、この方式では改善しない",
    # --- LiDAR のレベリング座（2026-08-06 に追加）------------------------
    # ⚠ **ここは「解いてみたが割に合わなかった」ではなく、いまの解き方では
    #   境界条件が書けない板。** 2 枚の板は 12mm 空いていて、荷重は
    #   **調整ねじ 4 本を通って**渡る（`ADJUST`）。ところが partners の
    #   ふるいは「厚み方向で届いていなければ実体は触れていない」で相手を
    #   落とすので、可動板は支持ゼロ、固定板は荷重ゼロになる。
    #   ふるいを ADJUST だけ緩めると、今度は相手の**投影全面**が固定領域に
    #   なって板が丸ごと拘束される（実際に効いているのは 4 か所の座面だけ）。
    #   正しく書くには「相手ではなくねじの座を支持にする」経路が要る。
    #   見込みは 6 枚で 180g 程度。18 枚の既存の板を壊す危険と釣り合わない
    #   ので、**質量の削減レバーとして残す**（DESIGN.md §7）。
    "lidar_lvl_base_front": "ADJUST（すきま 12mm・荷重はねじ 4 本を通る）の"
                            "境界条件が書けない。§7 の削減レバーに回した",
    "lidar_lvl_base_rear": "同上",
    "lidar_lvl_base_high": "同上",
    "lidar_lvl_top_front": "同上（可動板。支持は 4 本のねじの頭だけ）",
    "lidar_lvl_top_rear": "同上",
    "lidar_lvl_top_high": "同上",
    "lidar_high_brk": "解いた: +5g。70×80 の板に座面が 71% で余地が無い"
                      "（2026-08-06 に 70×69 t8 へ。座面はさらに増えた）",
    "lidar_low_brk_front": "解いた: 座面 76% > 空き 64% で目標体積率が置けない"
                           "（frac の下限が設計領域を超える。2026-08-06 に"
                           "t3 → t8 / 70×69 へ。座面の比率は変わらない）",
    "lidar_low_brk_rear": "解いた: 同上（front と同じ板）",
    "disp_frame": "解いた: +3g。表示器の枠は座面が 92% で、削れるのは"
                  "四隅だけ（PETG なので元から軽い）",
    # ⚠ 2026-08-07 に板を作り直した（角板 340×340 t2 + φ44 の丸穴 12 個 →
    #   受け円環 + ビームに載る帯 2 本 t3）。設計領域の外接矩形（340 角）が
    #   新しい外形（270×334）より大きいので、`press_plate` と同じ理由で
    #   この方式では改善しない。**形はもう荷重の通り道そのもの**
    #   （円環でバケツ底を受け、桁で帯へ渡し、帯がビームに載る）。
    "bucket_seat": "解いた: 384g で元の 310g より重い。2026-08-07 に円環＋帯へ"
                   "作り直して 281g（t2→t3 でも軽い）。設計領域が外形より"
                   "大きいので最適化しても戻るだけ",
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="境界条件だけ出す（解かない）")
    ap.add_argument("--only", default=None, help="この板だけ")
    ap.add_argument("--frac", type=float, default=FRAC_DEFAULT)
    ap.add_argument("--dx", type=float, default=DX_DEFAULT)
    ap.add_argument("--iters", type=int, default=80)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    regions, skipped, outside = collect(frac=args.frac, dx=args.dx)
    regions.sort(key=lambda r: -r.mass0)
    if args.only:
        regions = [r for r in regions if r.name == args.only]
        if not regions:
            print(f"板 {args.only} は対象に入っていない。--list で理由を見ること")
            return 2

    if args.list:
        print(f"{'板':<18} {'寸法':>13} {'t':>4} {'固定':>4} {'荷重':>4} "
              f"{'座面':>4} {'逃げ':>4} {'合力[N]':>8}  備考")
        print("-" * 100)
        for r in regions:
            f = sum(math.hypot(fx, fy) for _r, fx, fy in r.loads)
            print(f"{r.name:<18} {r.w:6.0f}×{r.h:6.0f} {r.t:4.1f} "
                  f"{len(r.fixed):4d} {len(r.loads):4d} {len(r.solid):4d} "
                  f"{len(r.void):4d} {f:8.0f}  {r.note}")
        print(f"\n対象 {len(regions)} 枚 / 外した板 {len(skipped)} 枚")
        for nm, why in skipped:
            print(f"  — {nm:<18} {why}")
        # ⚠ **そもそも板と見なされなかったもの**も数える。ここを出さないと
        #   「対象 9 枚」が全体の何割なのか誰にも言えない（前は黙って
        #   落ちていて、押出材も軸もねじも同じ「対象外」に混ざっていた）。
        by_why: dict[str, int] = {}
        for _nm, why in outside:
            key = why.split("。")[-1] or why
            by_why[key] = by_why.get(key, 0) + 1
        print(f"\n板ではない・検査対象外 {len(outside)} 個")
        for key, n in sorted(by_why.items(), key=lambda kv: -kv[1]):
            print(f"  · {n:4d} 個  {key}")
        return 0

    os.makedirs(OUT, exist_ok=True)
    # ⚠ **対象から外した板の輪郭は消す。** 残しておくと `topo_plate` が
    #   古い形を読んで STEP に載せるし、`topo_check` は「境界条件が作れない
    #   板になった」と言い続ける（実際 car_side の輪郭が残って落ちていた）。
    for nm in SKIP_PLATE:
        p = os.path.join(OUT, f"{nm}.json")
        if os.path.exists(p):
            os.remove(p)
            print(f"対象外にした {nm} の輪郭を消した")
    # ⚠ **対象から**落ちた**板の輪郭も消す（2026-08-05）。** 板厚を変えると
    #   その板が `collect()` の条件から外れることがある（`car_brk` を t3 →
    #   t5 にしたら 20 → 18 枚になった）。輪郭 JSON だけが残ると
    #   `topo_check` が「境界条件が作れない板になった」と言い続ける。
    #   ⚠ **`topo_plate()` で呼ばれている板は消さない。** 消すと組立が
    #     `FileNotFoundError` で落ちる（実際 3 枚消して全工程が止まった）。
    if not args.only:
        import re as _re
        used = set()
        for _p in glob.glob(os.path.join(ROOT, "src", "*.py")):
            with open(_p, encoding="utf-8") as _fp:
                used |= set(_re.findall(r'topo_plate\(\s*["\'](\w+)["\']',
                                        _fp.read()))
        alive = {r.name for r in regions} | set(SKIP_PLATE) | used
        for p in glob.glob(os.path.join(OUT, "*.json")):
            nm = os.path.splitext(os.path.basename(p))[0]
            if nm.startswith("_") or nm in alive:
                continue
            os.remove(p)
            print(f"対象から外れた {nm} の輪郭を消した（板厚などが変わった）")

    from topo import draw, shape, simp

    items = []
    for r in regions:
        for key, val in OVERRIDE.get(r.name, {}).items():
            setattr(r, key, val)
        base = simp.solve_initial(r)
        draw.heatmap_png(base, os.path.join(OUT, f"{r.name}_load.png"),
                         title=f"{r.name} 荷重")
        res, out, bad, log = solve_auto(r, args.iters)
        draw.shape_png(res, out, os.path.join(OUT, f"{r.name}_shape.png"),
                       title=f"{r.name} 最適化後")
        # ⚠ **輪郭を書く前に、その板の基準を凍結する。** 輪郭を書いた瞬間から
        #   組立は `topo_plate` でその形を使い、次に `topo_cache.py` を回すと
        #   「最適化後の板」を測ってしまう。設計領域も原点も元の質量も、
        #   そこで失われる（実際いちど失って測り直しになった）。
        topo_cache.freeze(topo_cache.load(), [r.name])
        # ⚠ 不良が残った輪郭を JSON に書かない。書くと `topo_plate` が
        #   それを読んで STEP に載せてしまう（ばらばらの板が図に出る）。
        #   ただし「座面が輪郭から 1〜2mm² はみ出す」は**輪郭の丸め**が
        #   板の縁の角を落としているだけで、組めなくなる話ではない
        #   （丸め半径 2mm の角 1 個が 0.86mm²）。これで JSON を捨てると
        #   全部の板が使えなくなる。致命だけで止める。
        # ⚠ **前より重い解で上書きしない。** 境界条件は増える一方で
        #   （ねじを実体化すれば座面と逃げが増える）、同じ板でも前より
        #   太い解が出ることがある。前の輪郭で組立検査が通っているなら、
        #   重いほうへ置き換える理由は無い。実際 2026-08-02 の解き直しで
        #   yaw_arm_r が 319 → 407g、pitch_side_R が 194 → 345g になり、
        #   黙って上書きされていた（面積で比べれば気づける）。
        path = os.path.join(OUT, f"{r.name}.json")
        heavier = False
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fp:
                prev = json.load(fp)
            # ⚠ **据え置きの条件は「前の輪郭で組立が成立していること」。**
            #   前の形がいまの境界条件で致命（ねじ座面が輪郭の外＝留められない）
            #   なら、軽さを理由に残す意味は無い。実際 13 枚がこの状態で、
            #   `topo_check` が「ねじが留められない（致命的）」を 300 件以上
            #   出しているのに、解き直しても「前より太い」で毎回据え置かれ、
            #   **直しても直らない**状態になっていた。
            prev_bad = unusable(shape.check(shape.from_json(prev), r))
            # ⚠ **境界条件が変わっていたら据え置けない（2026-08-05）。**
            #   前の輪郭は**前の逃げ穴・前の座面**で作った形なので、
            #   境界条件が動いた時点で「その形で組立が成立している」という
            #   据え置きの前提が消える。実測: 2 周回したのに 8 枚が
            #   `topo_check` で「境界条件が輪郭を作ったときと違う」を出し、
            #   そこから「逃げ穴に材料が残っている」が 51 件出ていた。
            #   ⚠ 据え置きは「軽いほうを選ぶ」最適化であって、**正しさより
            #     優先してよいものではない**。重くなるのは受け入れる。
            prev_stale = prev.get("bc") != bc_hash(r)
            if prev.get("area") and out.area > prev["area"] * 1.001 \
                    and not prev_bad and not prev_stale:
                heavier = True
                bad = bad + [f"前の輪郭 {prev['area']:.0f}mm² より太い "
                             f"{out.area:.0f}mm²。据え置いた"]
            elif prev_stale:
                log = log + ["前の輪郭は境界条件が違う（逃げ穴・座面が動いた）"
                             "ので、太くても置き換える"]
            elif prev_bad:
                log = log + [f"前の輪郭は使えない指摘 {len(prev_bad)} 件"
                             "（ねじが留められない等）なので、太くても置き換える"]
        if not fatal(bad) and not heavier:
            d = shape.to_json(out)
            d["bc"] = bc_hash(r)          # 形に関わる境界条件の指紋
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(d, fp, ensure_ascii=False, indent=1)
        elif fatal(bad):
            # ⚠ **使えない解が出た板は「素の板」を輪郭として書く（2026-08-05）。**
            #   新しい解を書かないだけだと、**前の輪郭がそのまま残る**。
            #   境界条件が動いていれば、その形は逃げ穴の位置が違う板
            #   （＝軸が通らない板）なので、DXF に出したら実物にならない。
            #   ⚠ かといって**消すのも駄目**。`tr_lib.topo_plate` は輪郭が
            #     無いと例外を投げる（黙って矩形に落ちるのを禁じている）。
            #     実際 3 枚消したら `export_meshes` が
            #     `FileNotFoundError: topo_plate(yaw_side_L): 輪郭が無い`
            #     で落ちた。
            #   → 設計領域そのまま（逃げ穴だけ開けた形）を書く。重いが
            #     必ず切れるし軸も通る。**軽さより「切ったものが組める」。**
            # ⚠ **「境界条件が変わっていなければ書かない」では駄目（2026-08-05）。**
            #   前に**最適化した**輪郭が置いてあり、境界条件が変わっていない
            #   ときに素通りして、その使えない輪郭が残り続けた。実測
            #   `car_brk_L` は「拘束座が輪郭の外に 9.2mm²」で 2 周とも
            #   書き換わらなかった。素の板に**まだ置き換わっていない**なら、
            #   境界条件が同じでも置き換える。
            need = True
            if os.path.exists(path):
                with open(path, encoding="utf-8") as fp:
                    prev_d = json.load(fp)
                need = not (prev_d.get("plain")
                            and prev_d.get("bc") == bc_hash(r))
            if need:
                d = shape.to_json(shape.plain_outline(r))
                d["bc"] = bc_hash(r)
                d["plain"] = True     # 最適化していない板だと分かるように
                with open(path, "w", encoding="utf-8") as fp:
                    json.dump(d, fp, ensure_ascii=False, indent=1)
                log = log + ["使える解が出なかったので素の板（設計領域＋"
                             "逃げ穴）を書いた"]
        items.append((res, out, bad))
        g = (r.mass0 - res.mass()) * 1000.0
        print(f"{r.name:<18} 体積率 {res.vol_frac:4.0%}  最小幅 "
              f"{out.min_width:5.1f}mm  {g:+7.0f}g  "
              # ⚠ **止める不良を先に出す。** 指摘の頭から 2 件だけ出して
              #   いたので、「使えない」と言いながら画面には座面の角が
              #   1mm² 欠けた話しか出ず、理由が読めなかった。
              + ("OK" if not bad else
                 " / ".join(m.split(': ', 1)[-1][:60]
                            for m in ((fatal(bad) or []) + bad)[:2]))
              + (f"   [{'; '.join(log)}]" if log else ""))

    draw.sheet_png(items, os.path.join(OUT, "sheet.png"))
    if args.md:
        draw.review_md(items, os.path.join(ROOT, "out", "topo.md"))
    g = sum(r.mass0 - res.mass() for r, (res, _o, b) in zip(regions, items)
            if not fatal(b))
    print(f"\n使える輪郭 {sum(1 for _r, _o, b in items if not fatal(b))}/"
          f"{len(items)} 枚・{g * 1000:+.0f}g")
    return 1 if any(fatal(b) for _r, _o, b in items) else 0


if __name__ == "__main__":
    raise SystemExit(main())
