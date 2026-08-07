"""TR 本体アセンブリ — build123d ソース（STEP / URDF 共通の幾何ソース）.

リンク分割は URDF と 1:1 で対応させる。各 link_*() は「そのリンクのローカル
座標系」で形状を返し、JOINTS がリンク間の原点・軸・可動範囲を持つ。
STEP は JOINTS を使ってポーズを与えた静的配置、URDF は同じ JOINTS から
関節定義を書き出す。

    base_link
      ├─ wheel_fl / fr / rl / rr      (continuous, axis Y)
      ├─ turret_yaw                   (revolute, axis Z, ±30deg)
      │    └─ shooter_pitch           (revolute, axis Y, 20..60deg)
      │         ├─ roller_upper       (continuous, axis Y)
      │         └─ roller_lower       (continuous, axis Y)
      ├─ singulator                   (continuous, axis Y)
      └─ grabber_slide                (prismatic, axis -X, 0..P.GRAB_STROKE=316)
           └─ grabber_press           (prismatic, axis +Z, 0..80)
"""

from __future__ import annotations

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))


import math
from functools import reduce
from math import atan2, cos, degrees, hypot, radians, sin, sqrt

from build123d import Align, Box, Compound, Cylinder, Location, Pos, Rot
from cadpy.assembly import label_shape

import tr_fix as F
import tr_lib as L
import tr_params as P
from tr_lib import LEDGER, bolt_circle

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
BASE = (Align.CENTER, Align.CENTER, Align.MIN)


# ---------------------------------------------------------------------------
# 部品の登録
# ---------------------------------------------------------------------------
# ⚠ URDF 生成はリンクごとに link_*() を**直接**呼ぶ（左右の車輪で同じ
#   link_wheel(1) を 2 回呼ぶ、上下ローラーで link_roller() を 2 回呼ぶ）。
#   同名 put の検査に引っかかって **URDF が生成できなくなっていた**。
#   そのあいだ tr.urdf は手で直されず、grabber_slide の上限が 0.4246
#   （いまは 0.316）、grabber_press が 0.07（いまは 0.105）、
#   fork_tilt の原点が 0.763（いまは 0.771）と食い違ったまま残っていた。
#   生成器だけこの検査を外す（生成器に同名検査は要らない）。
PUT_ALLOW_DUP = False


def put(parts, shape, name, to=None, how=None, note="", tool=None):
    """形状に**固有名**を付け、固定関係を宣言して `parts` に積む。

    ⚠ `to` を省略できるのは `F.root()` した基準部材だけ。
      名前も固定先も書かずに `parts.append(shape)` していたのが、
      浮きと意図しない干渉の根本原因だった。
      「置いた」と「留めた」は違う。ここで両方を同時に書かせる。
    """
    # ⚠ 同じ名前で 2 回置くと、bbox は上書きされるのに固定宣言は
    #   **両方とも残る**。片方の宣言が別の部品の位置で評価され、
    #   「離れ」や「浮き」が説明のつかない形で出る。名前は一意にする。
    if name in F.BOX and not PUT_ALLOW_DUP:
        raise ValueError(f"{name}: 同じ名前で 2 回 put している")
    label_shape(shape, name)
    if to is not None:
        F.fix(name, to, how, note=note)
    elif name not in F.ROOTS:
        raise ValueError(f"{name}: 固定先が宣言されていない（基準なら F.root() すること）")
    # 後から置く部品が**この部品の面**を参照できるように bbox を記録する。
    # 座標を手で書くと、相手を動かした瞬間に浮くか食い込む。
    F.BOX[name] = shape.bounding_box()
    # ねじは「工具が入る空間」も持つ。締まっていても回せなければ組めない。
    if tool is not None:
        F.TOOL[name] = tool
    # 配線は折れ線を控える。曲げ半径が足りるかは節点の角度で決まる。
    if getattr(L, "LAST_CABLE", None) is not None:
        F.CABLE[name] = L.LAST_CABLE
        L.LAST_CABLE = None
    # ⚠ **押出材は名前で見分けている**（`L.EXT_PREFIX`）。名前を載せ忘れると
    #   `screw_place` は中実材として扱い、板厚が足りればタップ、足りなければ
    #   貫通させて六角ナットを選ぶ。**2020 の中空にナットは入らない**ので、
    #   どちらも実機では組めない締結が図に出る（`disp_beam` が実際そうだった）。
    #   形を作った側が印を残すので、ここで名前と突き合わせる。
    if getattr(L, "LAST_EXT", False):
        L.LAST_EXT = False
        if not L.is_extrusion(name):
            raise ValueError(
                f"{name}: 押出材なのに L.EXT_PREFIX に載っていない"
                "（溝ナットで留める相手だと分からないので、ねじが中実材向けになる）")
    parts.append(shape)
    return shape


def put_screw(parts, shape, name, to, how, kind, size, length,
              extras=(), note="", tool=None):
    """ねじ 1 本を置き、**員数表に載る規格**（種類・呼び径・長さ）も控える。

    ⚠ `put()` だけで置いたねじは、図には出るが**買う数には出てこない**。
      実際 56 本のねじが図にあったのに、質量台帳は「ボルト・ナット・
      スペーサ類 700g」という 1 行の概算のままだった。置いた本数と
      買う本数が別々に管理されると、必ずどちらかが古くなる。
      置く場所がここ 1 つなら、ずれようがない。

    `extras` は同じ穴に要る**共締め品**の並び（("HEXNUT", 4) など）。
    ねじ・ナット・座金は必ず組で要るので、ねじを置いた場所でまとめて書く。
    """
    put(parts, shape, name, to=to, how=how, note=note, tool=tool)
    F.FASTENER[name] = (kind, size, length, tuple(extras))
    return shape


def tool_ray(loc, p_local, d_local):
    """局所座標の (点, 向き) を、配置 `loc` を掛けた**世界座標**へ移す。

    ⚠ 工具アクセスの宣言（F.TOOL）は世界座標で持つ。ブラケットのように
      4 象限へ回る部品では、局所で書いた頭の位置をそのまま渡すと
      **回転前の座標**を宣言することになり、検査は無関係な場所の空きを
      見る。回転・鏡映を掛けた後の点と向きを、置いたのと同じ変換から取る。
    """
    p0 = (loc * Pos(*p_local)).position
    p1 = (loc * Pos(p_local[0] + d_local[0], p_local[1] + d_local[1],
                    p_local[2] + d_local[2])).position
    return (p0.X, p0.Y, p0.Z, p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z)


def _member_under(pt, dirv, cands, depth: float = 3.0):
    """点 `pt` から `dirv` の**逆向き**に `depth` 進んだ所にある部材を返す。

    ⚠ L 金具は 4 象限へ回り、柱脚では倒れる。どちらの腕がどちらの部材に
      当たるかは、回転角の表を見比べないと分からない（実際 Rz が 0/180 と
      ±90 とで腕の役割が入れ替わる）。**当たり先は幾何から決める**。
      表を書き写す方式にすると、1 か所書き間違えても図は正しく見える。
    """
    px = pt[0] - dirv[0] * depth
    py = pt[1] - dirv[1] * depth
    pz = pt[2] - dirv[2] * depth
    for nm in cands:
        b = F.BOX.get(nm)
        if b is None:
            continue
        if (b.min.X <= px <= b.max.X and b.min.Y <= py <= b.max.Y
                and b.min.Z <= pz <= b.max.Z):
            return nm
    raise ValueError(
        f"({px:.1f},{py:.1f},{pz:.1f}) に {cands} のどれも無い"
        f" ← ねじの先が何にも刺さっていない")


def bracket_screws(parts, loc, brk, targets, flip_y: bool = False,
                   grip: float = 6.0):
    """L 金具の腕 2 枚に、溝ナット用のねじを 1 本ずつ通す。

    `loc` は金具を置いたのと**同じ**配置（`Pos(...) * Rot(...)`）。
    ねじの穴位置は `L.BRACKET_HOLE` にあるので、同じ変換を掛ければ芯が合う。

    ⚠ 金具は「浮いていない」ことの証明にはなるが、**留まっている**ことの
      証明にはならない。ねじが無ければ実機では手で押さえているのと同じ。
      骨格の 46 個の L 金具は荷重の主経路（上部質量の転倒モーメントが
      全部ここを通る）なので、ここを実体で描くのを最優先にする。
    """
    sy = -1.0 if flip_y else 1.0
    out = []
    for k, (p_local, d_local, flat) in enumerate((
            # 腕 a（板厚は Y）… ねじは -Y へ入る／腕 b（板厚は X）… -X へ入る
            # 腕 a を皿ねじにするのは、内隅で頭どうしが当たるのを避けるため
            # （理由は L.BRACKET_CSK_DIA のコメント）
            ((L.BRACKET_HOLE, sy * grip, 0.0), (0.0, sy, 0.0), True),
            ((grip, sy * L.BRACKET_HOLE, 0.0), (1.0, 0.0, 0.0), False))):
        ray = tool_ray(loc, p_local, d_local)
        # 座面から板厚ぶん入った先に相手の材がある（穴の中ではなく材の中を探る）
        tgt = _member_under(ray[:3], ray[3:], targets, depth=grip + 2.0)
        nm = f"scr_{brk}_{k}"
        # ⚠ 頭が当たるのは金具、ねじ山が噛むのは溝ナット。金具には
        #   バカ穴（φ5.5）が開いているので THRU、押出材へは TNUT。
        put_screw(out, loc * Pos(*p_local) * Rot(*_axis_rot(d_local))
                  * L.screw_tnut(5, grip, flat=flat),
                  nm, to=(brk, tgt), how=("THRU", "TNUT"),
                  kind="FLAT" if flat else "CAP", size=5,
                  length=L.screw_len(grip, 6.0),
                  extras=(("TNUT", 5),),
                  note="1-M5" + ("皿" if flat else "") + " + 後入れナット",
                  tool=ray)
    parts.extend(out)
    return out


def flange_screws(parts, loc, plate, target, pcd, n, size, grip, tag,
                  start=45.0, engage=None, extras=None, how_b="SCREW_IN",
                  link="base", kind="CAP"):
    """**ボルト円（PCD）**に実体のねじを置く。

    `loc` は「座面の中心を原点、+Z がねじを抜く向き（＝工具を差す向き）」
    とした配置。ねじは -Z へ入る。

    ⚠ ここは `scripts/screw_place.py` に**やらせてはいけない**。あちらは
      2 部品の接触面から場所を「探す」が、モーター・軸受・リニアブッシュの
      取付穴は**買った部品が決めている**（M3508 は 4-M4 PCD35、M2006 と
      M2006 用ブラケットは 4-M3 PCD26）。探して出てきた位置は、たとえ
      幾何的に成立していても実物の穴と合わない。実際この 20 組は
      「接触面に両方の材料がある場所が無い」で 1 本も置けていなかった
      （筒の中に筒が入る嵌合なので、平らな接触面がそもそも無い）。
    """
    engage = 1.5 * size if engage is None else engage
    ln = L.screw_len(grip, engage)
    # ⚠ **モーターのボルト円は「頭が出る余裕」が無いことが多い。** 取付板の
    #   すぐ外にプーリ・カップリング・支柱が来るので、六角穴付きの頭
    #   （M4 で高さ 4）がそのまま当たる（実測: ヨー支柱 33mm³ ×4、
    #   グラバーのプーリ 22mm³ ×4、昇降のプーリと仰角のカップリングは接触）。
    #   板厚が皿もみのぶんあるなら `kind="FLAT"` にして頭を沈める。
    body = L.screw_flat if kind == "FLAT" else L.screw
    out = []
    # ⚠ 工具アクセスの宣言は**世界座標**。可動リンクの中で置くねじは
    #   リンクの変換を掛けてから渡さないと、無関係な場所の空きを見ることになる
    #   （`auto_screws` と同じ註）。
    wloc = LINK_LOC.get(link, Location()) * loc
    for i, (px, py) in enumerate(bolt_circle(n, pcd, start)):
        ray = tool_ray(wloc, (px, py, grip), (0.0, 0.0, 1.0))
        put_screw(out, loc * Pos(px, py, grip) * body(size, ln),
                  f"scr_{tag}{i}", to=(plate, target),
                  how=("THRU", how_b), kind=kind, size=size, length=ln,
                  extras=(("WASHER", size),) if extras is None else extras,
                  note=f"1-M{size}", tool=ray)
    parts.extend(out)
    return out


def face_screws(parts, locs, a, b, size, grip, tag, kind="CAP", engage=None,
                how_b="SCREW_IN", extras=None, link="base", length=None,
                note=None):
    """**軸に平行でない当たり面**にねじを置く。

    `locs` は 1 本ごとの配置で、**原点が頭の座面・+Z が工具を差す向き**
    （ねじは -Z へ入る）。`grip` は座面から相手の面までの距離。

    ⚠ `screw_place.py` は接触面を bbox の重なりから探すので、**傾いた当たり面は
      見つけられない**。昇降コンベアのガイド板は 63° 傾いていて、そこに載る組は
      全部「接触面に両方の材料がある場所が無い」で落ちていた。傾きを知っている
      のは置いた側だけなので、ここで直接置く（`F.SLOT_AXIS` と同じ考え方）。
    """
    engage = 1.5 * size if engage is None else engage
    ln = L.screw_len(grip, engage) if length is None else float(length)
    body = L.screw_flat if kind == "FLAT" else L.screw
    out = []
    for i, lc in enumerate(locs):
        ray = tool_ray(LINK_LOC.get(link, Location()) * lc,
                       (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        put_screw(out, lc * body(size, ln), f"scr_{tag}{i}", to=(a, b),
                  how=("THRU", how_b), kind=kind, size=size, length=ln,
                  extras=(("WASHER", size),) if extras is None else extras,
                  note=note or f"1-M{size}", tool=ray)
    parts.extend(out)
    return out


def _axis_rot(d_local):
    """局所の**差し込み向き**を、`L.screw()`（軸 -Z）に掛ける回転へ直す。"""
    dx, dy, dz = d_local
    if abs(dz) > 0.5:
        return (0, 0, 0) if dz > 0 else (180, 0, 0)
    if abs(dy) > 0.5:
        return (-90, 0, 0) if dy > 0 else (90, 0, 0)
    return (0, 90, 0) if dx > 0 else (0, -90, 0)


# ---------------------------------------------------------------------------
# 宣言だけで実体の無かったねじを、凍結した位置から置く
#
# ⚠ **「留まっている」と書いてあるだけで図に無いねじが 622 本あった。**
#   買う数と質量は宣言から見積もれていたが、頭が座る面が本当にあるのか、
#   相手を貫いていないかは実体を置かないと言えない。図を見た人にも
#   「留まっていない」ように見える。
#
#   位置は `scripts/screw_place.py` が接触面の幾何から決めて
#   `out/screws.json` に凍結する。ここは**読んで置くだけ**。
#   組立 → 位置計算 → 組立 の循環を断つのが凍結の目的で、
#   `out/topo/` の輪郭と同じ考え方。
# ---------------------------------------------------------------------------
# 位置計算のときだけ False にする（前回のねじが接触面の判定に混ざるのを防ぐ）
AUTO_SCREWS = True
_SCREWS: dict | None = None
# リンクの世界変換。build() が姿勢ごとに記録する。凍結した位置は
# POSE_MATCH のリンクローカルなので、他の姿勢では**そのリンクの変換**を掛ける。
LINK_LOC: dict[str, Location] = {}
SCREWS_JSON = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                            "..", "out", "screws.json")


def _screws_json() -> dict:
    global _SCREWS
    if _SCREWS is None:
        import json
        try:
            with open(SCREWS_JSON, encoding="utf-8") as fp:
                _SCREWS = json.load(fp)
        except FileNotFoundError:
            # ⚠ 無くても組めるようにする（初回はここが空でないと
            #   位置計算そのものが走らせられない）。検査は
            #   scripts/screw_check.py が別途「足りない」と言う。
            _SCREWS = {"bc": "", "screws": []}
    return _SCREWS


def _rivet(dia: float, length: float):
    """ブラインドリベット。頭の座面を z=0 に置き、軸は -Z へ伸びる。"""
    head = Pos(0, 0, 0.6) * Cylinder(dia, 1.2, align=CTR)
    shank = Pos(0, 0, -length / 2) * Cylinder(dia / 2, length, align=CTR)
    return L.mat(head + shank, "STEEL")


def _rivet_flat(dia: float, length: float):
    """皿頭ブラインドリベット（120°）。**頭の上面が z=0**、軸は -Z へ伸びる。

    ⚠ 六角穴付き `screw()` や丸頭 `_rivet()` と**原点の意味が違う**
      （`L.screw_flat` と同じ約束）。あちらは頭の座面が z=0 で頭は面の上に
      出るが、皿は頭が板の中に沈むので上面が z=0。同じ原点で置くと、
      皿リベットが板の上に浮いて座らない。
    """
    from build123d import Cone
    hh = L.rivet_flat_head_h(dia)
    head = Pos(0, 0, -hh / 2) * Cone(dia / 2, L.rivet_flat_head(dia) / 2, hh,
                                     align=CTR)
    shank = Pos(0, 0, -(hh + (length - hh) / 2)) * \
        Cylinder(dia / 2, max(0.1, length - hh), align=CTR)
    return L.mat(head + shank, "STEEL")


def auto_screws(parts, link: str):
    """`link` に属する自動配置ねじを `parts` に積む。

    ⚠ 呼ぶのは**そのリンクの部品を全部置いた後**。ねじは put() で
      固定宣言も出すので、相手がまだ無いと「固定先が存在しない」になる。
    """
    if not AUTO_SCREWS:
        return []
    out = []
    for d in _screws_json()["screws"]:
        if d.get("link") != link:
            continue
        size = int(d["size"])
        grip = d.get("grip", 6.0)
        if d["kind"] == "RIVET":
            body = _rivet(d["size"], d["length"])
        elif d["kind"] == "RIVET_FLAT":
            body = _rivet_flat(d["size"], d["length"])
        elif any(e[0] == "TNUT" for e in d["extras"]):
            # ボルトと後入れナットは**1 つの締結具**として扱う（買うときも組）
            body = L.screw_tnut(size, grip, flat=d["kind"] == "FLAT")
        elif d["kind"] == "FLAT":
            body = L.screw_flat(size, d["length"])
        else:
            body = L.screw(size, d["length"])
        loc = Pos(*d["pos"]) * Rot(*_axis_rot(d["dir"]))
        # ⚠ **貫く相手・触れる相手も宣言する。** 候補を全部試しても
        #   3 つ目の部品をかすめる箇所は残る（柱の角・既にあるねじとの交差）。
        #   黙って置くと「宣言のない接触」として検査に出るだけなので、
        #   貫くなら THRU と書く（実機ではその部材に穴が要る、という意味）。
        thru = [t for t in d.get("thru", ()) if t not in (d["a"], d["b"])]
        put_screw(out, loc * body, d["name"],
                  to=(d["a"], d["b"], *thru),
                  how=tuple(d["how_ab"]) + ("THRU",) * len(thru),
                  kind=d["kind"], size=d["size"],
                  length=d["length"], extras=[tuple(e) for e in d["extras"]],
                  note=d.get("note", ""),
                  # 工具は頭の側から差す。`dir` は頭が向いている向き。
                  # ⚠ 工具アクセスの宣言は**世界座標**。凍結した位置は
                  #   リンクローカルなので、そのまま渡すと可動リンクの
                  #   ねじだけ無関係な場所の空きを見ることになる。
                  tool=tool_ray(LINK_LOC.get(link, Location()),
                                d["pos"], d["dir"]))
        for t in d.get("touch", ()):
            if t not in (d["a"], d["b"]) and t not in thru:
                F.touch_ok(d["name"], t)
    parts.extend(out)
    return out

# ---------------------------------------------------------------------------
# 関節定義（URDF と共有）
#   origin  : 親リンク座標系での関節原点 [mm]
#   axis    : 関節軸（親リンク座標系）
#   limits  : (min, max) 回転は deg、直動は mm
# ---------------------------------------------------------------------------
JOINTS = {
    "wheel_fl": dict(parent="base_link", type="continuous", axis=(0, 1, 0),
                     origin=(P.WHEELBASE_X / 2, P.TRACK_Y / 2, P.AXLE_Z), limits=None),
    "wheel_fr": dict(parent="base_link", type="continuous", axis=(0, 1, 0),
                     origin=(P.WHEELBASE_X / 2, -P.TRACK_Y / 2, P.AXLE_Z), limits=None),
    "wheel_rl": dict(parent="base_link", type="continuous", axis=(0, 1, 0),
                     origin=(-P.WHEELBASE_X / 2, P.TRACK_Y / 2, P.AXLE_Z), limits=None),
    "wheel_rr": dict(parent="base_link", type="continuous", axis=(0, 1, 0),
                     origin=(-P.WHEELBASE_X / 2, -P.TRACK_Y / 2, P.AXLE_Z), limits=None),
    "turret_yaw": dict(parent="base_link", type="revolute", axis=(0, 0, 1),
                       origin=(P.TURRET_X, 0.0, P.PEDESTAL_TOP_Z),
                       limits=(-P.YAW_LIMIT, P.YAW_LIMIT)),
    "shooter_pitch": dict(parent="turret_yaw", type="revolute", axis=(0, -1, 0),
                          origin=(0.0, 0.0, P.NIP_Z - P.PEDESTAL_TOP_Z),
                          limits=(P.PITCH_MIN, P.PITCH_MAX)),
    "roller_upper": dict(parent="shooter_pitch", type="continuous", axis=(0, 1, 0),
                         origin=(0.0, 0.0, (P.ROLLER_DIA + P.NIP_GAP) / 2), limits=None),
    "roller_lower": dict(parent="shooter_pitch", type="continuous", axis=(0, 1, 0),
                         origin=(0.0, 0.0, -(P.ROLLER_DIA + P.NIP_GAP) / 2), limits=None),
    "singulator": dict(parent="base_link", type="continuous", axis=(0, 1, 0),
                       origin=(P.HOP_X1 + 5.0, 0.0, P.FEED_RAMP_Z0 + P.PICK_ROLLER_DIA / 2),
                       limits=None),
    "grabber_slide": dict(parent="base_link", type="prismatic", axis=(-1, 0, 0),
                          origin=(0.0, 0.0, 0.0), limits=(0.0, P.GRAB_STROKE)),
    "grabber_press": dict(parent="grabber_slide", type="prismatic", axis=(0, 0, -1),
                          origin=(0.0, 0.0, 0.0), limits=(0.0, P.PRESS_STROKE)),
    # 櫛歯の傾斜。専用モーターは持たず、引込み端の固定カムが押し下げる受動関節。
    "fork_tilt": dict(parent="grabber_slide", type="revolute", axis=(0, -1, 0),
                      # ⚠ 関節原点＝回転中心＝**ヒンジ軸の位置**。
                      #   FORK_Z（櫛歯の上面）に置くと軸が櫛歯を貫くので、
                      #   軸半径 4 + 余裕 4 = 8mm 上に取る。
                      #   リンク内の部品はその分だけ局所 Z を下げる。
                      origin=(P.RAIL_X0 + P.FORK_LEN, 0.0,
                              P.FORK_Z + P.FORK_HINGE_UP),
                      limits=(0.0, P.FORK_TILT_MAX)),
}


# ===========================================================================
# base_link : 車体（非可動部すべて）
# ===========================================================================
def _frame_members():
    """ベースフレーム HFS5-2020 井桁。

    長手材（主桁・内桁）を通し、**横材は長手材のあいだの寸法に切って**
    ブラケットで留める。以前は横材を端から端まで 1 本で通していたので、
    交差部で実体が 20mm 素通しに重なっていた（食い込み 1885mm³ × 4 組）。
    CAD 上は描けるが、実機ではアルミフレームは互いを貫通できない。

    切り分けで横材は 4 本 → 12 本になる。部品は増えるが、これが実際に
    発注・切断・組立する姿。
    """
    z = P.BASE_Z0 + P.EXT_W / 2
    rails_y = (-P.FRAME_RAIL_Y_OUT, -P.FRAME_RAIL_Y_IN, P.FRAME_RAIL_Y_IN, P.FRAME_RAIL_Y_OUT)
    parts = []
    # --- 長手材（通し）---
    # この 4 本が機体の**基準部材**。ここから固定の連鎖が始まる。
    # 長手材どうしは直接触れないので、互いを固定先にはできない。
    # 横材が「どの長手材に留まるか」を宣言し、そちらから連鎖がつながる。
    for nm, y in (("rail_L_out", P.FRAME_RAIL_Y_OUT), ("rail_R_out", -P.FRAME_RAIL_Y_OUT),
                  ("rail_L_in", P.FRAME_RAIL_Y_IN), ("rail_R_in", -P.FRAME_RAIL_Y_IN)):
        F.root(nm)
        put(parts, Pos(0, y, z) * L.ext2020(P.BASE_X), nm)
        L.add_ext(f"ベース長手材 {nm}", P.BASE_X, "シャシー")
    # --- 横材（ステーションごとに長手材のあいだで切る）---
    # ⚠ ステーションは**モーターマウントの X 範囲（235..365）の外**に取る。
    #   車軸位置（±300）に置くと、縦板（|Y| 256..262）と横材（|Y|<=275）が
    #   必ず交差する。縦板を外へ逃がす余地は無い（内桁の外面 295 と
    #   車輪の内面 275 のあいだに板の居場所が無い）ので、横材を動かす。
    for x in (P.BASE_X / 2 - P.EXT_W / 2, 210.0, 110.0,
              -110.0, -210.0, -(P.BASE_X / 2 - P.EXT_W / 2)):
        # 端の2ステーションは主桁まで通す（側面トラス柱の直下＝荷重が集まる）。
        # 車軸位置の2ステーションはモーターマウントを受けるだけなので内桁まで。
        outer = abs(x) > P.WHEELBASE_X / 2 + 1
        y_end = (P.FRAME_RAIL_Y_OUT if outer else P.FRAME_RAIL_Y_IN) - P.EXT_W / 2
        tag = f"x{x:+04.0f}".replace("+", "p").replace("-", "m")
        for yc, ln in L.ext_seg(-y_end, y_end, rails_y[1:3]):
            # 両端が突き当たる長手材を名前で特定する（ここが「何に留まるか」）
            ends = _rail_at(yc - ln / 2, yc + ln / 2, outer)
            side = "mid" if abs(yc) < 1 else ("L" if yc > 0 else "R")
            nm = f"cross_{tag}_{side}"
            put(parts, Pos(x, yc, z) * Rot(0, 0, 90) * L.ext2020(ln), nm,
                to=ends, how="BRACKET")
            L.add_ext(f"ベース横材 {nm}", ln, "シャシー")
            # ⚠ ブラケットは**長手材が実在する側**に置くこと。
            #   端のステーション（X=±410）の外側には長手材が無く、
            #   角の線でしか当たらない（座面 0mm²）。内側へ向ける。
            # ⚠ X=+210 のステーションは、+X 面に置くと**六角レンチが入らない**。
            #   前輪のモーターマウント（X 235..365）がすぐ外にあり、頭から
            #   工具を差す 45mm を塞ぐ。-X 面へ回す（X=-210 側は後輪マウントが
            #   -365..-235 なので +X 面のままでよい）。
            arm = (-1,) if (x > 0 and outer) or abs(x - 210.0) < 1.0 else (1,)
            parts += _butt_brackets(x, yc, ln, z, "シャシー", nm, ends, arm)
    return parts


def _declare_touches():
    """固定関係ではないが**接して当然**の組を明示する。

    ⚠ ここに書けるのは「同じ部材に留まっている隣どうしが、たまたま面で
      接している」ような組だけ。**離れているべきものを黙らせるために
      書いてはいけない。** 書いた瞬間その組は検査から外れる。
      検査を通すために宣言を足すのは、検査を壊すのと同じ。
    """
    # ブラケットと、同じ長手材に留まる別部品（座面を分け合う）
    for sd, sy in (("L", 1), ("R", -1)):
        for st in ("xp300", "xm300"):
            F.touch_ok(f"brk_cross_{st}_mid_{'p' if sy > 0 else 'm'}",
                       f"mount_brk_{'f' if st[1] == 'p' else 'r'}{'l' if sy > 0 else 'r'}",
                       "同じ内桁の上面を分け合う")
    F.touch_ok("brk_cross_xp110_mid_p", "chair_mount_1", "同じ横材の上面")
    for sd in ("f", "r"):
        F.touch_ok(f"yaw_arm_{sd}", "yaw_pulley_big", "同じ旋回リングの上に並ぶ")
        F.touch_ok(f"yaw_arm_{sd}", "yaw_pulley_small", "旋回アームの上のモーター駆動系")
    for up in ("u", "d"):
        for k in range(2):
            F.touch_ok(f"nip_guide_{up}{k}", f"nip_guide_{up}{k + 1}", "3 分割ガイドの隣どうし")
    for st in ("xp410", "xm410"):
        for sd in ("L", "R", "mid"):
            F.touch_ok(f"cross_{st}_{sd}", "chair_mount_2", "同じ横材の上面")
            F.touch_ok(f"cross_{st}_{sd}", "chair_mount_3", "同じ横材の上面")
    F.touch_ok("brk_cross_xp410_L_m", "chair_mount_2", "同じ横材の上面")
    # --- 柱・梁とスカート -----------------------------------------------
    # スカートは骨格の外周を覆う膜。柱やブラケットの外面に当たって当然。
    for sd in ("L", "R"):
        F.touch_ok(f"post_front_{sd}", f"skirt_{sd}", "スカートが柱の外面を覆う")
        F.touch_ok(f"post_rear_{sd}", f"skirt_{sd}", "スカートが柱の外面を覆う")
        for pt in ("front", "rear"):
            for tag in ("m", "p"):
                F.touch_ok(f"brk_post_{pt}_{sd}_{tag}", f"skirt_{sd}",
                           "スカートがブラケットの外面を覆う")
        # 柱脚のガセットと、同じ柱に留まるブラケット
        for tag in ("m", "p"):
            F.touch_ok(f"brk_post_rear_{sd}_{tag}", f"gus_brace_{sd}_lo",
                       "同じ柱の脚元を分け合う")
        # レール取付プレートは上桁の内面いっぱいに張るので、そこへ留まる
        # 金具・柱・筋交い・非常停止板の側面に当たる
        F.touch_ok(f"post_rear_{sd}", f"rail_plate_{sd}", "柱の内面にプレートが接する")
        F.touch_ok(f"brace_{sd}", f"rail_plate_{sd}", "筋交いの側面にプレートが接する")
        for bt in ("brk_topbeam0_" + sd + "_pu", "brk_topbeam0_" + sd + "_pd",
                   "brk_topbeam1_" + sd + "_mu", "brk_topbeam1_" + sd + "_md"):
            F.touch_ok(bt, f"rail_plate_{sd}", "同じ上桁の内面を分け合う")
        for bt in ("brk_topbeam1_" + sd + "_pd",):
            F.touch_ok(bt, "pedestal_beam1", "台座横梁の端が上桁の金具に接する")
        # 前柱と横材のブラケットが同じ隅に集まる
        F.touch_ok(f"brk_cross_xp410_{sd}_pn", f"brk_post_front_{sd}_p",
                   "同じ隅に集まる金具どうし")
        F.touch_ok(f"brk_cross_xp410_{sd}_mn", f"brk_post_front_{sd}_p",
                   "同じ隅に集まる金具どうし")
        F.touch_ok(f"brk_cross_xp410_{sd}_pn", f"post_front_{sd}",
                   "金具が柱の面に当たる")
        F.touch_ok(f"brk_cross_xp410_{sd}_mn", f"post_front_{sd}",
                   "金具が柱の面に当たる")
        F.touch_ok(f"cross_xp410_{sd}", f"brk_post_front_{sd}_p",
                   "横材の端が柱の金具に接する")
        F.touch_ok(f"gus_brace_{sd}_hi", f"scr_J6_{sd}_4",
                   "レール取付ねじがガセットの脇を通る")
        F.touch_ok(f"brk_cross_xp410_{sd}_mn", "chair_mount_2", "同じ横材の上面")
        F.touch_ok(f"estop_plate_{'f' if sd == 'L' else 'r'}", f"rail_plate_{sd}",
                   "非常停止板が上桁のプレートに接する")
        # マスト横梁の継手は、内隅の L 金具（横梁の下面）と ±X 面の
        # ジョイントプレートが**同じ隅の稜線**で出会う。金具は
        # X -285.4..-265.4、プレートは X -265.4 から外側なので重なりは無く、
        # 稜線 1 本で触れるだけ。実機でも並べて締める組み合わせ。
        for sx in ("p", "m"):
            F.touch_ok(f"brk_mast_corner_{sd}", f"brk_mast_cross_{sd}{sx}",
                       "同じ継手の内隅と側面を分け合う")
    F.touch_ok("brk_cross_xp410_mid_pn", "chair_mount_2", "同じ横材の上面")
    F.touch_ok("brk_cross_xp110_mid_p", "chair_mount_0", "同じ横材の上面")
    F.touch_ok("brk_pedestal_beam1_pn", "estop_plate_f", "同じ梁の端に集まる")
    F.touch_ok("skirt_front", "chair_mount_2", "スカートが脚の外面を覆う")
    F.touch_ok("skirt_front", "chair_mount_3", "スカートが脚の外面を覆う")
    # --- 可動部と固定部のすきま（動作範囲で接する）------------------------
    for sd in ("L", "R"):
        F.touch_ok("pedestal_beam0", f"car_side_{sd}",
                   "キャリッジが引込み端で台座横梁の脇を通る")
        F.touch_ok("pedestal_beam0", f"ramp_strut_{sd}",
                   "斜路支柱が梁の下面に沿う")
        F.touch_ok(f"car_brk_{sd}", f"belt_clamp_{sd}",
                   "同じキャリッジの上で隣り合う")
        F.touch_ok(f"post_front_{sd}", "pedestal_beam1", "梁の端が柱の面に接する")
    for _i in (0, 1):
        F.touch_ok(f"pedestal_ring{_i}", "yaw_pulley_big",
                   "旋回リングの外周に歯付きリングが被る")
    # --- 2026-08-05 の部品分割で生まれた接触 --------------------------------
    # ⚠ 曲げ品を「平板 2 枚 + 端面タップ」に置き換えたので、**元は 1 部品
    #   だった面**が別部品の境になった。留まっているのは片方（多くは立板）
    #   だけなので、もう片方との接触が「宣言のない接触」として出る。
    #   どれも同じ組立の中で隣り合う板で、意図しない干渉ではない。
    #   ⚠ ここへ足すのは**分割の副作用だけ**。可動部どうしの接触を
    #     見逃さないよう、理由を 1 行ずつ書く（`touch_ok` の運用）。
    for _sd in ("L", "R"):
        for _t in ("f", "r"):
            F.touch_ok(f"brk_yaw_{_sd}{_t}", f"yaw_side_{_sd}",
                       "受け金具の水平板。側板は立板（…v）へ留まる")
            F.touch_ok(f"yaw_arm_{_t}", f"brk_yaw_{_sd}{_t}v",
                       "受け金具の立板。アームへは水平板が留まる")
        F.touch_ok("car_beam", f"car_side_{_sd}",
                   "横梁の板。側板へは耳（car_beam_e…）が留まる")
        F.touch_ok(f"car_brk_{_sd}", f"car_side_{_sd}",
                   "受け金具の水平板。側板へは立板（…v）が留まる")
        F.touch_ok(f"car_brk_{_sd}", f"press_post_{_sd}f0",
                   "受け金具の水平板と柱の下フランジが重なる")
        F.touch_ok("pedestal_beam0", f"ramp_seat_{_sd}v",
                   "座金具の垂直板が梁の側面に沿う")
        F.touch_ok(f"post_rear_{_sd}", f"hop_hanger_{_sd}0",
                   "吊り金具の水平板。柱へは立板（…v）が留まる")
        F.touch_ok(f"post_rear_{_sd}", f"hop_hanger_{_sd}1",
                   "吊り金具の水平板。柱へは立板（…v）が留まる")
        F.touch_ok("hop_front", f"ramp_foot_{_sd}e",
                   "斜路下端受けの耳板が前壁に沿う")
        F.touch_ok(f"press_guide_{_sd}", f"press_clamp_{_sd}",
                   "シャフトクランプがガイド軸を掴む")
    for _i in range(P.FORK_TINES):
        F.touch_ok(f"tine{_i}", f"fork_clamp{_i}",
                   "割締めクランプが櫛歯の上（底板の上）に載る")
    F.touch_ok("fork_root", "fork_root_v",
               "根元バーの底板と立板。留まるのは立板 ↔ クランプ（同じ組立）")
    F.touch_ok("tine2", "cam_follower",
               "カムフォロアの外周が中央の櫛歯の上面に接する（同じ fork リンク）")
    # ⚠ 「ウォームが側板の内面に沿う」という TOUCH_OK は**消した**。
    #   ウォームは回る部品で、側板は仰角ユニットの構造材。触れていたら
    #   実機では擦れて削れる。歯先半径 13.5 に対して軸線を内面から 15mm に
    #   置き、1.5mm 空けてある（tr_assembly.link_pitch の wh_y）。
    F.touch_ok("beltbrk_drv_L", "grab_motor", "同じ金具の脇にモーターが並ぶ")
    # プーリ軸受のブッシュは、ブラケットの穴に圧入されている。ブラケットは
    # 板（レール取付プレート / 上桁）にボルト留めなので、ブッシュの背面が
    # その板に当たる。金具の厚みぶん奥にあるだけで、留まる相手ではない。
    for sd_ in ("L", "R"):
        F.touch_ok(f"beltbush_drv_{sd_}", f"rail_plate_{sd_}",
                   "ブッシュの背面が、ブラケットを留めている板に当たる")
        F.touch_ok(f"beltbush_idl_{sd_}", f"topbeam0_{sd_}",
                   "ブッシュの背面が、ブラケットを留めている桁に当たる")
    # ⚠ 以前は「軸受の外側でカップリングが回る」と touch_ok を書いていた。
    #   回る継手が固定の軸受ブロックに**接してよい**と宣言していたわけで、
    #   実機ではそこが擦れて削れる。継手を側壁の外（|Y| 316 から）へ出し、
    #   ブロック（310 まで）と 6mm 空けた。宣言そのものが要らなくなった。
    # ⚠ 「ウォーム軸受ブロックがガイド板の逃げの縁に接する」という TOUCH_OK は
    #   **消した**。逃げの中心をウォーム軸に合わせ直して 4〜5mm 空けたので、
    #   接する理由がなくなった。ここが接していると、ガイド板（PETG t2）が
    #   ブロックに押されて反る。
    # ⚠ ヨーモーターの取付ねじは**皿**にして頭を天板の下面と面一にした。
    #   面一なので、天板が支柱の上面に載っているのと同じ形で頭も支柱に
    #   当たる。ボルト円（PCD35・半径 17.5）と支柱（|X| 14..30）は、
    #   4 本 90° 間隔である以上どう位相を回しても 2 本が支柱の上に来る
    #   （|cosθ| と |sinθ| を同時に 0.6 未満にはできない）。頭が出ていない
    #   以上これは干渉ではないので、接触として宣言する。
    for _sx in ("p", "m"):
        for _i in range(4):
            F.touch_ok(f"yaw_motor_post_{_sx}", f"scr_yaw_mot{_i}",
                       "皿の頭は天板の下面と面一。支柱の上面に当たるのは天板と同じ")
    # ヨーを振ったときだけ接する組（可動域の端で当たる）
    F.touch_ok("cab_turret", "yaw_pulley_big",
               "ヨー ±30° で砲塔系ハーネスが歯付きリングの脇をかすめる")
    F.touch_ok("roller_belt_u", "nip_wedge_L",
               "ニップ調整くさびの側面をローラー駆動ベルトがかすめる")
    for tag in ("m", "p"):
        F.touch_ok(f"brk_cross_xp410_L_{tag}", "skirt_front", "スカートが横材の端面を覆う")
        F.touch_ok(f"brk_cross_xp410_R_{tag}", "skirt_front", "スカートが横材の端面を覆う")
        F.touch_ok(f"brk_cross_xm410_L_{tag}", "skirt_rear", "スカートが横材の端面を覆う")
        F.touch_ok(f"brk_cross_xm410_R_{tag}", "skirt_rear", "スカートが横材の端面を覆う")


def _rail_at(y0, y1, outer):
    """横材の端 y0/y1 に突き当たる長手材の名前を返す。"""
    lut = {round(P.FRAME_RAIL_Y_OUT - P.EXT_W / 2, 1): "rail_L_out",
           round(-(P.FRAME_RAIL_Y_OUT - P.EXT_W / 2), 1): "rail_R_out",
           round(P.FRAME_RAIL_Y_IN - P.EXT_W / 2, 1): "rail_L_in",
           round(-(P.FRAME_RAIL_Y_IN - P.EXT_W / 2), 1): "rail_R_in",
           round(P.FRAME_RAIL_Y_IN + P.EXT_W / 2, 1): "rail_L_in",
           round(-(P.FRAME_RAIL_Y_IN + P.EXT_W / 2), 1): "rail_R_in"}
    out = []
    for y in (y0, y1):
        nm = lut.get(round(y, 1))
        if nm is None:
            raise ValueError(f"横材の端 y={y:.1f} に長手材が無い（宙に浮く）")
        out.append(nm)
    return tuple(dict.fromkeys(out))


def _butt_brackets(x, yc, ln, z, group, member, ends, arm_dir=(1,), dz=0.0):
    """Y 方向の横材の両端に、内隅ブラケットを置く。

    横材の ±X 側面と、突き当たる長手材の側面を L 字でつなぐ。
    ブラケット自身も「横材と長手材の 2 つに留まる部品」として宣言する。

    ⚠ 既定は**片側 1 個**。最初 2 個ずつ置いたら 80 個 960g になり、
    35kg 規定を 1.06kg 超えた。軽荷重の T 継手は 1 個で足りる。
    """
    out = []
    for k, (sy, yend) in enumerate(((-1, yc - ln / 2), (1, yc + ln / 2))):
        for sx in arm_dir:
            nm = f"brk_{member}_{'m' if sy < 0 else 'p'}{'' if sx > 0 else 'n'}"
            end = ends[min(k, len(ends) - 1)]
            loc = (Pos(x + sx * P.EXT_W / 2, yend, z + dz)
                   * Rot(0, 0, L.BRACKET_ROT[(sx, -sy)]))
            # ⚠ note は**その組 1 つに要る本数**。金具 1 個で "2-M5" と
            #   書いていたが、宣言は相手ごとに 2 つある（横材へ / 長手材へ）
            #   ので、員数表では 4 本と数えられていた。腕 1 枚に穴は 1 つ。
            put(out, loc * L.bracket(), nm, to=(member, end), how="TSLOT",
                note="1-M5（腕 1 枚に 1 本）")
            LEDGER.add("ブラケット HBLFSN5", 0.012, "MISUMI カタログ", group)
            bracket_screws(out, loc, nm, (member, end))
    return out


# --- 斜材ガセットのボルト位置 ---------------------------------------------
# ⚠ **傾いた押出材の溝は自動配置では出せない。** `screw_place.ext_axes` は
#   bbox の辺の長さから長手軸を決めるので、斜めに寝た押出材（bbox が断面より
#   太い）では None を返し、呼ぶ側は「両方に材料がある点」を面に散らす。
#   実測すると斜材に留まる 4 本は軸線から 3.0〜6.1mm 外れていて、溝口 6mm
#   （＝軸線から ±3mm）の外、下穴の無い肌に頭だけ乗っている本数があった。
#   1 本はガセットの三角形の下 3.5mm に浮いていた。
#   斜材は 2 本しか無いので、位置をここで決めて自動配置に渡さない。
BRACE_BOLT_END = 15.0        # 斜材の端からいちばん近いボルトまで [mm]
BRACE_BOLT_PITCH_MIN = 20.0  # ボルト間の最小ピッチ（後入れナットが並ぶ）


def _brace_bolt_holes(origin, legs, axis, end, n: int = 2):
    """斜材 → ガセットのボルトを、**斜材の溝の芯**に載せて板ローカルで返す.

    `origin` … 三角形の直角の頂点（世界 XZ）
    `legs`   … (a, b) 直角から +X / +Z へ伸びる腕（符号つき）
    `axis`   … 斜材の軸方向の単位ベクトル (ux, uz)。**ガセットの中へ入る向き**
    `end`    … 斜材の実端（世界 XZ）。ここから `axis` 方向へ並べる

    置ける区間を決めているのは**斜辺までの残肉**。斜辺は板ローカルで
    x/a + z/b = 1 の直線なので、そこからの距離が
    「バカ穴の半径 + 縁の最小残肉」を下回らない範囲に収める。
    """
    a, b = legs
    ux, uz = axis
    ex, ez = end[0] - origin[0], end[1] - origin[1]
    edge = L.BRACKET_HOLE_DIA / 2 + 4.0        # 穴の縁から斜辺まで（topo の EDGE_MIN）
    grad = hypot(1.0 / a, 1.0 / b)             # x/a+z/b の勾配（1/mm）
    f0 = ex / a + ez / b                       # 斜材の端の位置（0=直角, 1=斜辺）
    g = ux / a + uz / b                        # 1mm 進むごとに斜辺へ近づく量
    assert g > 0, "斜材の軸が斜辺から遠ざかっている（向きが逆）"
    s_max = (1.0 - edge * grad - f0) / g
    assert s_max > BRACE_BOLT_END, (
        f"斜材の端から {s_max:.1f}mm しか置けない（三角形が小さすぎる）")
    step = (s_max - BRACE_BOLT_END) / (n - 1) if n > 1 else 0.0
    assert n < 2 or step >= BRACE_BOLT_PITCH_MIN, (
        f"ボルトのピッチが {step:.1f}mm しかない（最小 {BRACE_BOLT_PITCH_MIN}mm）")
    out = []
    for i in range(n):
        s = BRACE_BOLT_END + i * step
        hx, hz = ex + s * ux, ez + s * uz
        # 腕の側の残肉も見る（斜辺だけ見ていると直角の側で縁に寄りうる）
        assert min(abs(hx), abs(hz)) >= edge, (
            f"ボルトが三角形の腕から {min(abs(hx), abs(hz)):.1f}mm しか離れていない")
        out.append((hx, hz))
    return out


def _side_frames():
    """側面トラス（|Y|=350）。

    後柱はそのままマスト主柱として z=1150 まで延長し、グラバーレール受け・
    砲塔台座・マストの3役を1本で兼ねる（部材点数と質量の削減）。
    """
    parts = []
    post_h = P.BEAM_TOP_Z - P.BASE_Z1
    mast_h = P.MAST_TOP_Z - P.BASE_Z1
    beam_z = P.BEAM_TOP_Z - P.EXT_W / 2          # 上桁・台座横梁の断面中心
    for sy in (1, -1):
        y = sy * P.SIDE_FRAME_Y
        sd = "L" if sy > 0 else "R"
        base_rail = f"rail_{sd}_out"
        # 柱は通し（縦荷重の主経路）。横材のほうを切って突き合わせる
        post_rear, post_front = f"post_rear_{sd}", f"post_front_{sd}"
        put(parts, Pos(P.RAIL_X0, y, P.BASE_Z1 + mast_h / 2) * Rot(0, 90, 0) * L.ext2020(mast_h),
            post_rear, to=base_rail, how="BRACKET", note="柱脚ブラケット 2 個・各 2-M5")
        L.add_ext("側面後柱（マスト主柱兼用）", mast_h, "シャシー")
        put(parts, Pos(P.FRONT_POST_X, y, P.BASE_Z1 + post_h / 2) * Rot(0, 90, 0) * L.ext2020(post_h),
            post_front, to=base_rail, how="BRACKET", note="柱脚ブラケット 2 個・各 2-M5")
        L.add_ext("側面前柱", post_h, "シャシー")
        # 柱脚ブラケット（柱の前後面 × ベース主桁の上面）
        for post_x, pnm in ((P.RAIL_X0, post_rear), (P.FRONT_POST_X, post_front)):
            for sx in (1, -1):
                bnm = f"brk_{pnm}_{'p' if sx > 0 else 'm'}"
                loc = (Pos(post_x + sx * P.EXT_W / 2, y, P.BASE_Z1)
                       * Rot(90, 0, 0) * Rot(0, 0, L.BRACKET_ROT[(sx, 1)]))
                put(parts, loc * L.bracket(), bnm,
                    to=(pnm, base_rail), how="TSLOT",
                    note="1-M5（腕 1 枚に 1 本）")
                LEDGER.add("柱脚ブラケット HBLFSN5", 0.012, "MISUMI カタログ", "シャシー")
                bracket_screws(parts, loc, bnm, (pnm, base_rail))
        # 上桁（グラバーレールの取付ビーム）。**柱で切り分ける**
        for k, (xc, ln) in enumerate(L.ext_seg(-430.0, P.FRONT_POST_X - P.EXT_W / 2,
                                               (P.RAIL_X0,))):
            beam = f"topbeam{k}_{sd}"
            # 区間 0 は後柱の -X 側（片持ち）、区間 1 は後柱〜前柱
            ends = (post_rear,) if k == 0 else (post_rear, post_front)
            put(parts, Pos(xc, y, beam_z) * L.ext2020(ln), beam, to=ends,
                how="BRACKET", note="各端 2 個・各 2-M5")
            L.add_ext(f"側面上桁 {beam}", ln, "シャシー")
            for sx, xend in ((-1, xc - ln / 2), (1, xc + ln / 2)):
                if abs(xend + 430.0) < 1.0:
                    continue                      # 後端は突き当たる相手が無い（片持ち）
                tgt = post_rear if abs(xend - P.RAIL_X0) < P.EXT_W else post_front
                # ⚠ 相手の柱が**その高さに存在するか**を確かめる。
                #   前柱の上端は上桁の上面と同じ 838 なので、桁の上に置いた
                #   ブラケットは柱と線でしか当たらない（座面 0mm²）。
                zs = (1, -1) if F.face(tgt, "z", 1) > beam_z + P.EXT_W else (-1,)
                for sz in zs:
                    bnm = (f"brk_{beam}_{'p' if sx > 0 else 'm'}"
                           f"{'u' if sz > 0 else 'd'}")
                    loc = (Pos(xend, y, beam_z + sz * P.EXT_W / 2) * Rot(90, 0, 0)
                           * Rot(0, 0, L.BRACKET_ROT[(-sx, sz)]))
                    put(parts, loc * L.bracket(), bnm, to=(beam, tgt),
                        how="TSLOT", note="1-M5（腕 1 枚に 1 本）")
                    LEDGER.add("上桁ブラケット HBLFSN5", 0.012, "MISUMI カタログ", "シャシー")
                    bracket_screws(parts, loc, bnm, (beam, tgt))
        # 斜材（前後方向の剛性 = マストの転倒モーメントを受ける）。
        # 両端は柱・上桁の面から 25mm 引いて止め、ガセット板で留める。
        # 以前は軸線どうしを交点まで伸ばしていたので、柱に 1758mm³、
        # 上桁に 1913mm³ 食い込んでいた。斜材は角度が付くぶん、
        # 「軸線が届く位置」と「実体が入れる位置」が最もずれる部材。
        ax0 = (P.RAIL_X0 + P.EXT_W / 2, P.BASE_Z1 + P.EXT_W / 2)
        ax1 = (100.0, beam_z - P.EXT_W)
        run, rise = ax1[0] - ax0[0], ax1[1] - ax0[1]
        d = hypot(run, rise) - 2 * P.BRACE_SETBACK
        ang = degrees(atan2(rise, run))
        # 斜材は軸力しか受けないので HFS5-2010 で足りる（2020 比 −354g/2本）。
        # 座屈: L=707・I_min≈0.16e4 → Pcr = π²EI/L² ≈ 2.2kN。実荷重は 200N 程度
        brace = f"brace_{sd}"
        put(parts, Pos((ax0[0] + ax1[0]) / 2, y, (ax0[1] + ax1[1]) / 2)
            * Rot(0, -ang, 0) * L.ext2010(d),
            brace, to=(f"gus_{brace}_lo", f"gus_{brace}_hi"), how="BOLT", note="各端 2-M5")
        L.add_ext2010("側面斜材", d, "シャシー")
        # ⚠ **溝の芯は bbox から出せないので、置いた側が教える。** 斜めに
        #   寝た押出材は bbox が断面より太く、中心線は溝の中心線ではない。
        #   ここに載せておくと screw_check の D 項が芯の上かを検査する。
        F.SLOT_AXIS[brace] = (ax0[0], y, ax0[1],
                              run / hypot(run, rise), 0.0, rise / hypot(run, rise))
        # ガセット（斜材の両端 → 柱／上桁）
        # ⚠ ガセットは相手の**幅ぶん重ねる**こと。柱の面（X=-265.4）から
        #   始めると線でしか当たらない。柱の反対面（-285.4）から張る。
        # ⚠ ねじの位置は**相手の溝の芯**に載せる。柱は縦なので溝は Z 方向に
        #   走り、芯は柱の中心 X（=RAIL_X0）。上桁は横なので溝は X 方向で、
        #   芯は梁の中心 Z（=beam_z）。芯を外すと、ねじ φ5 は溝口 6mm から
        #   はみ出して材を削る（幅方向へ 5mm ずらして 24,084mm³ 削った実績）。
        # ⚠ **斜材へのボルトも同じ**。斜材の溝は 20mm 面（±Y）の中心を軸線に
        #   沿って 1 条走る。芯は軸線そのもの。傾いているので自動配置では
        #   出せない（`_brace_bolt_holes` の注意書き）。
        ux, uz = run / hypot(run, rise), rise / hypot(run, rise)
        for tag, (gx, gz, ga, gb, tgt, holes) in (
                ("lo", (ax0[0] - P.EXT_W, ax0[1], 110.0, 120.0, post_rear,
                        ((P.EXT_W / 2, 30.0), (P.EXT_W / 2, 60.0)))),
                # 上端は**上桁の下面**を基準にする。斜材の軸端(808)に置くと
                # 梁（818..838）まで 10mm 届かず、宣言と実体が食い違う
                ("hi", (ax1[0], beam_z + P.EXT_W / 2, -110.0, -140.0,
                        f"topbeam1_{sd}",
                        ((-10.0, -P.EXT_W / 2), (-40.0, -P.EXT_W / 2))))):
            # ⚠ 柱の**内側**に置くとスライドレールの帯（|Y| 320.9..340）に
            #   4,467mm³ 食い込む。レールは動くので触れさせられない。外側へ出す。
            # ⚠ ガセットは**面内（引張・圧縮）でしか効かない**。斜材の軸力
            #   200N（上のコメントの座屈計算）を 110mm の三角で渡すだけなので、
            #   t4 は過大（`plate_audit.py` の板厚検査で安全率 233）。
            #   板厚を決めているのは応力ではなく座屈で、三角の腕 110mm に
            #   対して t ≥ 110/60 = 1.8mm あればよい。切削の実用下限 t3 に落とす。
            #   4 枚で 70g 減。
            # ⚠ 板厚を変えたら**取付位置も変える**こと。ここは「柱の面から
            #   板厚の半分だけ外」に置いている。+2.0 のまま t3 にすると
            #   柱の面との間に 0.5mm のすきまが空いて「離れ」になる。
            gus_t = 3.0
            gy = y + sy * (P.EXT_W / 2 + gus_t / 2)
            # 斜材へのボルト。端は「軸端から setback だけ内側」＝斜材の実端で、
            # そこから三角形の中へ（＝直角の頂点から斜辺へ）向かって並べる。
            bolts = _brace_bolt_holes(
                (gx, gz), (ga, gb),
                (ux, uz) if tag == "lo" else (-ux, -uz),
                (ax0[0] + P.BRACE_SETBACK * ux, ax0[1] + P.BRACE_SETBACK * uz)
                if tag == "lo" else
                (ax1[0] - P.BRACE_SETBACK * ux, ax1[1] - P.BRACE_SETBACK * uz))
            # 外形はトポロジー最適化した輪郭。⚠ 輪郭の原点は**元の直角三角形の
            #   bbox 中心**（＝直角の頂点から (ga/2, gb/2)）で、板厚は Z・面内は
            #   XY。ここは直角の頂点が原点・板厚 Y・面内 XZ なので、
            #   Rot(90,0,0) で XY → XZ に倒してから頂点へ戻す。掛け忘れると
            #   板が丸ごとずれる（pitch_side で一度やった）。
            shape = (Pos(ga / 2, 0, gb / 2) * Rot(90, 0, 0)
                     * L.topo_plate(f"gus_{brace}_{tag}", gus_t))
            # ねじのバカ穴（φ5.5）。開けないと軸が板の中に丸ごと入る
            for hx, hz in (*holes, *bolts):
                shape -= (Pos(hx, 0, hz) * Rot(90, 0, 0)
                          * Cylinder(L.BRACKET_HOLE_DIA / 2, gus_t + 2, align=CTR))
            g = put(parts, Pos(gx, gy, gz) * L.mat(shape, "A5052"),
                    f"gus_{brace}_{tag}", to=tgt, how="TSLOT",
                    note="2-M5 + 後入れナット")
            LEDGER.add_solid("斜材ガセット A5052 t3", g, "A5052", "シャシー")
            for i, ((hx, hz), mate) in enumerate(
                    [(h, tgt) for h in holes] + [(h, brace) for h in bolts]):
                # 板の**外面**（|Y| が大きいほう）に頭が座り、工具は外から入る。
                # 外側にはスカート（Z 25..135）しか無く、ガセットは Z 145 以上
                # なので 45mm の工具空間は空いている。
                head = (gx + hx, gy + sy * gus_t / 2, gz + hz)
                put_screw(parts, Pos(*head) * Rot(*_axis_rot((0.0, sy, 0.0)))
                          * L.screw_tnut(5, gus_t),
                          f"scr_gus_{brace}_{tag}_{i}",
                          to=(f"gus_{brace}_{tag}", mate),
                          how=("THRU", "TNUT"), kind="CAP", size=5,
                          length=L.screw_len(gus_t, 6.0), extras=(("TNUT", 5),),
                          note="1-M5 + 後入れナット",
                          tool=(*head, 0.0, float(sy), 0.0))
    return parts


def _decks():
    """電装デッキ（肉抜きあり）と砲塔台座（横梁2本＋リングプレート）。"""
    # ⚠ 肉抜きより先に**材料を見直す**。ESC とバッテリーを載せるだけの板に
    #   アルミ t1.5（肉抜き後 472g）を使う理由が無い。テクセル t5 で 198g。
    parts = []
    # ⚠ +X 端は椅子マウント脚（X 85..135）の手前で終えること。
    #   X=110 まで伸ばすと脚と 3,404mm³ 重なる。
    deck = put(parts, Pos(-135.0, 0, P.BASE_Z1 + P.DECK_T / 2) * L.mat(
        Box(430.0, 430.0, P.DECK_T, align=CTR), "TEKCELL"),
        # ⚠ デッキ板は X -350..80。X=-410 の横材までは 50mm 届かない。
        #   届かない相手を固定先に書くと「離れ」になる。
        "deck_plate", to=("cross_xm110_mid", "cross_xm210_mid"), how="RIVET",
        note="テクセルはねじが効かないので座金付きリベット")
    LEDGER.add_solid("電装デッキ テクセル t5 430x430", deck, "TEKCELL", "シャシー")
    beam_z = P.BEAM_TOP_Z - P.EXT_W / 2
    ln = 2 * P.SIDE_FRAME_Y - P.EXT_W
    for k, x in enumerate(P.TURRET_X + bx for bx in P.PEDESTAL_BEAM_X):
        nm = f"pedestal_beam{k}"
        ends = ("topbeam1_R", "topbeam1_L")
        put(parts, Pos(x, 0, beam_z) * Rot(0, 0, 90) * L.ext2020(ln), nm,
            to=ends, how="BRACKET")
        L.add_ext("砲塔台座横梁", ln, "砲塔")
        # ⚠ ブラケットは**上桁が存在する側**へ。前側の横梁（X=360）の +X 面は
        #   上桁の前端（370）と一致するので、外側に置くと線でしか当たらない。
        arm = (-1,) if x + P.EXT_W / 2 >= F.face(ends[0], "x", 1) - 1.0 else (1,)
        parts += _butt_brackets(x, 0.0, ln, beam_z, "砲塔", nm, ends, arm)
    # 台座リングプレート。**横梁の上に載せる**（以前は梁の中に埋めていた）。
    # 肉抜き穴は φ56 → φ32 に縮小。φ56 は内径 190 と外径 300 のあいだ 55mm を
    # 完全に食い切っており、リングが**8 個のバラバラの円弧**に切断されていた
    # （ソリッド数が deck_3#0..#7 と 8 個に増えていたのが手がかり）。
    ring = (Cylinder(P.PEDESTAL_RING_R, P.PEDESTAL_PLATE_T, align=CTR)
            - Cylinder(P.PEDESTAL_BORE_R, 6.0, align=CTR))
    # ⚠ 肉抜き穴は**ボルト位置を避ける**こと。PCD323 の 8 等配は
    #   締結位置（±110, ±116 → 角度 ±46.5°）と重なり、4 本が穴の中を
    #   締めていた（相手に届かず「離れ」として出た）。0/90/180/270 に置く。
    for bx, by in bolt_circle(4, 2 * (P.PEDESTAL_BORE_R + P.PEDESTAL_RING_R) / 2):
        ring -= Pos(bx, by, 0) * Cylinder(14.0, 6.0, align=CTR)
    # ⚠ 締結位置は**横梁の真上**でなければならない。PCD244 の等配 8 本のうち
    #   4 本は X=±122 で、横梁（X=125±10 と 345±10 → 台座中心から ±110±10）を
    #   外れて空中を締めていた。梁の X に合わせて 2 本 × 2 列に置き直す。
    # ⚠ 給送の通り道を開けること。昇降コンベアは旋回体側にあり、
    #   斜路(805)からニップ(1000)へ上がる途中で**この固定プレートの高さ(838..842)を
    #   横切る**。600mm 幅の布はリングの内径(Ø276)を通れないので、
    #   -X 側に開口を設けて外周側から通す。
    #   開口は台座横梁(X=±110±10)より外なので、リングの支持は減らない。
    # ⚠ 600mm 幅の布はリング内径（Ø276）を通れないので、**外周側を通す**しかない。
    #   通り道（|X| < 115、|Y| <= 285）にプレートを残すと必ず貫かれる。
    #   横梁の真上だけを残した 2 枚の円弧にする。ベアリングは横梁が受ける。
    keep = None
    for x0, x1 in ((-185.0, -140.0), (115.0, 185.0)):
        b_ = Pos((x0 + x1) / 2, 0, 0) * Box(x1 - x0, 2 * P.PEDESTAL_RING_R + 10,
                                            P.PEDESTAL_PLATE_T + 2, align=CTR)
        keep = b_ if keep is None else keep + b_
    ring = ring & keep
    ring = Pos(P.TURRET_X, 0, P.BEAM_TOP_Z + P.PEDESTAL_PLATE_T / 2) * ring
    if len(ring.solids()) != 2:
        raise ValueError(f"台座リングプレートは横梁上の 2 枚のはずが "
                         f"{len(ring.solids())} 個になっている")
    # ⚠ **2 枚の円弧は別部品として置く（2026-08-05）。** 1 つの名前で
    #   2 ソリッドを put すると、製作データが「平板 1 枚の輪郭で表せない」と
    #   して落ちる（実際そうなっていた）。実物も横梁ごとに 1 枚ずつ切る
    #   2 部品なので、名前を分けるのが図とも合う。
    # ⚠ **円弧 1 枚が留まるのは、その真下の梁 1 本だけ。** 両方の梁を
    #   固定先に書いたら `topo_opt` が
    #   「荷重が全部 fixed 節点に載っている（自由 dof に力が無い）」で落ちた。
    #   実測: beam0 は X 75..95、beam1 は X 350..370。円弧は X 50..95 と
    #   350..420 なので、**X 昇順で 1 対 1 に対応する**（`scr_J5` の
    #   `bx < 0 → beam0` と同じ規則）。
    rs = sorted(ring.solids(), key=lambda s_: s_.center().X)
    for i_, sol_ in enumerate(rs):
        put(parts, L.mat(sol_, "A5052"), f"pedestal_ring{i_}",
            to=f"pedestal_beam{i_}", how="BOLT", note="4-M5")
        LEDGER.add_solid("砲塔台座リングプレート A5052 t4（肉抜き）", sol_,
                         "A5052", "砲塔")
    return parts


def _skirt():
    """全周スカート（巻き込み対策 7.2.6）。"""
    h = P.SKIRT_Z1 - P.SKIRT_Z0
    zc = (P.SKIRT_Z0 + P.SKIRT_Z1) / 2
    parts = []
    for sx in (1, -1):
        # 前後のスカートは端の横材（外側ステーション）に留める
        tgt = tuple(f"cross_xp410_{s}" if sx > 0 else f"cross_xm410_{s}"
                    for s in ("L", "mid", "R"))
        # LiDAR ブラケットが骨格の端面まで届くよう、窓を開ける
        # ⚠ 2026-08-06 に幅 100 → 160。立板を「LiDAR をまたぐ 2 本脚」に
        #   したので、脚は |Y| 44..72 まで外へ出る（`P.LIDAR_BRK_Y`）。
        #   100 のままだと脚がスカートを突き抜ける。
        # ⚠ 高さは**座の実際の上下端**から取る。UST-20LX の検出面は底面から
        #   47.4mm（中央ではない）ので、走査面を中心に決め打ちすると外れる。
        #   吊り下げなので、窓を通るのは可動板（と胴の -X 端）だけ。
        win_z0 = P.LIDAR_LOW_Z + P.LIDAR_PLANE_Z - 8.0
        win_z1 = P.BASE_Z1 + 2.0
        panel = (Box(P.SKIRT_T, P.BASE_Y, h, align=CTR)
                 - Pos(0, 0, (win_z0 + win_z1) / 2 - zc)
                 * Box(P.SKIRT_T + 2,
                       P.LIDAR_LVL_PITCH_Y + 2 * P.LIDAR_LVL_EDGE + 16.0,
                       win_z1 - win_z0, align=CTR))
        put(parts, L.mat(Pos(sx * (P.BASE_X / 2 + P.SKIRT_T / 2), 0, zc) * panel, "PC"),
            f"skirt_{'front' if sx > 0 else 'rear'}", to=tgt, how="RIVET",
            note="各横材に 2-φ3.2")
        for sd in ("L", "R"):
            F.touch_ok(f"skirt_{'front' if sx > 0 else 'rear'}", f"skirt_{sd}",
                       "全周スカートの角で突き合う")
        for r in ("rail_L_out", "rail_R_out", "rail_L_in", "rail_R_in"):
            F.touch_ok(f"skirt_{'front' if sx > 0 else 'rear'}", r, "骨格の端面を覆う")
        F.touch_ok(f"skirt_{'front' if sx > 0 else 'rear'}",
                   f"lidar_lvl_top_{'front' if sx > 0 else 'rear'}",
                   "LiDAR の窓を通る（可動板の -X 端）")
    for sy in (1, -1):
        put(parts, L.mat(Pos(0, sy * (P.BASE_Y / 2 + P.SKIRT_T / 2), zc)
                         * Box(P.BASE_X, P.SKIRT_T, h, align=CTR), "PC"),
            f"skirt_{'L' if sy > 0 else 'R'}",
            to=f"rail_{'L' if sy > 0 else 'R'}_out", how="RIVET",
            note="8-φ3.2 を長手材に沿って")
    total = sum(p.volume for p in parts)
    LEDGER.add(f"スカート ポリカーボネート t{P.SKIRT_T} 全周", total * P.DENSITY["PC"] / 1000,
               "PC 体積計算", "安全")
    return parts


def _drive_mounts():
    """M3508 マウント（切削板 t8 + 上部耳）とモーター・ハブアダプタ。"""
    parts = []
    for sx in (1, -1):
        for sy in (1, -1):
            x = sx * P.WHEELBASE_X / 2
            sd = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            rail_in = f"rail_{'L' if sy > 0 else 'R'}_in"
            ypl = sy * (P.MOTOR_PLATE_Y + P.MOTOR_PLATE_T / 2)
            # ⚠ **2026-08-04: 曲げをやめた。** 前は「縦板と上部の耳を 1 枚から
            #   曲げる」形にしていたが、自校ではアルミ板を曲げられない
            #   （`export_fab.CAN_BEND = False`）。
            #   耳の役目は「縦板を桁の溝ナットへ落とし込む」ことなので、
            #   **MISUMI の L 金具（HBLFSN5）2 個**に置き換える。金具は
            #   購入品で、この機体で既に 46 個使っている（`brk_*`）。
            #   ⚠ 耳を単に別部品にするだけでは駄目。縦板（z 27.5..147.5）が
            #     耳（z 135..139）を 1,920mm³ 貫く。金具なら縦板の**面**に
            #     留まるので、貫きも分断も起きない。
            # ⚠ 肉抜きを**後から**掛けると、motor_mount_plate が開けた
            #   軸穴・ボルト穴の位置に材料が戻ることはないが、
            #   肉抜き穴のグリッドが軸穴と重なって形が崩れる。
            #   肉抜き → 軸穴 の順に処理する。
            web = (Pos(x, ypl, (P.AXLE_Z + P.BASE_Z0) / 2 + 5.0) * Rot(90, 0, 0)
                   * L.motor_mount_plate(130.0, 120.0, 4.0, path=True,
                                         axle_z=P.AXLE_Z - ((P.AXLE_Z + P.BASE_Z0) / 2 + 5.0)))
            put(parts, L.mat(web, "A5052"), f"mount_brk_{sd}",
                to=f"mount_ear_{sd}", how="BOLT",
                note="2-M5（耳板の端面にタップ）")
            # 縦板 ↔ 桁上面 の L 金具。縦板の板厚は Y なので、金具の腕は
            # 「Y 面（板）」と「Z 面（桁の上面）」に当てる。X 方向へ 2 個離す。
            # ⚠ **L 金具では届かない。** 縦板の外面は |Y|=260、桁の内面は
            #   |Y|=275 で 15mm 空いている。HBLFSN5 の腕は 17mm なので
            #   桁に 2mm しか掛からない。ここは元の「耳」の張り出し
            #   （258→292）が要る寸法だった。
            # → 耳は残すが**別の平板**にし、**t8** にして縦板から耳の
            #   **端面へ M5 をねじ込む**（端面タップ ＝ 横穴加工でできる）。
            #   t4 の耳では M5 のタップが立たない（`L.TAP_MIN[5] = 7.5`）。
            # ⚠ **耳の内端は縦板の外面（|Y| = MOTOR_PLATE_Y + T）に合わせる。**
            #   前は縦板の**中心**から 17mm で置いていたので、耳が縦板へ
            #   板厚の半分（2mm）ぶん食い込んでいた（120×2×8 = 1,920mm³）。
            #   端面にタップを立てて横から留める以上、突き合わせが正しい。
            #   外端は 260+34 = 294 で、桁の内面 |Y|=275 は変わらず跨ぐ。
            #   ⚠ `P.MOTOR_PLATE_T`(=5) は**溝の呼び厚**で、縦板の実厚は
            #     `motor_mount_plate(..., 4.0)` の 4mm。呼び厚で置くと 1mm
            #     浮いて「宣言した固定先と接していない」になる。実厚で置く。
            ear = (Pos(x, ypl + sy * (2.0 + 17.0), P.BASE_Z1 + 4.0)
                   * L.plate(120.0, 34.0, 8.0))
            # ⚠ 耳板の固定先は**桁だけ**。縦板との締結は縦板の側で
            #   宣言済み（`mount_brk_* → mount_ear_*`）なので、ここにも
            #   書くと相互宣言＝固定の循環になる。
            put(parts, L.mat(ear, "A5052"), f"mount_ear_{sd}",
                to=rail_in, how="TSLOT", note="2-M5 溝ナット")
            LEDGER.add_solid("モーターマウント 耳板 A5052 t8", ear, "A5052", "駆動")
            # モーター（取付面 |Y|=255、軸は外向き）
            # ⚠ m3508 は取付面 z=0 に対し**本体が -Z 側（75.4）・軸が +Z 側（25.8）**。
            #   Rot(-90*sy) で本体が内側へ出るので、取付面は縦板の**内面**に置く。
            #   外面に置くと本体の最後の 6mm が板の中に入る（5,542mm³）。
            #   軸は板を貫くが、それは逃げ穴を開ける前提（motor_mount_plate が持つ）。
            put(parts, Pos(x, sy * P.MOTOR_PLATE_Y, P.AXLE_Z)
                * (Rot(-90 * sy, 0, 0) * L.m3508()),
                f"motor_{sd}", to=f"mount_brk_{sd}", how="BOLT", note="4-M4 PCD35")
    LEDGER.add("M3508 P19", P.M3508_MASS, "DJI カタログ", "駆動", qty=4)
    LEDGER.add_solid("モーターマウント 曲げ板 A5052 t4（肉抜き）", parts[0], "A5052", "駆動", qty=4)
    LEDGER.add("ハブアダプタ A5052", 0.049, "体積概算", "駆動", qty=4)
    # 駆動配線のサドル。**内桁の外面（|Y|=295）から配線（|Y|=318）までの
    # 23mm には何も無かった**。「内桁の上面に結束」と宣言していたが、
    # 骨格と椅子脚を避けた結果、実体は桁から離れたところを走っている。
    # 距離を埋める金具が無ければ、実機ではケーブルが垂れて車輪に巻き込む。
    # ⚠ X は椅子マウント脚（|Y| 265..305、X 125..175 / 190..230）と
    #   マウント曲げ板（|Y| 256..292、X 235..365）の外を選ぶ。
    for sx in (1, -1):
        for sy in (1, -1):
            sd = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            rail = f"rail_{'L' if sy > 0 else 'R'}_in"
            # ⚠ 取付面は内桁の**外側面**。立ち上がりの下端を桁の上面に置くと、
            #   外側面とは角（線）でしか当たらず座面が 0mm² になる。
            #   下端を桁の**下面**まで下ろして、外側面と 20mm 重ねる。
            y_face = F.face(rail, "y", sy)
            z_bot = F.face(rail, "z", -1)
            # 腕の上面をケーブルの下端に合わせる（載せる高さは配線が決める）
            rise = (P.BASE_Z1 + 5.0 - 4.0) - z_bot - 3.0
            # ⚠ サドルは配線の**水平区間**（X -60..225 / -95..-225）に置く。
            #   端で斜めに下りる区間に置くと、載せる高さが合わず届かない。
            #   椅子マウント脚（X 125..175 / 190..230、|Y| 265..305）も避ける。
            for k, xs in enumerate((-20.0, 110.0) if sx > 0 else (-160.0, -215.0)):
                put(parts, Pos(xs, y_face, z_bot)
                    * Rot(0, 0, 0 if sy > 0 else 180) * L.cable_saddle(rise=rise),
                    f"cab_saddle_{sd}{k}", to=rail,
                    how="BOLT", note="内桁の外面へ 2-M4")
                LEDGER.add("配線サドル A5052 t3", 0.006, "体積概算", "電装")
    return parts


def _fasteners():
    """主要締結部のボルトを実体として置く（scripts/fasteners.py の J1〜J10 に対応）。

    ねじ 1 本ごとに「**どの 2 部品を留めているか**」を宣言する。
    以前はグループ単位で `("fasteners","base"): BOLTED` と書いていたので、
    56 本 × 167 部品の全組が「接触してよい」ことになり、
    レールの外に 3.5cm 浮いたねじが 4 本あっても検査を通っていた。

    ねじ山は描かない（座面位置と本数が分かればよい）。

    ⚠ 質量は**もう概算に混ぜない**。`put_screw()` が呼び径と長さを控えるので、
      置いた本数から質量が出る（`_fastener_ledger()`）。以前は「ボルト・
      ナット・スペーサ類 700g」の 1 行にまとめていたが、実際に数えたら
      その何倍もあった。概算の中に隠れているあいだは誰も気づけない。
    """
    parts = []
    # J1 M3508 ↔ マウント板: 4-M4 PCD35、外側から内側へ
    for sx in (1, -1):
        for sy in (1, -1):
            sd = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            x = sx * P.WHEELBASE_X / 2
            y = sy * (P.MOTOR_PLATE_Y + 5.0)
            for i, (px, pz) in enumerate(bolt_circle(4, P.M3508_BOLT_PCD, 45.0)):
                put_screw(parts, Pos(x + px, y, P.AXLE_Z + pz) * Rot(-90 * sy, 0, 0) * L.screw(4, 14),
                    # ⚠ ハブへの刺さりを宣言で消してはいけない。ハブは
                    #   **回るリンク**なので、車体側のねじを留め先にすると
                    #   「関節が無いのに別リンクへ留めている」になる。
                    #   逃げ穴を φ6（M4 軸用）で開けていたが、そこへ来るのは
                    #   軸ではなく**頭（φ7）**。ハブを頭の外へ逃がすのが正解。
                    # ⚠ M3508 のフランジは t4.3。M4 のタップには 6mm 要るので
                    #   タップは切れない。モーター側の**めねじ**（データシート
                    #   PCD35 の M4 ねじ穴、深さ 8）に入れる。マウント板は
                    #   貫通穴なので BOLT。
                    f"scr_J1_{sd}_{i}",
                    to=(f"mount_brk_{sd}", f"motor_{sd}"),
                    how=("THRU", "SCREW_IN"),
                    kind="CAP", size=4, length=14.0,
                    note="4-M4 PCD35（モーター側めねじ 深8）",
                    tool=(x + px, y, P.AXLE_Z + pz, 0.0, sy, 0.0))
    # J2 マウント耳 ↔ ベース内桁: 2-M5、上から
    for sx in (1, -1):
        for sy in (1, -1):
            sd = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            x = sx * P.WHEELBASE_X / 2
            for i, dx in enumerate((-40.0, 40.0)):
                put_screw(parts, Pos(x + dx, sy * P.FRAME_RAIL_Y_IN, P.BASE_Z1 + 4.0)
                    * L.screw(5, 16, washer=True),
                    # ⚠ 桁へ留まるのは**耳板**（`mount_ear_*`）。縦板は
                    #   耳板の端面タップへ留まる（2026-08-05 に曲げをやめた）。
                    #   `mount_brk` のままだと「宣言のどの組にも紐づかない
                    #   ねじ 8 本」になる（`fastener_bom` が実際に落とした）。
                    f"scr_J2_{sd}_{i}",
                    to=(f"mount_ear_{sd}", f"rail_{'L' if sy > 0 else 'R'}_in"),
                    how="SCREW_IN",
                    kind="CAP", size=5, length=16.0, extras=(("WASHER", 5),),
                    note="2-M5 + 平座金",
                    tool=(x + dx, sy * P.FRAME_RAIL_Y_IN, P.BASE_Z1 + 4.0,
                          0.0, 0.0, 1.0))
    # J5 砲塔リング ↔ 台座横梁: 8-M5、上から
    for i, (bx, by) in enumerate(P.PEDESTAL_BOLTS):
        beam = "pedestal_beam0" if bx < 0 else "pedestal_beam1"
        # ⚠ +X 側の 4 本は歯付きリング（外径 r178、Z 842.5..861.5）の
        #   真下に来る。台座プレートの上に頭を出すと、頭外縁 r183 のうち
        #   内側 4mm がリングの下へ潜って 311mm³ 削る。
        #   r 178..185 の 7mm 帯に M5 の頭（φ8.5）は 2 本並べられないので、
        #   **皿ねじにして頭をプレートに沈める**（Z で分離する）。
        deep = bx > 0
        put_screw(parts, Pos(P.TURRET_X + bx, by,
                             # 0.5mm ちょうどでは接触判定に入るので 1.5mm 沈める
                             P.PEDESTAL_TOP_Z - (6.0 if deep else 0.0))
            * L.screw(5, 18, washer=not deep),
            # ⚠ 台座リングプレートは t4。M5 のタップには 7.5mm 要る。
            #   プレートは貫通穴（BOLT）で、ねじが刺さるのは**梁の溝ナット**。
            # ⚠ リングは横梁ごとに 1 枚（`pedestal_ring0/1`）。梁と同じ
            #   規則で X から選ぶ。`ring0` が -X 側（`_pedestal_group` で
            #   X 昇順に並べている）。
            f"scr_J5_{i}",
            to=(f"pedestal_ring{0 if bx < 0 else 1}", beam),
            how=("THRU", "TNUT"),
            # -X 側は座金付きキャップ、+X 側は歯付きリングの下へ潜るので皿ねじ
            kind="FLAT" if deep else "CAP", size=5, length=18.0,
            extras=(("TNUT", 5),) if deep else (("TNUT", 5), ("WASHER", 5)),
            note="8-M5 溝ナット",
            tool=(P.TURRET_X + bx, by,
                  P.PEDESTAL_TOP_Z - (6.0 if deep else 0.0), 0.0, 0.0, 1.0))
    # J6 スライドレール ↔ 側面上桁: 各レール 6-M4（内側から外へ）
    # ⚠ 座標は **RAIL_X0 からの相対** で書いていたのに、
    #   アウターレールの実長（390mm・X -275.4..114.6）を確かめていなかった。
    #   6 本のうち 2 本が X=-460 / -360 とレールの外に出て、空中に浮いていた。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for i, x in enumerate((-250.0, -170.0, -90.0, -10.0, 50.0, 105.0)):
            beam = f"topbeam{0 if x < P.RAIL_X0 else 1}_{sd}"
            # ⚠ 座面は**アウターレールの内面**。可動段（インナー 317..319、
            #   中間 325..328）より外から入れないと、ねじが動く段を貫く。
            # ⚠ 座面はアウターレールの内面（RAIL_OUT_INNER_Y）。
            #   330 と手で書いていたので頭が中間レールの走行路へ出ていた。
            # ⚠ **呼び長さ 26 は買えない**（流通するのは 25 と 30）。手で
            #   書いた長さは `L.screw_len()` の切り上げを通らないので、
            #   員数表に 12 本ぶんの M4×26 が載ったままだった。
            #   掴み代はアウターレール t1.75 + 取付プレート t4 = 5.75mm。
            #   ナット 3.2 + 座金 0.8 を足して 9.75 → **12** で足りる。
            #   26 だと Y 360.25 まで届き、斜材ガセット（Y 360..363、
            #   x=-10 と 50 の位置）へ 0.25mm 刺さっていた。
            put_screw(parts, Pos(x, sy * P.RAIL_OUT_INNER_Y, P.RAIL_Z)
                * Rot(90 * sy, 0, 0) * L.screw(4, 12),
                # ⚠ アウターレールは t1.8。M4 のタップには 6mm 要るので
                #   レール側にはタップを切れない（公式 CAD の取付穴は
                #   貫通穴）。ねじが刺さるのは t4 の取付プレート側で、
                #   そこも 6mm 足りないので**裏からナット**で留める。
                f"scr_J6_{sd}_{i}", to=(f"rail_out_{sd}", f"rail_plate_{sd}"),
                how="THRU",
                kind="CAP", size=4, length=12.0,
                extras=(("HEXNUT", 4), ("WASHER", 4)),
                note="6-M4 + ナット",
                tool=(x, sy * P.RAIL_OUT_INNER_Y, P.RAIL_Z, 0.0, -sy, 0.0))
    # J8 マスト頭部横梁 ↔ 主柱: 4-M5（上から）
    # ⚠ 呼び長さは 20 ではなく 12。**20 だと横梁の断面 20mm をちょうど貫いて、
    #   先端が下面（z=1152）に届く。** そこには内隅の L 金具（brk_mast_corner_*）
    #   の腕が座っているので、ねじの先が金具の座面を突いて浮かせる。
    #   溝ナットに噛むのに要るのは板厚 6 + かみ合い 6 = 12mm で足りる
    #   （`L.screw_len` の考え方と同じ）。長いねじは「余っても害は無い」
    #   ように見えるが、**反対側の面に何か付いた瞬間に害になる**。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for i, dx in enumerate((-6.0, 6.0)):
            put_screw(parts, Pos(P.MAST_X + dx, sy * (P.MAST_Y - P.EXT_W / 2 - 6.0),
                                 P.MAST_BEAM_Z + P.EXT_W / 2) * L.screw(5, 12, washer=True),
                f"scr_J8_{sd}_{i}",
                to=("mast_cross", f"brk_mast_cross_{sd}{'p' if dx > 0 else 'm'}"),
                how=("TNUT", "THRU"),
                kind="CAP", size=5, length=12.0,
                extras=(("TNUT", 5), ("WASHER", 5)), note="2-M5 溝ナット",
                tool=(P.MAST_X + dx, sy * (P.MAST_Y - P.EXT_W / 2 - 6.0),
                      P.MAST_BEAM_Z + P.EXT_W / 2, 0.0, 0.0, 1.0))
    # ⚠ 柱脚を主桁へ**直接**留めるねじ（scr_post_*、柱の底から下向きに 8 本）は
    #   やめた。理由は 2 つ。
    #     ・柱脚ブラケットにねじを実体で入れたので、同じ継手が
    #       「金具 2 個 × 各 2-M5」で成立している。直付けは二重。
    #     ・頭が**中空の柱の中**に来るので、六角レンチが入らない。
    #       工具アクセスの検査を通っていたのは、塞いでいる相手（柱）が
    #       締結先そのもので、検査の除外リストに入っていたため。
    #   さらに金具の後入れナット（φ11）と 0.5mm まで近づいていた。
    return parts


def _cables():
    """主要ハーネスの取り回し。可動部（砲塔・グラバー）との干渉を検査できるようにする。

    * 電源幹線: バッテリー → ブレーカー → ESC群（デッキ上 z=150）
    * 駆動4輪: デッキ → 内桁沿い → 各モーター後端
    * 砲塔系: デッキ → 側面トラス後柱 → 台座 → 旋回体（±30°のねじれ許容ループ）
    * グラバー系: 側面上桁 → キャリッジ（ストローク425mmぶんのUループ）
    * センサー: 前後LiDAR・上段LiDAR → デッキ
    """
    z_deck = P.BASE_Z1 + 15.0
    parts = []
    # 電源幹線
    # ⚠ 高さ z_deck(150) はバッテリー・ブレーカー・ESC が載っている帯そのもの。
    #   機器の中を通していた（バッテリー 5,938 / ブレーカー 5,593 / ESC 6 組）。
    #   機器の**上**（z=200）を通し、端点だけ各機器の脇へ降ろす。
    z_up = P.BASE_Z1 + 65.0
    # ⚠ 配線は**両端に行き先がある**。deck_plate に沿わせるだけの宣言では、
    #   もう一方の端が空中で終わっていることに気づけない。
    #   幹線はバッテリー → ブレーカー → ESC 群をつなぐので、
    #   始点をバッテリーの上面、終点を C620 の上面に着地させる。
    #   ⚠ 折り返し（172°）を作ると曲げ半径が足りない。ESC 群（X -330）へは
    #     戻らずに、ブレーカーから素直に伸ばす経路にする。
    #   ⚠ 着地点は相手の **Y 範囲の中**に置く。cable_to は Z だけを面に
    #     合わせるので、Y がずれていると「上面に降ろした」つもりで
    #     機器の脇を通り過ぎる（ブレーカーは |Y|<=20）。
    put(parts, L.cable_to([(-150, 0, z_up), (-280, 0, z_up), (-350, 0, z_up),
                           (-340, 110, z_up), (-330, 150, z_up)],
                          [(0, ("battery", "z", 1)),
                           (2, ("breaker", "z", 1)),
                           (-1, ("esc620_0", "z", 1))], 12.0),
        "cab_power", to=("battery", "breaker", "esc620_0"),
        how=("CONNECT", "CONNECT", "CONNECT"),
        note="配線サドルで机上 65mm を通す")
    # 駆動4輪（デッキ → 内桁沿い → モーター後端 |Y|=180）
    # ⚠ 高さ z_deck(150) を通すと**椅子の座面の中**を走る（座面下面 150）。
    #   実際に椅子と 3,483mm³ 重なっていた。内桁の内側・骨格の高さ(125)を通す。
    # ⚠ 骨格の高さ帯（115..135）を通すと横材の中を走る。**内桁の内側面に沿わせる**。
    #   前後 2 本が同じ経路を共有して 6,050mm³ 重なっていたので、
    #   前は Y を少し内へ、後ろは外へずらして分ける。
    # ⚠ 骨格の高さ帯（115..135）を横切ると、横材とブラケットを次々に貫く。
    #   **内桁の上面（135）に載せて**走らせ、モーターの手前で下ろす。
    #   終端はモーター本体の面に合わせる（手で書いた終点は必ず埋まるか浮く）。
    z_run = P.BASE_Z1 + 6.0
    for sx in (1, -1):
        for sy in (1, -1):
            sd = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            y_run = sy * (P.FRAME_RAIL_Y_IN + (5.0 if sx > 0 else 12.0))
            x_dn = -60.0 if sx > 0 else -95.0
            # ⚠ 終端はモーターの**後端面**（|Y| が小さい側）。外側へ回すと
            #   車輪（|Y| 275..325）とマウント曲げ板（256..292）を貫く。
            # ⚠ 内桁（|Y| 275..295）の**内側の面**に沿わせ、マウント曲げ板
            #   （|Y| 256..292、X 235..365）の手前で下ろす。
            # ⚠ 高さの帯を選ぶ:
            #     骨格   Z 115..135（横材・ブラケットが |Y|<=275 を塞ぐ）
            #     椅子   Z 150.. （X 130..460 / Y -50..310）
            #   → **Z 139〜147 の 8mm の隙間**を、内桁の真上を通す。
            #     内桁は |Y| 275..295 なので、椅子（Y<=310）とは X で分ける。
            m_face = F.face(f"motor_{sd}", "y", -sy) - sy * 12.0
            # ⚠ モーター本体（X 279..321、|Y| 179.6..280.8）を **X を保ったまま**
            #   横切ってはいけない。外(|Y|=318)から内(179.6)へ寄せる区間は、
            #   モーターの X 範囲の**手前**で済ませる。以前は本体を貫いて
            #   3,766mm³、さらに車輪へ 1,532mm³ 入っていた。
            #   さらにマウント曲げ板（|Y| 256..292、X 235..365）の X 範囲も
            #   外すこと。内へ寄せる区間がここを通ると板を削る。
            x_appr = F.face(f"mount_brk_{sd}", "x", -sx) - sx * 5.0
            # ⚠ 横材（|Y|<=275、Z 115..135）と椅子マウント脚（Z 135..150、
            #   X 125..175 / 190..230）の両方を避ける。内桁の**真上の外側**
            #   |Y|=305 を Z=143 で走り、脚の X 範囲の外で下ろす。
            # ⚠ 塞いでいるものを列挙して残りを取る:
            #     椅子マウント脚  |Y| 265..305  X 125..175 / 190..230
            #     マウント曲げ板  |Y| 256..292  X 235..365
            #     内桁            |Y| 275..295  Z 115..135
            #   → |Y|=318（内桁の外・側面トラスの内）を Z=147 で走る
            # ⚠ マウント曲げ板は X 235..365 / |Y| 256..292 / Z 27.5..147.5。
            #   その **X 範囲の外**（X<=230）で下ろしてから、車軸の高さで
            #   モーターの後端へ寄せる。
            y_run = sy * 318.0
            # 端点は**面に着地**させる。中間点は塞いでいるものを避けた空き帯。
            # ⚠ 高さの層をほかの束と分ける。デッキ上は
            #     電源幹線 Z 215 / 砲塔系 Z 159..171 / 駆動系 Z 151..159
            #   砲塔系と同じ高さを走らせると、デッキ上で必ず交差する。
            # ⚠ 起点の Y も砲塔系（X=-95 で Y 193..205）の**外**へ取る。
            #   内側から出ると、外へ向かう途中で必ず砲塔系を横切る。
            # ⚠ 内側へ寄せるのは**車軸の高さまで下りてから**。デッキ板
            #   (Z 135..140) と椅子 (X 130..460 / Y -50..310 / Z>=150) は
            #   どちらも Z>=135 にあるので、Z 45 まで下りれば下をくぐれる。
            put(parts, L.cable_to([
                (x_dn, sy * 230.0, z_deck + 12.0),
                (x_dn, y_run, P.BASE_Z1 + 5.0),
                (x_appr, y_run, P.BASE_Z1 + 5.0),
                (x_appr, y_run, P.AXLE_Z + 14.0),
                (x_appr, m_face, P.AXLE_Z + 14.0),
                (F.face(f"motor_{sd}", "x", -sx) + sx * 21.0, m_face, P.AXLE_Z + 14.0)],
                [(-2, (f"motor_{sd}", "y", -sy)),
                 (-1, (f"motor_{sd}", "y", -sy))], 8.0),
                f"cab_drive_{sd}",
                to=tuple(f"cab_saddle_{sd}{k}" for k in range(2)) + (f"motor_{sd}",),
                how=("ROUTE", "ROUTE", "CONNECT"),
                note="サドル 2 個で内桁の外面に沿わせ／モーター後端に差込")
    # 砲塔系（後柱に沿って上げ、台座から旋回体へループ）
    # 砲塔系: レール帯(782〜818)の下を通し、ヨー軸の中心穴から旋回体へ入れる。
    # 中心から入れれば ±30° のヨーでケーブルがねじれない（旋回テーブルにφ110の穴を用意済み）
    # ⚠ **固定側の配線は旋回体の中まで伸ばさない。**
    # 以前は終点を (220, 0, 870) すなわち旋回体の内部（X125..345 / Z769..977）に
    # 置いていたので、漏斗に逃げ穴を開けても次はローラー、それを避けると次はプーリ、と
    # 当たる相手が変わり続けた。旋回体の占有領域そのものを通っているのが原因だった。
    # 固定側はヨー軸の中心穴の**下**（台座下面）で終え、そこから先は旋回体側の配線とする。
    # ⚠ 終端の (CABLE_THRU_X, 0, 730) へ Y=0 で寄せると、椅子の上の
    #   マスコット規定外形（X 90..390 / Y 0..300 / Z 162..762）を 33,995mm³ 貫く。
    #   マスコットの**外側（|Y|>=310）を通ってから**ヨー軸へ寄せる。
    put(parts, L.cable_to([
        # ⚠ (0,150)→(-350,330) の直線は |Y|=285 を X≈-262 で横切る。
        #   そこは J2 のボルト（X -260、M5 頭の天面 Z 144）の真上で、
        #   束の下端 143 が頭に 96mm³ かぶる。中間点を足して横断位置を
        #   ボルトから 120mm 以上ずらす。
        (0, 150, z_deck), (-150, 300, z_deck),
        (-350, 330, z_deck), (-350, 330, 770.0),
        # ⚠ スライドレール（X -275..131、Y 334.2..336、Z 779.4..814.6）の
        #   帯にいるあいだは Z 772（上端 778）を保つ。X 200 で 790 まで
        #   上げると、レールの X 範囲の中で帯に入り 84mm³ 削る。
        # ⚠ φ12 の束は |Y|=336 だと 330..342 になり、側面筋交いの内面
        #   （340）に 53mm³ かかる。333 に寄せる（327..339）。
        (100, 333, 772.0), (140, 333, 772.0),
        (P.CABLE_THRU_X, 336, 800.0),
        # ⚠ 終端 |Y|=180 は**昇降コンベアの挟み込みベルトとプーリの帯**
        #   （|Y| 173..187）そのもの。ヨー -30° で旋回体が回ると、
        #   下端プーリを 872mm³・軸を 202mm³ 貫く。
        #   昇降系は |Y|=180（ベルト/プーリ）と |Y|=246（軸受）にあるので、
        #   空いているのは |Y|<165。140 まで寄せる。
        # ⚠⚠ **この終端は 3 回いじって 3 回悪化させた。座標を当てるのを
        #   やめた記録を残す。** ヨー -30° のとき、旋回体の中のこの領域は
        #   昇降コンベアの **L 側だけにある駆動系**（下端プーリ |Y| 173..187 /
        #   スタブ軸 |Y| 40..251 / ガイド板 |Y|<=281）で埋まっている。
        #     終端 |Y|=180 → プーリを 872 + 軸 202 + ベルト 13 = 1,087mm³
        #     終端 |Y|=140 → リングの内縁に乗って 961mm³（4 件）
        #     リングの外で上げてから内へ → プーリに 2,254mm³（7 件・最悪）
        #   空き帯が無いのではなく、**昇降駆動が L 側に片寄っている**ことが
        #   原因（右側には同じものが無いのでヨー +30° では出ない）。
        #   配線を動かすのではなく、昇降駆動の左右振り分けを決め直すのが筋。
        #   ⚠ 「いちばん良い状態」を **y-30p20 だけ見て選んだのが誤り**。
        #     140 は y-30p20 では 977mm³ だが、match で旋回リングに
        #     961mm³ 入る（全姿勢の合計では 180 のほうが良い）。
        #       180: match 0 + y-30p20 1,087 = 1,087mm³
        #       140: match 961 + y-30p20 977 = 1,938mm³
        #     状態を比べるときは 1 姿勢で決めない。
        # ⚠ ここを終端より 20mm 外（|Y| 140）に置くと、半径 141 になって
        #   旋回リングの内縁（半径 140）に 0.00mm で当たる。終端と同じ
        #   |Y| で立ち上げれば半径 121 で、内縁まで 19mm 空く。
        (P.CABLE_THRU_X, P.CABLE_TURRET_END_Y, 826.0),
        (P.CABLE_THRU_X, P.CABLE_TURRET_END_Y, 870.0)],
        # ⚠ 中間点を足すと後ろの添字がずれる。柱に沿わせる点は
        #   「デッキから外へ出た次の点」なので 3。
        [(0, ("deck_plate", "z", 1)), (3, ("post_rear_L", "y", -1))], 12.0),
        "cab_turret", to=("deck_plate", "post_rear_L"), how="ROUTE",
        note="後柱の溝に沿わせる。以降は旋回体側ハーネス")
    # ⚠ **U ループの可動側**。長いあいだ描いておらず、配線の片端検査で
    #   「ループ免除 1」として数えていた箇所。可動端はキャリッジと一緒に
    #   316mm 動くので 1 本の静的な折れ線では表せない。代わりに
    #   **行程ぶんの通り道（ケーブルベアが占める帯）を実体で描く**。
    #   実物より大きいが干渉検査としては安全側で、キャリッジ側板とは
    #   どの姿勢でも接する（＝片端にならない）。
    #   ⚠ 帯の場所は実測で探した。固定物だけを障害物として 14mm 角のセルで
    #     全 X 範囲を走査すると、空いているのは
    #       Z 765..827 の |Y| 275..325（下の広い帯）
    #       Z 846..866 の |Y| 275..325（駆動軸の上）
    #     の 2 つだけ。Z 832..844 は駆動軸（|Y| 242..335）が塞ぐ。
    #   ⚠ キャリッジ自身の部品（後端横梁 Z 764..787 / ベルトクランプの腕
    #     Z 791..811 / 受け金具 Z 810.5..813.5）を避けると、下の帯で使えるのは
    #     **Z 815..827 の 12mm** だけ。φ10 の束が通る。
    #   ⚠ 上下 2 段の U ループにすると、曲がる部分が Z 832..844 を横切って
    #     駆動軸に当たる（可動端と一緒に動くので必ず通る）。
    #     **横に寝かせた U ループ**（2 本の直線を Y 方向に並べる）にする。
    #   ⚠ 別部品にすると固定側の折れ線とのあいだで「離れ」か「食い込み」に
    #     なる。同じハーネスの続きなので**融合して 1 部品**にする。
    #   ⚠ 帯の X は**機体の中だけ**（-485..-20）。行程いっぱいに -600 まで
    #     伸ばしたら、スタート姿勢の外形が 1085mm になって規定 3.2.2
    #     （1000mm）を破った。機体の外へ出るのは競技中だけで、スタート時の
    #     ケーブルベアは畳まれている。機体の外には何も無いので、
    #     中だけ描いてもキャリッジ側板とはどの姿勢でも重なる（接触が続く）。
    yb0, yb1 = 284.0, P.CAR_SIDE_Y - 2.5
    bay = (Pos((-485.0 - 20.0) / 2, (yb0 + yb1) / 2, 822.5)
           * Box(465.0, yb1 - yb0, 11.0, align=CTR)
           # 立ち上がり: 帯の角から固定側の着地点（Z 830..840）へ繋ぐ
           + Pos(-325.0, 306.0, 833.5) * Box(12.0, 12.0, 25.0, align=CTR))
    # グラバー系: ストロークぶんの余長を吸収する U ループ（ケーブルベア相当）。
    # ⚠ 「斜路の通路幅より外」だけでは足りない。**旋回体の掃引包絡の外**に出すこと。
    #   Z>760 では砲塔サイドプレートがヨー±30° で X=-27.5 まで来る（sweep_envelope.py）。
    #   以前は X=200 まで前に出していて、ヨー+30° で 0.90mm まで詰まっていた。
    # → 前端を X=-60（包絡の外）で止め、|Y| もレール外側寄り（335）に振る。
    put(parts, L.cable_to([
        # ⚠ |Y|=333 だとレール取付プレート（336..340）に φ10 の束が
        #   184mm³ かかる。プレートの内面より内側（330 → 325..335）を通す。
        # ⚠ φ10 の束は最小曲げ半径 40mm。90° 折るには両隣に 40mm の
        #   直線が要る。節点を詰めすぎると実機で被覆が潰れる。
        (-100, 330, 895.0), (-200, 330, 895.0), (-370, 330, 895.0),
        # ⚠ アルミ押出材の面には**溝**がある。桁の中心高さ（Z=828）に
        #   着地させると溝の底までしか届かず 0.83mm 空く。
        #   溝を外した高さ（Z=833）に当てること。
        # ⚠ 833 では φ10 の束の下端が 828 になり、キャリッジ側板の上端
        #   （826）まで 2mm しかない。キャリッジは 316mm 走るので擦れる
        #   （要 3mm）。835 にすると 4mm 空き、取付プレート（Z 779..838）
        #   への着地も 830..838 で面が残る。
        (-370, 285, 845.0), (-325, 285, 835.0),
        (-325, P.RAIL_PLATE_Y - 5.0, 835.0)],
        # ⚠ 上桁（340..360）に着地させると、手前のレール取付プレート
        #   （336..340）を必ず通過して 184mm³ 削る。沿わせる相手は
        #   **手前にある面**＝プレートの内面（RAIL_PLATE_Y）にする。
        #   プレートは配線より後に組むので F.face は使えない。
        [], 10.0) + bay,
        "cab_grabber", to=("rail_plate_L", "car_side_L"), how="ROUTE",
        note="ケーブルベア。可動側は行程ぶんの帯として描く"),
    # ⚠ **U ループの可動側**。ここは長いあいだ描いていなかった（配線の
    #   片端検査で「ループ免除 1」として数えていた箇所）。
    #   可動端はキャリッジと一緒に 316mm 動くので、1 本の静的な折れ線では
    #   表せない。代わりに**行程ぶんの通り道（ケーブルベアが占める帯）を
    #   そのまま実体で描く**。実物より大きいが、干渉検査としては安全側で、
    #   キャリッジ側板とはどの姿勢でも接する（＝片端にならない）。
    #   ⚠ 帯の場所は実測で探した。固定物だけを障害物として 14mm 角のセルで
    #     全 X 範囲を走査すると、空いているのは
    #       Z 765..827 の |Y| 275..325（下の広い帯）
    #       Z 846..866 の |Y| 275..325（駆動軸の上）
    #     の 2 つだけ。Z 832..844 は駆動軸（|Y| 242..335）が塞ぐ。
    #   ⚠ キャリッジ自身の部品（後端横梁 Z 764..787 / ベルトクランプの腕
    #     Z 791..811 / 受け金具 Z 810.5..813.5）を避けると、下の帯で使える
    #     のは **Z 815..827 の 12mm** だけ。φ10 の束が通る。
    #   ⚠ 上下 2 段の U ループにすると、曲がる部分が Z 832..844 を横切って
    #     駆動軸に当たる（可動端と一緒に動くので必ず通る）。
    #     **横に寝かせた U ループ**（2 本の直線を Y 方向に並べる）にする。
    # 10×15 の小型ケーブルベア（0.12kg/m）× 465mm + 中の配線
    LEDGER.add("グラバー配線 ケーブルベア", 0.120, "概算", "配線")
    # センサー
    for sx, nm in ((1, "front"), (-1, "rear")):
        # ⚠ デッキ上の -Y 側は機器で埋まっている:
        #     バッテリー |Y|<=17.5 / C610 4 個 |Y| 109..206
        #   空いているのは **|Y| 17.5..109** だけ。椅子（Y>=-50）も外れる
        #   ので、その帯の中央 -63 を通す。
        # ⚠ 終端は LiDAR 本体の**上面**（＝吊り下げたときの取付面）。座の
        #   φ40 の穴が本体の真ん中を開けてあるので、そこを真上から下ろす。
        #   斜めに入れると穴の縁に当たるので、**穴の真上まで来てから**降ろす。
        # ⚠ 椅子（X 130..460、Y -50..310、Z 150..162）は前方を塞ぐ。
        #   Y は -50 より外、X は 460 より前でしか Z 150 の帯を通れない。
        #   LiDAR へ寄せるのは X 476（椅子より前）まで出てから。
        put(parts, L.cable_to([(0, -63, z_deck), (sx * 400, -63, z_deck),
                               (sx * 476, -63, P.BASE_Z1 + 12.0),
                               (sx * 476, 0, P.BASE_Z1 + 12.0),
                               (sx * (P.LIDAR_LOW_X + 2.0), 0,
                                P.BASE_Z1 + 12.0)],
                              [(0, ("deck_plate", "z", 1)),
                               (-1, (f"lidar_low_{nm}", "z", 1))], 6.0),
            f"cab_lidar_{nm}", to=("deck_plate", f"lidar_low_{nm}"),
            how=("ROUTE", "CONNECT"))
    # 柱の溝に沿って上げ、+X 面のブラケットへ回り込む
    # ⚠ デッキから斜めに上げるとマスコット外形を横切る。
    #   **|Y|>=320 まで外へ出てから**柱に沿って上げること。
    # ⚠ 椅子（X 90..420 / Y -50..310 / Z 150..）を横切らないこと。
    #   デッキから **|Y|>=330 の外側**を通ってから前へ出す。
    # ⚠ 柱（X 370..390 / |Y| 340..360）の**外側の面**（|Y|=360）に沿わせる。
    #   350 は柱の中。ブラケットの手前で回り込む。
    # ⚠ コネクタは**本体**に刺さる。ブラケットの面に着地させると、
    #   本体（Z 465..535）まで 17mm 届かないまま「接続済み」になる。
    #   宣言した相手の面に着地させること。
    # ⚠ 2026-08-06: 吊り下げに変えたので、上段は**本体の下面**が開いた面に
    #   なった（座は本体の上）。そこへ下から着地させる。立板（X 390..398 /
    #   Y 320..390）を避けるため、|Y|=396 まで外へ出てから登る。
    put(parts, L.cable_to([(0, 150, z_deck), (-60, 330, z_deck), (200, 366, z_deck),
                           (360, 366, 200.0), (380, 396, 400.0),
                           # φ6 は最小曲げ半径 24mm。21mm では折れる
                           (400, 396, P.LIDAR_HIGH_Z - 50),
                           (P.LIDAR_HIGH_X, 396, P.LIDAR_HIGH_Z - 32),
                           (P.LIDAR_HIGH_X, P.LIDAR_HIGH_Y,
                            P.LIDAR_HIGH_Z - 26)],
                          [(0, ("deck_plate", "z", 1)),
                           (-1, ("lidar_high", "z", -1))], 6.0),
        "cab_lidar_high", to=("deck_plate", "lidar_high"),
        how=("ROUTE", "CONNECT"))
    # ⚠ 配線どうしは同じ経路を共有するので必ず触れる。**触れてよい**が、
    #   それは束ねてあるからで、宣言が無ければただの干渉と区別できない。
    #   デッキ上の帯を共有する束を明示する。
    #   触れている組**だけ**を書く。触れていない組を書くと、今度は
    #   「束ねると宣言したのに離れている」になる。
    F.fix("cab_lidar_high", "cab_turret", "BUNDLE", note="デッキ上で同じサドルに通す")
    F.fix("cab_lidar_front", "cab_lidar_rear", "BUNDLE",
          note="前後 LiDAR は同じ起点から出る（-Y 側の空き帯を共有）")
    F.fix("cab_drive_rl", "cab_turret", "BUNDLE",
          note="後方の駆動配線と砲塔系はデッキ上で交差する")
    return parts


def _hopper():
    """ホッパー（車体固定）: 4側壁 + 傾斜底 + 前壁リップ。"""
    t = P.HOP_DANPLA_T
    x0, x1, y = P.HOP_X0, P.HOP_X1, P.HOP_Y
    z_top = P.HOP_TOP_Z
    z_front = z_top - P.HOP_DEPTH_FRONT
    drop = (x1 - x0) * 0.0 + (x1 - x0) * (0.0)  # 明示化のためのゼロ項
    rise = (x1 - x0) * abs(sin(radians(P.HOP_FLOOR_SLOPE)) / cos(radians(P.HOP_FLOOR_SLOPE)))
    z_back = z_front + rise
    parts = []

    def wall(w_len, w_h, *_ignored):
        """板厚 Z の平板として作って返す（呼び側で立てる）。

        ホッパーは雑巾を寄せるだけで、荷重は 14 枚 × 48g = 0.7kg しかない。
        A5052 t1.0 の無垢で 634g あり、機体で 3 番目に重い製作品だった。
        肉抜きで削るより、**材料をプラダンに替える**ほうが効く:
          A5052 t1.0  2.68 kg/m²  →  プラダン t4.0  0.80 kg/m²
        中空板なので t1.0 のアルミ板より曲げ剛性はむしろ高い。
        穴あけの必要もなくなる（＝加工工数も減る）。
        """
        return L.mat(Box(w_len, w_h, t, align=CTR), "PP_DANPLA")

    # 側壁 2 枚。側面トラス後柱に吊る
    # ⚠ 側壁の上端はシンギュレータ軸（Z 716..724）の軸受まで届かせる。
    #   712 で止めると、軸を「側壁で受ける」と宣言しながら 4mm 空く。
    h = z_top - z_front + 40.0
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        put(parts, Pos((x0 + x1) / 2, sy * (y + t / 2), z_top - h / 2 + 20.0)
            * Rot(90, 0, 0)
            * (wall(x1 - x0, h)
               - Pos(P.HOP_X1 + 5.0 - (x0 + x1) / 2,
                     P.FEED_RAMP_Z0 + P.PICK_ROLLER_DIA / 2 - (z_top - h / 2 + 20.0), 0)
               * Cylinder(11.0, 3 * t, align=CTR)),
            f"hop_side_{sd}",
            to=(f"hop_hanger_{sd}0", f"hop_hanger_{sd}1"), how="RIVET",
            note="プラダンは座金付きリベット + 角アングル")
    # 後壁
    h_back = z_top - z_back + 10.0
    # ⚠ `Rot(90,0,90)` は X→Z / Y→X の写像で、622mm の長辺が**立ってしまう**。
    #   実測すると 120×4×622 になる（欲しいのは 4×622×120）。
    #   回転の合成は勘で書かず、bounding_box で確かめてから使うこと。
    UPRIGHT = Rot(0, 0, 90) * Rot(90, 0, 0)
    put(parts, Pos(x0 - t / 2, 0, (z_back + z_top) / 2) * UPRIGHT
        * wall(2 * y + 2 * t, h_back), "hop_back",
        to=("hop_side_L", "hop_side_R"), how="RIVET")
    # 前壁（送り出し基準）。**上端がそのまま分離ゲートの土台**になる。
    # ⚠ 以前は上端を HOP_TOP_Z(712) まで立て、ローラー（最下点 Z=700）が
    #   顔を出す窓を 5 つ開けて逃がしていた。だが窓は幅 46 ×5 = 230mm しか
    #   なく、**残る 390mm では布が壁の上端に当たって出られない**。
    #   ローラーだけ壁から出ても、布が出られなければ送給にならない。
    #   窓で逃がすのをやめ、壁の上端そのものを通り道の高さ
    #   （P.HOP_FRONT_TOP_Z = ゲート 695 − パッド 3 − 台 1 = 691）まで下げる。
    #   これで軸受ブロック（Z 705..735）の逃げも要らなくなる。
    z_ftop = P.HOP_FRONT_TOP_Z
    front = wall(2 * y + 2 * t, z_ftop - z_front)
    put(parts, Pos(x1 + t / 2, 0, (z_front + z_ftop) / 2) * UPRIGHT * front, "hop_front",
        to=("hop_side_L", "hop_side_R"), how="RIVET")
    # --- 分離ゲート（リタードパッド）------------------------------------
    # ⚠ **ここが「捌く」の実体。** 送るローラーだけでは、1 枚目に連れられた
    #   2 枚目・3 枚目がそのまま一緒に出る（重送）。ローラーの真下に
    #   「1 枚ぶんだけ開いた門」を作り、乗り上げた 2 枚目を摩擦で戻す。
    # ⚠ ゲートは**回らない**ので車体側（ここ）に置く。シンギュレータの
    #   リンクに置くと、回るリンクの部品を前壁にリベット留めすることになる。
    # ⚠ 隙間の基準はローラー軸ではなく**軸受ブロック**から取りたいところ
    #   だが、ブロック（A5052）と前壁（プラダン）のあいだにはリベットの
    #   段が 1 つ入る。ここは前壁の上端を基準面にして、パッド上面が
    #   ローラー最下点から SING_GATE_GAP だけ下に来るよう寸法を積む。
    # ⚠ パッドは**ローラーの真下の 5 か所だけ**。門として効くのは布を
    #   引っ張っている場所だけで、そこ以外に敷いても摩擦材が増えるだけ。
    #   ローラーとローラーのあいだは前壁の上端（691、パッド上面より 4mm 下）が
    #   そのまま案内面になる。
    # ⚠ **未解決: 山が減るとローラーが布に届かない。**
    #   前壁の上端 691 に対し、床は前端 592。雑巾 14 枚を縫い代込み 5mm/枚と
    #   見ると山の頂は 662 で、ゲート 695 まで 33mm 足りない（枚数が減れば
    #   さらに離れる）。プリンタの給紙カセットと同じ**圧板（ばねで山を
    #   ローラーへ押し上げる板）**が要る。プラダン t4 で 120×620 なら 60g で、
    #   いまの余裕（10g）では載らないので入れていない。
    #   → 満載時（山の高さ 100mm 以上）だけ成立する状態であることを明記する。
    #     圧板を入れるか、床の前端を上げる（HOP_DEPTH_FRONT を下げる）かの
    #     どちらかを、質量の枠が空いた時点で決めること。
    x_wc = x1 + t / 2                       # 前壁の板厚中心
    for k, py in enumerate(P.PICK_ROLLER_Y):
        clip = put(parts, Pos(x_wc, py, z_ftop) * L.retard_clip(
                P.SING_PAD_W, t, P.SING_CLIP_T, P.SING_PAD_LEN, 6.0),
            f"sing_clip{k}", to="hop_front", how="RIVET",
            note="2-φ3.2 座金付きリベット（プラダンは座金必須）")
        # パッドは台の web の上面に載る。-X 端（布が来る側）を 45° に削ぐ
        pad = put(parts, Pos(x_wc - t / 2 - P.SING_CLIP_T, py, z_ftop + P.SING_CLIP_T)
            * L.retard_pad(P.SING_PAD_W, P.SING_PAD_LEN, P.SING_PAD_T),
            f"sing_pad{k}", to=f"sing_clip{k}", how="PRESS",
            note="シリコン板を台へ落とし込んで接着（摩耗したら単独で交換）")
    LEDGER.add_solid("リタードパッド台 A5052 t1", clip, "A5052", "装填",
                     qty=len(P.PICK_ROLLER_Y))
    LEDGER.add_solid("リタードパッド シリコン t3（消耗品）", pad, "SILICONE", "装填",
                     qty=len(P.PICK_ROLLER_Y))
    # ホッパー吊り金具。**側壁（|Y|=310..314）と後柱（340..360）のあいだ
    # 26mm には何も無かった**。「後柱にリベット留め」と宣言していたが、
    # 届く実体が無ければ実機では吊れない。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # 柱は 1 本（X=RAIL_X0）なので、金具もその X に置く。
        # 高さを変えて 2 枚にし、前後の倒れも受ける。
        for k, hz in enumerate((z_top - 30.0, z_top - 100.0)):
            # ⚠ 板の端面（4×20 = 80mm²）で柱に留めることはできない。
            #   側壁の外面から柱まで水平に渡し、柱の面で立ち上げる。
            # ⚠ **L 字を 1 部品にしない**（曲げられない）。水平板と立板を
            #   別々に置き、立板を水平板の端面のタップへ留める。
            #   端面に M5 を立てるので水平板は **t8**（`L.TAP_MIN[5] = 7.5`）。
            hh, vv = L.angle_hanger_parts(60.0, 26.0, 40.0, 8.0, sy)
            org_hg = Pos(P.RAIL_X0, sy * (y + t), hz)
            put(parts, org_hg * hh, f"hop_hanger_{sd}{k}",
                to=f"hop_hanger_{sd}{k}v", how="BOLT",
                note="2-M5（水平板の端面にタップ）")
            put(parts, org_hg * vv, f"hop_hanger_{sd}{k}v",
                to=f"post_rear_{sd}", how="TSLOT", note="2-M5 溝ナット")
            # 水平板は立板のぶん短くした（run 26 → 18mm）ので概算も直す
            LEDGER.add("ホッパー吊り金具 水平板 A5052 t8", 0.023, "体積概算", "装填")
            LEDGER.add("ホッパー吊り金具 立板 A5052 t8", 0.052, "体積概算", "装填")
    # シンギュレータ軸（X HOP_X1+1..HOP_X1+9、Z 716..724）の軸受ブロック。
    # ⚠ 軸は側壁の +X 端（HOP_X1）より前にあるので、側壁に直接は受からない。
    #   側壁を前へ伸ばすと前壁と 1,920mm³ 重なる。**内面に軸受を出す**。
    ax_x = P.HOP_X1 + 5.0
    ax_z = P.FEED_RAMP_Z0 + P.PICK_ROLLER_DIA / 2
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ 穴は軸径（φ8.4）ではなく**ブッシュ外径**（φ12）で開ける。
        #   軸径で開けると、注記に書いた「フランジブッシュ」が入らない。
        blk = put(parts, L.mat(Pos(ax_x, sy * 305.0, ax_z)
                         * (Box(30.0, 10.0, 30.0, align=CTR)
                            - Rot(90, 0, 0) * Cylinder(P.SING_BUSH_OD / 2, 12.0,
                                                       align=CTR)), "A5052"),
            f"sing_brg_{sd}", to=f"hop_side_{sd}", how="BOLT",
            note="2-M4 + フランジブッシュ")
        LEDGER.add_solid("シンギュレータ軸受ブロック A5052", blk, "A5052", "装填")
        # フランジブッシュ。**注記だけで実体が無かった部品**。
        # ⚠ 鍔はホッパーの**内側**（|Y|=300 の面）へ出す。外側へ出すと
        #   軸継手（|Y| 316 から）と同じ側に並び、回る継手が鍔をこする。
        bush = put(parts, Pos(ax_x, sy * 305.0, ax_z) * Rot(-90 * sy, 0, 0)
                   * L.flange_bush(P.SING_SHAFT_DIA, P.SING_BUSH_OD, 10.0, 18.0),
                   f"sing_bush_{sd}", to=f"sing_brg_{sd}", how="PRESS",
                   note="無給油フランジブッシュ（φ12 穴へ圧入）")
        LEDGER.add_solid("フランジブッシュ POM φ8/φ12", bush, "POM", "装填")
    # --- シンギュレータの駆動（車体側）-----------------------------------
    # ⚠ モーターの**ハウジングは回らない**。以前は M2006 をまるごと
    #   `link_singulator()`（回るリンク）に描き、しかも固定先はカップリング
    #   だけだった＝**自分の出力軸で自分をぶら下げていた**。「ハウジングは
    #   車体側に置く」と注記にはあったが、置かれていなかった。
    #   本体をここへ移し、回るのは出力軸だけにする。
    # ⚠ 取付板を側壁（プラダン）へ留めてはいけない。中空板にモーターの
    #   反力（1.0N·m）は効かない。軸を受けているのと**同じ A5052 ブロック**
    #   の上面へ載せる（軸とモーターの芯が同じ部品で決まる）。
    y_wall = F.face("hop_side_L", "y", 1)
    y_c0 = y_wall + 2.0                                  # 継手の -Y 端 316
    y_c1 = y_c0 + P.SING_CPLG_LEN                        # 継手の +Y 端 346
    brk_t = 3.0
    # ⚠ 継手と取付板の隙間は **1mm しか取れない**。M2006 の出力軸は 12mm
    #   しかなく、板厚 3 を通したあと残るのが 9mm。継手の掴み代を 8mm
    #   （軸径の 1.0 倍）確保すると隙間は 1mm になる。
    #   隙間 2mm にすると掴み代が 7mm（0.875 倍）まで落ちて滑る側に振れる。
    #   回るのは継手の端面（振れは軸受ブロックの穴公差ぶん）なので、
    #   組立公差 0.25mm に対して 1mm あれば擦らない。
    y_mot = y_c1 + 1.0 + brk_t                           # モーター取付面 350
    z_blk = F.face("sing_brg_L", "z", 1)
    x_b0, x_b1 = F.face("sing_brg_L", "x", -1), F.face("sing_brg_L", "x", 1)
    y_b0 = F.face("sing_brg_L", "y", -1)
    # 腕はブロックの**上面**に載せる。端面（10×30）で突き合わせると
    # 座面が 300mm² 出ず、ボルトの座が取れない。
    arm = Pos((x_b0 + x_b1) / 2, (y_b0 + y_mot) / 2, z_blk + brk_t / 2) \
        * Box(x_b1 - x_b0, y_mot - y_b0, brk_t, align=CTR)
    fc = Pos(ax_x, y_mot - brk_t / 2, ax_z)
    face = fc * Box(44.0, brk_t, 44.0, align=CTR)
    face -= fc * Rot(90, 0, 0) * Cylinder(6.0, brk_t + 2, align=CTR)   # 軸の通り穴
    for bx, bz in bolt_circle(4, P.M2006_BOLT_PCD, 45.0):
        face -= Pos(ax_x + bx, y_mot - brk_t / 2, ax_z + bz) \
            * Rot(90, 0, 0) * Cylinder(1.7, brk_t + 2, align=CTR)
    # ⚠ A5052 t3 だと 34g ある。残り 10g の枠では持てないので 3D プリント
    #   （PETG）にする。M2006 は 90g・反力 1.0N·m で、PCD26 のボルト 4 本に
    #   かかる力は 1 本 19N。PETG の腕（30×3）でも面圧 0.2MPa で足りる。
    brk = put(parts, L.mat(arm + face, "PETG"), "sing_mot_brk",
              to="sing_brg_L", how="BOLT", note="2-M4（ブロックの上面へタップ）")
    LEDGER.add_solid("シンギュレータ モーター取付金具 PETG", brk, "PETG", "装填")
    # ⚠ この金具は「腕（ブロックの上面に載る板）＋面板（44 角の立板）」の
    #   1 ソリッドなので、bbox が Z 方向に 44mm 伸びる。`screw_place` は
    #   bbox の重なりが最も薄い軸を接触の法線とするので、**当たっている
    #   のは腕の下面なのに、そこを法線と見なせない**（「接触面に両方の
    #   材料がある場所が無い」で落ちていた）。腕の位置は分かっているので
    #   ここで直接置く。
    y_b1 = F.face("sing_brg_L", "y", 1)
    _cx, _cy = (x_b0 + x_b1) / 2, (y_b0 + y_b1) / 2
    if (x_b1 - x_b0) >= (y_b1 - y_b0):
        _seats = [(_cx + s * (x_b1 - x_b0 - 14.0) / 2, _cy, z_blk + brk_t)
                  for s in (-1, 1)]
    else:
        _seats = [(_cx, _cy + s * (y_b1 - y_b0 - 14.0) / 2, z_blk + brk_t)
                  for s in (-1, 1)]
    face_screws(parts, [Pos(*s) for s in _seats], "sing_mot_brk", "sing_brg_L",
                4, brk_t, "sing_motbrk")
    put(parts, Pos(ax_x, y_mot, ax_z) * Rot(90, 0, 0) * L.m2006(with_shaft=False),
        "sing_motor", to="sing_mot_brk", how="BOLT", note="4-M3 PCD26")
    LEDGER.add("M2006 P36 (シンギュレータ)", P.M2006_MASS, "DJI カタログ", "装填")
    # 傾斜底
    # ⚠ 傾いた板を「斜辺の長さ -t」で作ると、**両端が壁に届かない**。
    #   実際 X 方向で前壁と 3.7mm 空き、後壁とも接していなかった。
    #   長めに作って前後壁の内面（x0 / x1）で**垂直に切り落とす**。
    #   こうすれば端面が壁の面と平行になり、面で当たる。
    ln = hypot(x1 - x0, rise) + 4 * t
    floor = (Pos((x0 + x1) / 2, 0, (z_front + z_back) / 2 - t / 2)
             * Rot(0, degrees(atan2(-rise, x1 - x0)), 0)
             * L.mat(Box(ln, 2 * y, t, align=CTR), "PP_DANPLA"))
    for xc, sx in ((x1, 1), (x0, -1)):
        floor -= Pos(xc + sx * 400.0, 0, (z_front + z_back) / 2) \
            * Box(800.0, 4 * y, 400.0, align=CTR)
    # ⚠ 床は柱（|Y| 340..360）まで届かない。留まるのは**側壁**（内面 |Y|=310）。
    #   届かない相手を固定先に書いても、実機では宙に浮く。
    # ⚠ 箱組なので床は前後壁とも留まる。側壁だけ書いていたのは宣言漏れ。
    put(parts, floor, "hop_floor",
        to=("hop_side_L", "hop_side_R", "hop_front"), how="RIVET")
    # ⚠ 「箱組」の質量は**プラダンの板だけ**から出すこと。`parts` 全部の
    #   体積にプラダンの密度を掛けていたので、吊り金具 4 個・軸受ブロック
    #   2 個（いずれも A5052 で個別に計上済み）が**二重に**乗っていた。
    #   材質の違う部品を 1 つの密度でまとめると、こういう重複は見えない。
    danpla = [p for p in parts if p.label in ("hop_side_L", "hop_side_R",
                                              "hop_back", "hop_front", "hop_floor")]
    LEDGER.add("ホッパー プラダン t4 箱組",
               sum(p.volume for p in danpla) * P.DENSITY["PP_DANPLA"] / 1000,
               "PP_DANPLA 体積計算", "装填")
    LEDGER.add("ホッパー 角補強アングル + リベット", 0.180, "概算", "装填")
    return parts


def _feed_ramp():
    """斜路搬送（45°）: 下面ガイド板 + 側板 + 押さえベルト系。"""
    x0, x1 = P.FEED_RAMP_X0, P.FEED_RAMP_X1
    z0, z1 = P.FEED_RAMP_Z0, P.FEED_RAMP_Z1
    ln = hypot(x1 - x0, z1 - z0)
    ang = degrees(atan2(z1 - z0, x1 - x0))
    parts = []
    # ⚠ 端部プーリ（φ44）は板の**外**に置く。板を端から端まで張ると
    #   プーリが 4 個とも板を貫く（846mm³ ×4）。板をプーリ 1 個ぶん短くする。
    put(parts, Pos((x0 + x1) / 2, 0, (z0 + z1) / 2) * Rot(0, -ang, 0)
        # ⚠ 上端プーリ（X 77.5..105.5、Z 779..807）の手前で止める。
        #   -56 では板の +X 端が 94 まで来て 23mm³ ×4 かかる。
        * L.mat(Box(ln - 92.0, P.FEED_RAMP_W, 1.0, align=CTR), "A5052"),
        "ramp_guide", to=("ramp_side_L", "ramp_side_R"), how="RIVET")
    # 側板。**高さを一定にできない**。60mm 一様にすると、傾斜に沿って Z が 699..851 に広がり
    #   キャリッジ横梁（Y-313..313 / Z764..767）… Y をどこに置いても跨がれる
    #   砲塔サイドプレート（旋回・Z820..950）    … Z820 を 31mm 超える
    # の両方に当たる。ガイド板の**下側**に付けて、通路の上を空ける。
    # 布は上から押さえベルトで拘束しているので、横ズレはこの高さで足りる。
    # ⚠ 側板の位置は**斜面の法線方向**で決める。world Z でずらすと、
    #   傾斜 54° では法線方向の量が cos をかけた値になり（-12.5 → -7.3）、
    #   端部軸（法線 +11..+17）に届かず「離れ」になる。
    #   +6 に置くと法線 -6.5..+18.5 を占め、軸を内側に含む。
    #   上端は world Z 815.8（砲塔サイドプレートの下端 820 まで 4.2mm）。
    q_side = 6.0
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        put(parts, Pos((x0 + x1) / 2 - q_side * sin(radians(ang)),
                       sy * P.FEED_RAMP_SIDE_Y,
                       (z0 + z1) / 2 + q_side * cos(radians(ang)))
            * Rot(0, -ang, 0) * L.mat(Box(ln, 1.0, P.FEED_RAMP_SIDE_H, align=CTR), "A5052"),
            f"ramp_side_{sd}", to=(f"ramp_foot_{sd}", f"ramp_strut_{sd}"), how="RIVET")
    # 支持。**斜路は今まで何にも留まっていなかった**（最も近い部材まで 45mm）。
    # 板を空中に置いても搬送路にはならない。下端をホッパー前壁へ、
    # 上端を砲塔台座横梁へ渡す。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        y = sy * P.FEED_RAMP_SIDE_Y
        # 下端 → ホッパー前壁
        # 前壁は X = HOP_X1 .. HOP_X1+t。脚はその**外面**から生やす。
        x_wall = P.HOP_X1 + P.HOP_DANPLA_T
        # ⚠ +2.0 のずらしを入れると壁の面から 2mm 浮く。面から始めること。
        # ⚠ 板の端面（3×24 = 72mm²）ではリベット 2 本の座が取れない。
        #   壁側に耳を立てて、板面で当てる。
        # ⚠ **足は耳の板厚（3mm）ぶん短くする。** 前は足を x_wall..x0 いっぱいに
        #   取り、耳を x_wall..x_wall+3 に立てていたので 2 枚が 3×3×9 = 81mm³
        #   重なっていた。耳は壁に面で当てる板、足は斜路を受ける板で、
        #   どちらも別々に切り出すので突き合わせないと組めない。
        # ⚠ **足は側板の「外側」へ出す（2026-08-05）。** 内側は
        #   リタードパッド（`sing_pad3` |Y| 247..283）と側板の内面
        #   （`ramp_side_L` |Y| 289.5..290.5）に挟まれて **6.5mm しか無い**。
        #   t8 にしたら側板へ 25mm³ 食い込んだ。外側（|Y| 290.5..300）を
        #   実測すると、その X/Z の帯には `sing_bush_L`（Z 711..729）しか
        #   居らず、足の Z（681..707）とは重ならない。
        # ⚠ **t3 → t6・高さ 24 → 26。** 耳との座面は「足の**端面**」なので
        #   面積を決めるのは足の板厚。3×24 = 72mm²、6×24 = 144mm² では
        #   要 150 に届かない。6×26 = 156mm² にする。M4 の端面タップも
        #   t6 で立つ（`L.TAP_MIN[4] = 6.0`）。
        foot = Pos(1.5, sy * 5.5, 0) * Box(x0 - x_wall - 3.0, 6.0, 26.0,
                                           align=CTR)
        # 耳の Y 幅はピックローラー（|Y| 250..280）に触れない 12mm に抑える
        # ⚠ 耳は本体と**同じ Y**（|Y| 282..294）に立てていた。ローラーは
        #   かわせていたが、その真下に付く**リタードパッド**（幅 36 →
        #   |Y| 283 まで）と 1mm 重なる。耳だけ 4mm 外へ寄せて 3mm 空ける。
        # ⚠ 耳の高さは 40（Z 674..714）だった。前壁の上端を 691 まで下げた
        #   ので、691 より上の 23mm は**どこにも当たらない板**になる。
        #   しかもそこは軸受ブッシュの鍔（|Y| 298.5 から、Z 711..729）と
        #   0.5mm ですれ違う位置で、外へ寄せた分そのまま擦れになる。
        #   壁の上端で切って 17mm にする（座面は 12×17 = 204mm²、要 80）。
        # ⚠ **耳の上端は足の上端（z0+6）まで伸ばす。** 前は壁の上端 691 で
        #   切っていたので、足（Z 682..706）との重なりが Z 9mm しか無く、
        #   座面が 27mm²（要 150）しか出なかった。壁より上は留まらないが、
        #   足を受ける面としては要る。軸受ブッシュの鍔（Z 711..729）には
        #   まだ 5mm 空く。
        # ⚠ **厚くするのは足のほう（t3 → t8）。** 耳を t8 にしても座面は
        #   増えない（当たっているのは**足の端面**なので、面積を決めるのは
        #   足の板厚 3mm のほう。実測 3×24 = 72mm²、要 150）。
        #   足を t8 にすれば座面 8×24 = 192mm² になり、同時に M4 のタップも
        #   足の端面に立つ（`L.TAP_MIN[4] = 6.0`）。耳は t3 のままでよい
        #   （耳は壁へ**面で**当たるので板厚は座面に効かない）。
        ear_top = z0 + 7.0
        ear_bot = z0 - 6.0 - 20.0
        # ⚠ **耳は別の平板にする。** 曲げられないので（`export_fab.CAN_BEND`）、
        #   水平の足と垂直の耳を 1 部品にすると作れない。
        # ⚠ 耳の Y は**足を覆いきる**こと。足（|Y| 290.5..296.5）から外れた
        #   ぶんは座面にならない（実測 6mm の足に対し耳が 3.5mm しか
        #   重なっておらず、144mm² のはずが 84mm² に落ちた）。
        #   |Y| 290.5..300.5 に置く（`sing_bush_L` は Z 711..729 なので当たらない）。
        # ⚠ 外端は |Y| 298.5 まで。300 から先はシンギュレータの軸受ブロック
        #   （`sing_brg_L` |Y| 300..310 / Z 704..734）で、耳の上端（707）と
        #   Z が重なる。幅 10（→300.5）にしたら実測 0.00mm で接した。
        #   足（290.5..296.5）を覆う 8mm で足りる。
        ear = Pos(-(x0 - x_wall) / 2 + 1.5, sy * 6.5,
                  (ear_top + ear_bot) / 2 - (z0 - 6.0)) \
            * Box(3.0, 8.0, ear_top - ear_bot, align=CTR)
        org_f = Pos((x_wall + x0) / 2, y - sy * 2.0, z0 - 6.0)
        # ⚠ **壁に留まるのは耳であって足ではない。** 足を `hop_front` へ
        #   「4-φ3.2 リベット」と宣言していたが、足が壁に当たっているのは
        #   端面（3×9 = 27mm²、要 80）だけ。壁に**面で**当たっているのは
        #   耳（12×17 = 204mm²）なので、宣言を耳の側へ移す。
        #   足は耳へ留める（耳 t8 の端面に M4）。
        ft = put(parts, L.mat(org_f * foot, "A5052"),
                 f"ramp_foot_{sd}", to=f"ramp_foot_{sd}e", how="BOLT",
                 note="2-M4（足の端面にタップ。耳は貫通）")
        put(parts, L.mat(org_f * ear, "A5052"),
            f"ramp_foot_{sd}e", to="hop_front", how="RIVET", note="4-φ3.2")
        LEDGER.add_solid("斜路 下端受け 足板 A5052 t6", ft, "A5052", "装填")
        LEDGER.add("斜路 下端受け 耳板 A5052 t3", 0.003, "体積概算", "装填")
        # 上端 → 台座横梁
        # ⚠ 支柱は台座横梁の**下面に座面をもって当たる**こと。
        #   斜めに突き当てると線接触になり、座面が 106mm² しか出ない。
        #   梁の下面（Z=818）まで斜めに上げ、そこから水平に 40mm 伸ばす。
        x_beam = F.face("pedestal_beam0", "x", 1)
        z_beam = F.face("pedestal_beam0", "z", -1)
        h = z_beam - z1
        run = x_beam - x1
        # ⚠ 支柱は側板（|Y| 289.5..290.5）を**包んではいけない**。
        #   幅 3 で中心 290 だと 288.5..291.5 で側板を飲み込み 7mm³。
        #   側板の外面（290.5）に face で当てる。
        sr = put(parts, L.mat(Pos((x1 + x_beam) / 2, y + sy * 2.0, (z1 + z_beam) / 2)
                              * Rot(0, -degrees(atan2(h, run)), 0)
                              * Box(hypot(run, h), 3.0, 24.0, align=CTR), "A5052"),
                 # ⚠ 留まるのは座金具の**垂直板**（`…v`）。2026-08-05 に
                 #   座金具を水平板と垂直板へ分けたので、`ramp_seat_{sd}`
                 #   （水平板・板厚 Z）に「ボルト」と書くと 3mm の小口に
                 #   留めることになる（`assembly_check` が「接していない」で落とす）。
                 f"ramp_strut_{sd}", to=f"ramp_seat_{sd}v", how="BOLT",
                 note="2-M4")
        # ⚠ 斜めの支柱を梁に直接当てると線接触になり座面が出ない。
        #   梁の下面に沿う水平の座金具を噛ませ、支柱はそこへ留める。
        #   （斜材とガセットの関係と同じ。角度の付いた部材は必ず座を要する）
        # ⚠ 座金具は 2 つの面を持つ L 字。
        #     水平面 … 梁の下面に当たる（板厚を Z に取る）
        #     垂直面 … 斜めの支柱の側面に当たる（板厚を Y に取る）
        #   どちらか一方だけだと、もう片方が線接触になる。
        # ⚠ 斜路の上端軸（X 88.5..94.5、|Y|<=290、Z 790..796）を避ける。
        #   幅 48 で中心 x_beam-24 = 71 だと +X 端が 95 まで来て軸を 85mm³
        #   削る。軸の手前（86）で止める幅 30 にする。梁（X 75..95）とは
        #   75..86 で 11mm 重なるので座面は取れる。
        # ⚠ +X 端は「支柱の -X 端（87.1）より先」かつ「上端軸の -X 端
        #   （88.5）より手前」。この 1.4mm の窓に収める必要がある。
        #   48 だと軸を 85mm³ 削り、86 だと支柱に 1.1mm 届かない。
        # ⚠ 垂直面は**支柱の側面**（|Y| 288.5..291.5 の -Y 側）に当てる。
        #   -18.5 では支柱から 18.5mm 離れたところに立っていて、
        #   「支柱にボルト留め」と宣言しながらどこにも触れていなかった。
        # ⚠ 垂直面は支柱と**面で**重ねる。X の重なりが 0.9mm しかないと
        #   座面が 66mm² しか出ない。上端軸（X 88.5..94.5、Z 790..796）は
        #   Z で避けられるので、垂直面を Z 798..818 に取って +X 側へ
        #   118 まで伸ばす（支柱と 31×20 = 620mm² で当たる）。
        # ⚠ 支柱は側板の外面（|Y| 290.5..293.5）に貼ってあるので、座金具の
        #   垂直面が当てられるのは支柱の**外側の面**（293.5）だけ。
        #   内側（290.5）は側板と共有していて、そこへ当てると側板に
        #   食い込む。左右で符号が変わるので sy を使う。
        # ⚠ 幅 40（|Y| 270..310）だとキャリッジ側板の内面（311.9）まで
        #   1.9mm しかない。側板は 316mm 走るので擦れる。36 にして 3.9mm。
        # ⚠ **側板を避けるだけでは足りなかった。** 側板をレールへ留める
        #   M4（`scr_a0061` ほか）は、ナットが側板の内面よりさらに内側
        #   （|Y| 309）まで出る。幅 36（|Y| 308）だと最小すきま 2.59mm で、
        #   `validate.py` の要求 3.0mm を 3 姿勢とも切っていた。しかも
        #   キャリッジは 316mm 走るので、ここは**すれ違う相手**。
        #   30 にして |Y| 305、ナットまで 4mm 空ける。梁との座面は
        #   13×30 = 390mm²（要 80）で足りる。
        # ⚠ **水平面と垂直面を別部品にする**（曲げられない）。水平板を梁の
        #   下面へ留め、垂直板は水平板の**端面のタップ**へ留める。
        org_st = Pos(68.0, y + sy * 0.0, z_beam)
        st = put(parts, L.mat(org_st * Pos(0, 0, -1.5)
                              * Box(40.0, 30.0, 3.0, align=CTR), "A5052"),
                 f"ramp_seat_{sd}", to="pedestal_beam0", how="BOLT", note="2-M4")
        # ⚠ **垂直板は水平板の下に潜らせる（2026-08-05）。** 前は Z 798..818 で、
        #   水平板（Z 815..818）と 40×3×3 = 360mm³ 重なっていた。平板 2 枚は
        #   同じ場所を占められない。上端を水平板の下面（815）に合わせる。
        # ⚠ **t3 → t6。** 水平板を貫く M4 は垂直板の**上の端面**に立てる
        #   ことになるが、t3 では M4 のタップが立たない（`L.TAP_MIN[4] = 6.0`）。
        #   厚くすると座面も 40×3 = 120mm²（要 150 に足りない）→ 40×6 = 240mm²
        #   になり、`assembly_check` の「座面が足りない締結」も同時に消える。
        #   支柱が当たる内側の面（|Y| 293.5）は動かさない。外面は 296.5 →
        #   299.5 だが、側板をレールへ留めるナット（|Y| 309）まで 9.5mm 空く。
        put(parts, L.mat(org_st * Pos(15.0, sy * 6.5, -13.0)
                         * Box(70.0, 6.0, 20.0, align=CTR), "A5052"),
            f"ramp_seat_{sd}v", to=(f"ramp_seat_{sd}", f"ramp_strut_{sd}"),
            how="BOLT", note="2-M4（垂直板の端面にタップ）")
        LEDGER.add_solid("斜路 上端座金具 水平板 A5052 t3", st, "A5052", "装填")
        LEDGER.add("斜路 上端座金具 垂直板 A5052 t6", 0.022, "体積概算", "装填")
        LEDGER.add_solid("斜路 上端支柱 A5052 t3", sr, "A5052", "装填")
    # ⚠ 「ガイド/側板」の質量は**ガイド板と側板だけ**から出すこと。
    #   `parts` 全部の体積を足していたので、下端受け・上端支柱・上端座金具
    #   （いずれも直前に個別計上済み）が**二重に**乗っていた。合計 75g、
    #   実際の板 76g とほぼ同じ量が幽霊として計上されていたことになる。
    #   概算値（12/16/9g）が実体（12.0/3.8/21.9g）とばらばらだったので、
    #   個別のほうも体積計算に直した。
    plates = [p for p in parts if p.label in ("ramp_guide", "ramp_side_L", "ramp_side_R")]
    LEDGER.add("斜路搬送 ガイド/側板 A5052 t1.0",
               sum(p.volume for p in plates) * P.DENSITY["A5052"] / 1000,
               "A5052 体積計算", "装填")
    # 押さえベルト（2本、幅50・ゴム）— 上側から布を挟んで搬送
    # ⚠ 以前は「平らな板 1 枚」で描いていた。ベルトは輪なので、端部プーリ
    #   （φ22）の**中を貫いて** 1,556mm³ ×4、端部軸も 589mm³ ×4 重なる。
    #   宣言（MESH 許容 3,200）の陰に隠れて検査は 0 と言うが、ビューアでは
    #   全部干渉に見える。プーリに**巻き付くスタジアム形**で描き直す。
    # ⚠ 持ち上げ量は「プーリ半径 + ニップ」で決まる。8 のままでは
    #   ベルトの下側の直線部がガイド板の中（-3）に入る。
    #   11 + 3 = 14 にすると下側の内面がガイド板上面から 2.5mm 上に来て、
    #   布を挟む隙間になる。上側の外面は斜路上端で Z 820.8（旋回リングの
    #   下面 838 まで 17mm）、端部軸は Z 796.6（側板の上端 799.8 の内）。
    p_off = 14.0
    pul_r = 11.0
    ends = (("lo", x0 + 12.0, z0 + 12.0 * (z1 - z0) / (x1 - x0)),
            ("hi", x1 - 12.0, z1 - 12.0 * (z1 - z0) / (x1 - x0)))
    ox, oz = -p_off * sin(radians(ang)), p_off * cos(radians(ang))
    b_span = hypot(ends[1][1] - ends[0][1], ends[1][2] - ends[0][2])
    belts = []
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        put(belts, L.mat(
            Pos((ends[0][1] + ends[1][1]) / 2 + ox, sy * 150.0,
                (ends[0][2] + ends[1][2]) / 2 + oz)
            * Rot(90, 0, 0) * Rot(0, 0, ang)
            * L.belt_loop(b_span, pul_r, 2.0, 50.0), "RUBBER"),
            f"ramp_belt_{sd}",
            to=(f"ramp_pul_{sd}_lo", f"ramp_pul_{sd}_hi"),
            how="MESH", note="端部プーリに巻き付く")
    LEDGER.add("搬送ベルト（ゴム 幅50）", sum(b.volume for b in belts) * 1.2e-3 / 1000,
               "ゴム 体積計算", "装填")
    # 端部プーリと軸。ベルトは輪であって板ではないので、掛かる相手が要る。
    # これが無いのでベルトは 28mm 浮いたままだった。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for tag, xe, ze in ends:
            # ⚠ 胴の幅はベルト幅（50）より広くする。50 のままだと鍔
            #   （半径 +3）がベルトの縁に食い込む。54 で片側 2mm 空ける。
            put(parts, Pos(xe + ox, sy * 150.0, ze + oz) * Rot(90, 0, 0)
                * L.pulley(2 * pul_r, 54.0, 6.0),
                f"ramp_pul_{sd}_{tag}", to=f"ramp_shaft_{tag}", how="PRESS")
    for tag, xe, ze in ends:
        put(parts, L.mat(Pos(xe + ox, 0, ze + oz) * Rot(90, 0, 0)
                         * Cylinder(3.0, 2 * P.FEED_RAMP_SIDE_Y, align=CTR), "STEEL"),
            f"ramp_shaft_{tag}", to=("ramp_side_L", "ramp_side_R"), how="ROTATE",
            note="軸受は側板の穴（フランジブッシュ）")
    LEDGER.add("搬送ベルト端部プーリ φ44 幅50", 0.030, "体積概算", "装填", qty=4)
    LEDGER.add("搬送ベルト軸 φ6", 0.026, "体積概算", "装填", qty=2)
    return parts + belts


def _electronics():
    parts = []

    def box(dims, x, y, z, name, label, group="電装", qty=1, material="MOTOR",
            to="deck_plate", how="BOLT",
            note="4-M3 + スペーサ（テクセルは座金）"):
        b = put(parts, L.mat(Pos(x, y, z) * L.box_part(dims), material),
                name, to=to, how=how, note=note)
        if len(dims) > 3 and label:
            LEDGER.add(label, dims[3], "実測/カタログ", group, qty)
        return b

    z0 = P.BASE_Z1 + P.DECK_T
    box(P.BATTERY, -150.0, 0.0, z0, "battery", "6S LiPo バッテリー", material="BATTERY",
        how="CLAMP" if "CLAMP" in F.HOW else "BOLT", note="マジックテープ + ベルト固定")
    box(P.MCU_BOARD, 0.0, 0.0, z0, "mcu", "制御基板 (STM32)", material="PCB")
    box(P.BREAKER, -350.0, 0.0, z0, "breaker", "主幹30Aブレーカー")
    # C620 は **幅 11.5mm の細長い基板**（公式CAD実測）。以前は 46mm 幅の箱と
    # 想定して 90mm ピッチで並べていたが、実寸なら 20mm ピッチで7枚が
    # 140mm に収まる。基板を立てず、長辺(93.1)を X に寝かせて並べる。
    for i in range(7):
        box(P.ESC_C620, -330.0, 150.0 - i * 20.0, z0, f"esc620_{i}", "")
    LEDGER.add("C620 ESC", P.ESC_C620[3], "実測/カタログ", "電装", qty=7)
    # C610 は 44.8 × 22.0 × 2.4 の薄板。25mm ピッチで4枚。
    # ⚠ デッキは Y -215..215 しかない。-180 から 25mm ピッチで 4 枚並べると
    #   4 枚目が -255 になり、**デッキの外に出て空中に浮いていた**。
    #   デッキの内側に収まる位置へ寄せる。
    for i in range(4):
        box(P.ESC_C610, -100.0, -120.0 - i * 25.0, z0, f"esc610_{i}", "")
    LEDGER.add("C610 ESC", P.ESC_C610[3], "実測/カタログ", "電装", qty=4)
    # 非常停止 ×2（前後・見やすい位置）+ 電源表示LED
    # 非常停止 ×2。⚠ **砲塔の掃引半径 331.9mm の外に置くこと**。
    #   以前は前側を (330, 352) に置いていたが、台座が φ80 あるので
    #   Y=322..382 を占め、ヨー-30° で回った砲塔サイドプレート（Y=324まで）と
    #   **2mm 重なっていた**。台座ごと |Y|>=340（側面上桁の真上）へ寄せる。
    # ⚠ 上桁ブラケット（柱の直近 X=-265/370 の上下）を避けた X に置く
    # ⚠⚠ **掃引半径をコメントにべた書きすると必ず古くなる。**
    #   上の「331.9mm」は YAW_ARM_X を 150→160 に動かした時点で嘘になって
    #   いて、実際の旋回アーム先端は hypot(160+30, 320) = **372.0mm**。
    #   前側の非常停止（軸から 365.8mm）はその内側にあり、ヨー +20° で
    #   アームに 3,177mm³ 埋まっていた。**全仰角で埋まる**ので、
    #   規定 7.2.4 の非常停止が押せない状態だった。
    #   match（ヨー 0）では出ないので、掃引を刻まないと見つからない。
    arm_r = hypot(P.YAW_ARM_X + 30.0, 320.0)
    # ⚠ アームは Y=0 でも半径 130..190 を通る「棒」なので、内側へ逃げる
    #   道は無い。外へ出すには軸から 380mm 必要で、取付板の内側の角まで
    #   含めると |Y|>=409（骨格から 59mm 突き出す）。
    #   → **アームの平面（Z 862..868）より下に付ける**のが正解。上桁の
    #     外側の面に外向きで付ければ、Z 798..858 に収まって掃引に入らず、
    #     しかも外から押しやすくなる（7.2.4 は視認性と操作性を要求する）。
    for (x, y), sd, beam, on_top in (((300.0, 360.0), "f", "topbeam1_L", False),
                                     ((-200.0, -360.0), "r", "topbeam1_R", True)):
        if on_top:
            put(parts, Pos(x, y, P.PEDESTAL_TOP_Z - 2.0) * L.plate(80.0, 60.0, 4.0),
                f"estop_plate_{sd}", to=beam, how="TSLOT", note="2-M5")
            put(parts, Pos(x, y, P.PEDESTAL_TOP_Z) * L.estop(),
                f"estop_{sd}", to=f"estop_plate_{sd}", how="BOLT",
                note="2-M4 + φ22 穴とロックリング")
        else:
            sy = 1.0 if y > 0 else -1.0
            put(parts, Pos(x, y + sy * 2.0, P.BEAM_TOP_Z - 10.0)
                * Rot(90, 0, 0) * L.plate(80.0, 60.0, 4.0),
                f"estop_plate_{sd}", to=beam, how="TSLOT", note="2-M5")
            put(parts, Pos(x, y + sy * 4.0, P.BEAM_TOP_Z - 10.0)
                * Rot(-90 * sy, 0, 0) * L.estop(),
                f"estop_{sd}", to=f"estop_plate_{sd}", how="BOLT",
                note="2-M4 + φ22 穴とロックリング（外向き）")
    LEDGER.add("非常停止 取付板 A5052 t4", 0.051, "体積概算", "安全", qty=2)
    LEDGER.add("非常停止スイッチ", 0.080, "カタログ", "安全", qty=2)
    # 前柱の +X 面（x=390）に当てる。410 では 8mm 浮いていた
    put(parts, Pos(P.FRONT_POST_X + P.EXT_W / 2 + 12.0, -352.0, P.ESTOP_Z)
        * Cylinder(12.0, 20.0, align=BASE),
        "power_led", to="post_front_R", how="TSLOT", note="2-M5 溝ナット")
    LEDGER.add("電源表示LED + 遠隔停止受信機", 0.250, "概算", "電装")
    # -----------------------------------------------------------------------
    # ⚠⚠ **未解決: 機上計算機（ミニPC）と 8 インチ表示器が質量の枠に入らない。**
    #
    # ■ まず、この図には**計算機が 1 台も載っていない**
    #   戦略書 §4.5.7 は機上計算機（AMCL 自己位置推定・弾道ソルバー・相手検出）を
    #   選定済みで、照準系はこれが無いと成立しない。LiDAR も ESC も制御基板も
    #   描いてあるのに、それらを束ねる計算機だけが図面から抜けていた。
    #   → **残枠 161.9g（34.838kg / 規定 35.0kg）は、必須部品を 1 つ数えていない
    #     見かけの余裕**である。ここを埋めると枠はほぼ無くなる。
    #
    # ■ 置き場所は実測で確保できている（**入らないのは質量だけ**）
    #   全姿勢（match / stowed / loading / ヨー±30×仰角20〜70 / グラバー0〜316）で
    #   空き直方体を走査した結果:
    #       ミニPC   X -250..-110 / Y   55..178 / Z 140..180   全姿勢で空き
    #       DC-DC    X -300..-240 / Y -150..-90 / Z 140..175   全姿勢で空き
    #       表示器   X -310..-283 / Y -360..-140 / Z 380..560  後柱以外は空き
    #   ⚠ デッキの上は見た目ほど空いていない。埋まっているのは
    #       バッテリー |Y|<=17.5 / C620 7 枚 X -377..-284 / C610 4 枚 Y -206..-109
    #       ブレーカー X -380..-320 / 制御基板 X -50..50
    #     さらに配線が 2 層で走る（電源幹線 Z 194..212 / 砲塔・LiDAR 系 Z 144..156）。
    #     残る島は X -280..-60 の **+Y 側**（Y 45..215）と -Y 側（Y -215..-45）の 2 つ。
    #     -Y 側は前後 LiDAR の束（Y=-63・Z 147..153）が全長を横切って割っているので、
    #     デッキ直置きの機器を置けるのは実質 **+Y 側の 180×160 だけ**。
    #   ⚠ 表示器は**上には置けない**。スタート時外形 970×803×1192 に対し、
    #     規定 1000×1000×1200 の余裕は高さ 8mm・前後 30mm しかない。空いているのは
    #     左右 197mm だけ。後柱（post_rear_R）の **-X 面**に後ろ向きで付ければ
    #     外形は 1mm も増えず、ホッパー（Z 572 以上）の下を視線が通る。
    #     ・試して駄目だった案 1: ホッパー後壁（X -439..-435）へ直付け。後壁は
    #       プラダン t4 で高さ 71mm（Z 646..717）しかなく 8 インチが載らない。
    #       後端の横材（Z 115..135）から柱を立てて受けると、HFS5-2020 は
    #       0.5kg/m なので 400mm × 2 本で **+400g**。表示器本体より重い。
    #     ・試して駄目だった案 2: 後柱の**外**（|Y| 361..400・全姿勢で空き）へ
    #       外向き。外形は増えないが画面が真横を向き、後ろの操縦者から見えない。
    #   実配置・配線まで一度組んで、**match 姿勢の組立検査**（浮き0/離れ0/未宣言接触0/
    #   食い込み0/曲げ不足0/配線片端0、許容内の重なりも 9,988mm³ から増えず）と
    #   外形（970×803×1192 のまま）が通ることを確認した。落ちたのは質量の 1 項目だけ。
    #   ⚠ そのとき画面は 200×130×12（仮寸）で描いた。実品 195×122×14 は Y/Z が
    #     小さく X が 2mm 厚いだけで、走査した空き（X -310..-283）の内側に収まる。
    #
    # ■ 実在品と質量（すべてメーカー公表値）
    #   ミニPC   MeLE Quieter4C (N100/N150)  131×81×18.3 / **203g** / 完全ファンレス
    #            USB-C 12V/2A・PL1 8W / GbE + USB3.2×2 + USB2.0 + HDMI×2
    #            https://store.mele.cn/pages/quieter-series
    #   （軽い側）MeLE PCG02 (N100) 140×60×19.5 / **130g** / ファンレス / 12V/2A
    #            https://store.mele.cn/pages/pc-stick-series
    #            ⚠ スティック型で **Ethernet が無い**。戦略書 §4.5.7 が第一候補に
    #              している北陽 UST-20LX は **100BASE-TX** なので、これを採るなら
    #              USB-GbE アダプタ（+25g 級）が別に要り、残枠 25g を使い切る。
    #            ⚠⚠ ついでに見つかった食い違い: 戦略書は UST-20LX（Ethernet）を
    #              第一候補にしているのに、**scripts/bom.py は LD19/STL-19P 相当
    #              （¥7,000 ×3・UART/USB）で見積もっている**。どちらに決めるかで
    #              計算機に要る I/O も価格も変わる。先に機種を確定すること。
    #   表示器   Elecrow SH080  195×122×14 / **280g** / Mini-HDMI + USB 5V/2A / <8W
    #            https://www.elecrow.com/sh080-8-inch-mini-hdmi-portable-lcd-display-1280x800-monitor-resolution-with-hdmi-port-built-in-speakers.html
    #            ⚠ **8 インチで 250g を切る「完成品」は実在しない**（公表値ベース）。
    #              250g 未満は裸パネル構成のみ（Pimoroni PIM372 パネル 125g /
    #              秋月 SHARP LQ079L1SX02 パネル 85g）で、どちらも**ドライバ基板の
    #              質量が非公表**＝買って測るまで総質量が確定しない。
    #   DC-DC    Pololu D36V50F12  25.4×25.4×9.5 / **7g** / 13.3-50V → 12V 4.5A(54W)
    #            https://www.pololu.com/product/4095/specs
    #            ⚠ 6S（19.8〜25.2V）直結は不可。降圧は必須。19V 系は 6S の下限が
    #              19.8V なので満充電時しか成立せず、100g 未満の 100W 級 19V 品も
    #              見つからなかった。**12V 系に決める**のが正しい。
    #   取付枠   表示器取付枠 PETG t4 205×132（後柱の -X 面へ 2-M5 溝ナット）約 50g
    #            ⚠ A5052 t3 だと同じ形で 236g。残枠 161.9g だけで足りない。
    #   ⚠ 電流も忘れないこと。scripts/power_budget.py の I_QUIESCENT は 1.2A
    #     （制御系・センサー）。ミニPC 24W ＋ 表示器 8W を 22.2V から取ると
    #     **+1.4A** で、待機電流がほぼ倍になる。規定 3.2.5 の 30A 枠は駆動系の
    #     遮断素子合計なので直接は当たらないが、スピンアップと重なる区間の
    #     余裕が減る。入れるときは power_budget.py の I_QUIESCENT も上げる。
    #
    # ■ 質量の勘定（残枠 161.9g に対して）
    #       ミニPC(Quieter4C) 203 + DC-DC 7            = 210g  →  **48g 超過**
    #       ミニPC(PCG02)     130 + DC-DC 7            = 137g  →  25g 残るが GbE 無し
    #         └ USB-GbE アダプタ +25g 級を足すと       ≈ 162g  →  ほぼゼロ〜超過
    #       ＋表示器 280 ＋取付枠 50                    = 540g  →  **378g 超過**
    #   → **どの組み合わせでも規定 35.0kg を破る。載せない。**
    #
    # ■ 塞がっているのは質量だけではない — **予算（規定 3.1.6）も足りない**
    #   scripts/bom.py の PURCHASED にも計算機は 1 行も無い。予算対象の合計は
    #   ¥383,670 / 上限 ¥400,000 で、**残りは ¥16,330**。戦略書 §4.5.7 は機上
    #   計算機を「約3万円」と見積もっているので、質量の枠が空いても**予算で
    #   もう一度落ちる**。入れるときは bom.py に行を足して 40 万円を再確認する
    #   こと（価格は購入前に実際の販売ページで確定する。ここには書かない）。
    #
    # ■ 枠が空いたら入る（DESIGN.md §7 の削減レバーが先）
    #   §7 は −2.6kg の削減計画を持っている。最初の 1 本（スライドレールを
    #   アルミ合金 3 段引へ、−0.9kg）だけで 540g は 1.7 倍の余裕をもって払える。
    #   順序は **①レール置換 → ②ミニPC＋DC-DC → ③表示器**。②は照準系の必須部品
    #   なので、③（ピット・立ち上げ用の表示）より必ず先に入れること。
    #
    # ■ 入れるときの配線（一度描いて検査を通した経路。座標はそのまま使える）
    #   cab_pc_in   ブレーカー天面(-350,-15,203) → (-350,-60,215) → (-320,-120,215)
    #               → (-290,-120,215) → DC-DC 天面(-290,-120,159)  φ6 / CONNECT×2
    #               ⚠ 電池ではなく**ブレーカーの負荷側**から取る。手前から取ると
    #                 非常停止でも遠隔停止でも PC の電源が切れない（規定 7.2.4）。
    #               ⚠ 降ろす X を -272 にすると表示器へ行く束を 41mm³ 貫く。-290 へ。
    #   cab_pc_pwr  DC-DC → (-262,-60,175) → (-230,20,175) → (-210,45,158)
    #               → ミニPC の -Y 面(-200,57,150)  φ6 / CONNECT×2
    #               ⚠ **空いている高さの帯は Z 170..190 の 20mm だけ**（上は電源幹線
    #                 Z 194..212、下は砲塔・LiDAR 系 Z 144..156 と機器そのもの）。
    #                 Z 185 に上げると電源幹線まで 1.5mm しか残らない（手計算）。
    #   cab_usb_mcu ミニPC の +X 面 → (-90,100,150) → (-70,40,150) → 制御基板の
    #               -X 面(-52.25,25,150)  φ4.5 / CONNECT×2
    #               ⚠ 「回路と USB 接続」の相手は**制御基板**。ESC（C620/C610）は
    #                 CAN でしか喋らないので USB は刺さらない。
    #               ⚠ Y=20 で寄せるとバッテリーまで 0.25mm。Y=25 にする。
    #   cab_usb_disp ミニPC の -X 面(-247.5,100,150) → (-275,100,150) → (-275,60,178)
    #               → (-275,-280,178) → 後柱内面(-281.5,-337.5,215)
    #               → (-281.5,-337.5,380) → (-295,-300,395)
    #               → 画面下面(-295,-300,422.5)   φ5 / CONNECT+ROUTE×2+CONNECT
    #               （枠は Y -360..-150 / Z 415..565、画面は Y -350..-150 / Z 425..555）
    #               ⚠ 柱に沿わせる X は**柱の中心（-275.4）から外す**。押出材の面の
    #                 中央には溝があり、中心に置いた束は溝の底まで届かず「離れ」に
    #                 なる（実際そう出た）。溝口の外の X=-281.5 に寄せる。
    #               ⚠ 取付枠（Z 415..565）の**下**をくぐって画面の下面へ入れる。
    #                 枠と柱のあいだ（X -289.4..-285.4）に束は入らない。
    #               ⚠ **USB-C 1 本（映像＋給電、外径 5.0）を前提にした経路**。
    #                 HDMI＋USB 給電の 2 本（束で外径 8）にすると、ミニPC を出る
    #                 90° の折れに直線が 32mm 要る（いまは 27.5mm）ので、出口を
    #                 -X 面から -Y 面へ振り直すこと。
    # -----------------------------------------------------------------------
    # ⚠ 2026-07-31 ユーザー判断: **質量は後で削る。先に完成させる。**
    #   上の調査どおりの実在品を、実体として入れる。
    #   （入れたぶん +0.90kg。規定 35.0kg は現時点で超過している。DESIGN.md §7 の
    #     削減レバーで払う前提。「載らないから描かない」にすると、置き場所も
    #     配線も図に無い状態が続いて、後から入れるときに同じ調査をやり直す。）

    # --- 機上計算機 MeLE Quieter4C（131×81×18.3 / 203g / 12V・ファンレス）---
    # ⚠ デッキの上で機器が置けるのは **+Y 側の 180×160 の島だけ**（-Y 側は
    #   前後 LiDAR の束が全長を横切る）。バッテリー（|Y|<=17.5）とも当たらない。
    z_deck_top = F.face("deck_plate", "z", 1)
    put(parts, L.mat(Pos(-182.0, 97.5, z_deck_top + P.PC_H / 2)
                     * Box(P.PC_X, P.PC_Y, P.PC_H, align=CTR), "PC"),
        "pc_mini", to="deck_plate", how="BOLT", note="4-M3 + スペーサ")
    LEDGER.add("機上計算機 MeLE Quieter4C", 0.203, "メーカー公表値", "電装")

    # --- 降圧 DC-DC Pololu D36V50F12（25.4×25.4×9.5 / 7g / 12V 4.5A）---
    # ⚠ 6S（19.8〜25.2V）直結は不可。19V 系は 6S の下限が 19.8V なので満充電時
    #   しか成立しない。**12V 系に決める**。
    put(parts, L.mat(Pos(-290.0, -120.0, z_deck_top + 9.5 / 2)
                     * Box(25.4, 25.4, 9.5, align=CTR), "PCB"),
        "pc_dcdc", to="deck_plate", how="BOLT", note="4-M2.5 + スペーサ")
    LEDGER.add("降圧 DC-DC Pololu D36V50F12", 0.007, "メーカー公表値", "電装")

    # --- 8 インチ表示器 Elecrow SH080（195×122×14 / 280g）---
    # ⚠ **片持ちにしない。** 後柱 1 本の -X 面に 205 幅の枠を留めると、
    #   柱（|Y| 340..360）から画面中心まで 190mm の片持ちになり、280g が先端に
    #   ぶら下がる。左右の後柱に渡した横材へ付ければ両端支持になり、画面も
    #   機体の中心に来て操縦者から見やすい。
    # ⚠ 高さ Z 470..490 は、ホッパー（Z 572 以上）と側面筋交い（-X 端 -257.7）の
    #   どちらにも当たらない帯。グラバーは Z 763 以上なので関係しない。
    # ⚠ **横材を柱の -X 面に「重ねて」いた（〜2026-08-07）。BRACKET と宣言して
    #   金具は 1 個も描かれておらず、実機では横材がただ柱の前に浮いている。**
    #   重ね継手には標準の L 金具が入らない。金具は腕を**それぞれの材の軸方向**
    #   へ伸ばして初めて穴が溝の芯に乗る（`bracket()` の穴は内隅から 11mm で、
    #   溝の芯は材の縁から 10mm。腕を軸と直角に伸ばすと必ず 1mm ずれて
    #   ナットに入らない）。幅 20 の金具では横材の溝（X -295.4）と柱の溝
    #   （X -275.4）が 20mm 離れていて、両方に届く向きが存在しない。
    #   → **柱の内側に渡す T 継手**にする（マスト横梁 `mast_cross` と同じ形）。
    #     横材の上下 2 か所の入隅に金具が入り、各端 2 個・計 4 個で留まる。
    #     画面は 20mm 前へ出るが、後端は フォーク（-710）なので包絡は変わらない。
    y_in = F.face("post_rear_R", "y", 1)             # -340（柱の内面）
    x_post_c = (F.face("post_rear_R", "x", -1)
                + F.face("post_rear_R", "x", 1)) / 2  # -275.4（柱の中心）
    beam_len = 2 * abs(y_in)
    put(parts, Pos(x_post_c, 0.0, P.DISP_Z) * Rot(0, 0, 90) * L.ext2020(beam_len),
        "disp_beam", to=("post_rear_L", "post_rear_R"), how="BRACKET",
        note="各端 金具 2 個・各 2-M5")
    L.add_ext("表示器横材", beam_len, "電装")
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for up in (1, -1):
            # 入隅は「横材の上面／下面」と「柱の内面」。腕は横材の長手（Y）と
            # 柱の長手（Z）へ伸びるので、穴は両方とも溝の芯に乗る。
            loc = (Pos(x_post_c, sy * abs(y_in), P.DISP_Z + up * P.EXT_W / 2)
                   * Rot(0, 0, -90 * sy) * Rot(90 * up, 0, 0))
            bnm = f"brk_disp_{sd}{'u' if up > 0 else 'd'}"
            put(parts, loc * L.bracket(), bnm,
                to=("disp_beam", f"post_rear_{sd}"), how="TSLOT",
                note="1-M5（腕 1 枚に 1 本）")
            LEDGER.add("表示器横材 ブラケット HBLFSN5", 0.012,
                       "MISUMI カタログ", "電装")
            bracket_screws(parts, loc, bnm, ("disp_beam", f"post_rear_{sd}"))

    # 取付枠。⚠ **平板を横材の -X 面に当てて 2-M5 で留める**という宣言だったが、
    #   ねじは 1 本も描かれていなかった。しかも描けない: 溝は横材の -X 面に
    #   1 条しか無く、そこは**画面（|Y|<=97.5・Z 419..541）の真裏**なので、
    #   頭が画面の中に入る。枠を画面より 20mm 広げて逃がすと、枠が画面より
    #   目立つ幅（245mm）になる。
    #   → 枠を**横材をくわえるコの字**にする（PETG は 3D プリントなので
    #     一体で出せる）。ねじは横材の**上下の溝**へ入れるので画面と干渉せず、
    #     上下 2 点で掴むから「面の摩擦だけで倒れを止める」形でもなくなる。
    x_frame = F.face("disp_beam", "x", -1)            # -285.4（横材の -X 面）
    frame = Pos(x_frame - P.DISP_FRAME_T / 2, 0.0, P.DISP_Z) \
        * Box(P.DISP_FRAME_T, P.DISP_W + 10.0, P.DISP_H + 10.0, align=CTR)
    for up in (1, -1):
        frame += Pos(x_frame + P.EXT_W / 2, 0.0,
                     P.DISP_Z + up * (P.EXT_W + P.DISP_FRAME_T) / 2) \
            * Box(P.EXT_W, P.DISP_CLAW_Y, P.DISP_FRAME_T, align=CTR)
    put(parts, L.mat(frame, "PETG"),
        "disp_frame", to="disp_beam", how="TSLOT",
        note="4-M5 溝ナット（横材の上下の溝）")
    LEDGER.add("表示器取付枠 PETG t4（コの字）", 0.065, "体積概算", "電装")
    # 爪 → 横材の溝。⚠ 位置は**横材の芯**から取る。爪の Y は自由に選べるが、
    #   X は溝の芯（柱・横材の中心）でなければナットに入らない。
    for up in (1, -1):
        z_seat = P.DISP_Z + up * (P.EXT_W / 2 + P.DISP_FRAME_T)
        for i, sy in enumerate((1, -1)):
            ray = (x_post_c, sy * P.DISP_CLAW_PITCH / 2, z_seat, 0.0, 0.0, float(up))
            put_screw(parts,
                      Pos(x_post_c, sy * P.DISP_CLAW_PITCH / 2, z_seat)
                      * Rot(*_axis_rot((0.0, 0.0, float(up))))
                      * L.screw_tnut(5, P.DISP_FRAME_T),
                      f"scr_disp_claw_{'u' if up > 0 else 'd'}{i}",
                      to=("disp_frame", "disp_beam"), how=("THRU", "TNUT"),
                      kind="CAP", size=5,
                      length=L.screw_len(P.DISP_FRAME_T, 6.0),
                      extras=(("TNUT", 5),),
                      note="1-M5 + 後入れナット", tool=ray)

    # ⚠ **筐体と画面を 1 個の箱で描かない。** 195×122×14 の箱ひとつだと、
    #   材質が 1 つしか付かないので「画が出る面」がどこにも無い。ビューアでも
    #   プロモでも白い板が 1 枚立っているだけになり、表示器に見えない
    #   （実際そうなっていた）。実品どおり **ベゼル（筐体）と有効表示領域を
    #   分ける**。8 型 16:10 の画は 172.3×107.7 で、まわりに左右 11.4 /
    #   上下 7.2 のベゼルが残る。
    # ⚠ 色は**ブーリアンのあとで**付ける。`L.mat(...) - Box(...)` と書くと
    #   `L.mat()` が入れた色は削るときに落ち、STEP で無色（＝材質不明）になる。
    x_disp = x_frame - P.DISP_FRAME_T
    x_face = x_disp - P.DISP_T                        # 画面側（-X）の面
    scr_at = Pos(x_face + P.DISP_SCR_T / 2, 0.0, P.DISP_Z)
    bezel = Pos(x_disp - P.DISP_T / 2, 0.0, P.DISP_Z) \
        * Box(P.DISP_T, P.DISP_W, P.DISP_H, align=CTR)
    bezel -= scr_at * Box(P.DISP_SCR_T, P.DISP_SCR_W, P.DISP_SCR_H, align=CTR)
    put(parts, L.mat(bezel, "PC"),
        "disp_panel", to="disp_frame", how="BOLT", note="4-M3")
    LEDGER.add("8インチ表示器 Elecrow SH080", 0.280, "メーカー公表値", "電装")

    # 前面ガラス（画が出る面）。ベゼルに彫った座へ落とし込む＝インロー。
    # ⚠ 台帳には載せない。280g は**表示器 1 台ぶんのカタログ値**で、
    #   ベゼル側に計上済み。ここで足すと同じ物を二重に数える。
    put(parts, L.mat(scr_at * Box(P.DISP_SCR_T, P.DISP_SCR_W, P.DISP_SCR_H,
                                  align=CTR), "SCREEN"),
        "disp_screen", to="disp_panel", how="PRESS",
        note="有効表示領域 172.3×107.7（1280×800）")

    # --- 配線 ---------------------------------------------------------------
    # ⚠ 電源は**ブレーカーの負荷側**から取る。電池側から取ると、非常停止でも
    #   遠隔停止でも計算機の電源が切れない（規定 7.2.4）。
    put(parts, L.cable_to([
        (-350.0, -15.0, F.face("breaker", "z", 1)),
        (-350.0, -60.0, 215.0), (-320.0, -120.0, 215.0),
        # ⚠ 降ろす X を -272 にすると表示器へ行く束を貫く。-290 で降ろす。
        # ⚠ 入力は DC-DC の **+Y 面**から入れる（出力は -Y 面）。同じ側へ
        #   降ろすと、出力側の 1 本目の線分と 15mm³ 交差する。
        (-290.0, -95.0, 215.0),
        (-290.0, F.face("pc_dcdc", "y", 1),
         (F.face("pc_dcdc", "z", 1) + F.face("pc_dcdc", "z", -1)) / 2)], [], 6.0),
        "cab_pc_in", to=("breaker", "pc_dcdc"), how="CONNECT",
        note="ブレーカー負荷側 → DC-DC 入力")
    LEDGER.add("計算機 電源配線", 0.030, "概算", "配線")

    # ⚠ デッキの上で空いている高さの帯は **Z 170..190 の 20mm だけ**
    #   （上は電源幹線 Z 194..212、下は砲塔・LiDAR 系 Z 144..156 と機器そのもの）。
    # ⚠ 入力と出力を**同じ面から出さない**。どちらも天面にすると、端点の
    #   球どうしが 131mm³ 重なる（配線どうしの重なりは BUNDLE でしか許されない
    #   が、入力と出力は別の回路なので束ねるとは言えない）。出力は -Y 面から。
    put(parts, L.cable_to([
        # ⚠ Z は**相手の面から取る**。154.25 と手で書いたら DC-DC の天面
        #   （149.5）より 4.75mm 高く、「離れ」になった。
        (-290.0, F.face("pc_dcdc", "y", -1),
         (F.face("pc_dcdc", "z", 1) + F.face("pc_dcdc", "z", -1)) / 2),
        (-262.0, -60.0, 175.0), (-230.0, 20.0, 175.0), (-210.0, 45.0, 158.0),
        (-200.0, F.face("pc_mini", "y", -1), 150.0)], [], 6.0),
        "cab_pc_pwr", to=("pc_dcdc", "pc_mini"), how="CONNECT",
        note="DC-DC → 計算機 12V")

    # ⚠ 「回路と USB 接続」の相手は**制御基板（mcu）**。ESC（C620/C610）は
    #   CAN でしか喋らないので USB は刺さらない。
    # ⚠ Y=20 で寄せるとバッテリーまで 0.25mm。Y=25 にする。
    put(parts, L.cable_to([
        (F.face("pc_mini", "x", 1), 100.0, 150.0),
        (-90.0, 100.0, 150.0), (-70.0, 40.0, 150.0),
        (F.face("mcu", "x", -1), 25.0, 150.0)], [], 4.5),
        "cab_usb_mcu", to=("pc_mini", "mcu"), how="CONNECT",
        note="USB（計算機 ↔ 制御基板）")

    # ⚠ 柱に沿わせる X は**柱の中心から外す**。押出材の面の中央には溝があり、
    #   中心に置いた束は溝の底まで届かず「離れ」になる。溝口の外へ寄せる。
    put(parts, L.cable_to([
        (F.face("pc_mini", "x", -1), 100.0, 150.0),
        (-275.0, 100.0, 150.0), (-275.0, 60.0, 178.0),
        (-275.0, -300.0, 178.0),
        (-281.5, -337.5, 215.0), (-281.5, -337.5, P.DISP_Z - 100.0),
        # ⚠ 画面へは**真下から垂直に**入れる。斜めに寄せると底面の角を
        #   703mm³ 削る（斜めの線分が板の内側を通る）。
        (x_disp - P.DISP_T / 2, -60.0, P.DISP_Z - 100.0),
        (x_disp - P.DISP_T / 2, -60.0, F.face("disp_panel", "z", -1))], [], 5.0),
        "cab_usb_disp", to=("pc_mini", "disp_panel", "post_rear_R"),
        how=("CONNECT", "CONNECT", "ROUTE"),
        note="USB-C 1 本（映像＋給電）。後柱の溝口の外に沿わせる")
    LEDGER.add("計算機 USB 配線", 0.040, "概算", "配線")
    return parts


# ---------------------------------------------------------------------------
# 計測輪（オドメトリ輪）1 組 — 双列オムニ + 縦リニアガイド + 輪ゴム予圧
# ---------------------------------------------------------------------------
# ローカル系（yaw=0）で組み、最後にまとめて回す。
#   ・車輪は**ローカル +X 方向に転がる**（車軸はローカル Y）
#   ・支持側 si（+1 なら +Y 側）に 軸受・マグネット・基板が並ぶ（片持ち）
#   ・上下ガイドは車輪の**転がり方向の後ろ**（ローカル +X 側）に立てる
#
# Y の積み上げ（車輪の中心面から支持側 si へ d mm）:
#     -12 .. 25   取付板（固定側・法線は X なので Y は「板の幅」）
#     -10 .. 10   オムニホイール φ50 × W20
#      -7 .. 13   LM ブロック（レール中心 d=3 の ±10）
#      10 .. 13   変換ハブのつば（オムニへ 3-M3）
#      14 .. 17   キャリッジ金具の web（軸受ボスを持つ板）
#      14 .. 22   軸受ボス φ14（MR105ZZ ×2 を突き合わせ）
#      23 .. 25   マグネット φ6×2（軸端に接着）
#      26 ..27.6  エンコーダ基板（マグネットとのエアギャップ 1.0mm）
#      19 .. 23   輪ゴムのピン座（固定側・可動側とも）
#
# X の積み上げ（車軸を 0 とし、ガイド側へ）:
#       0         車軸 / 軸受ボス / マグネット / 基板
#      25         オムニの外周 ＝ LM ブロックの上面（同じ値になったのは偶然）
#      21 .. 25   キャリッジ金具のフランジ（ブロック上面へ 4-M3）
#    28.5 .. 35   LM レール（内面は車輪外周から 3.5mm 逃げる）
#      35 .. 41   取付板 t6（レール取付面 ＝ 板の内面 35）
#
# ⚠ **ガイドを車輪の真上には置けない。** 真上に置くと、取付板の下端は
#   「車輪がいちばん上がったとき（+8mm）の頂点 Z 58」より上にしか
#   下ろせない。骨格の下面が Z 115 なので板は 54mm しか残らず、
#   ブロック（38.8）+ ストローク（±8）+ レール端の逃げが入らない。
#   転がり方向の後ろへ 35mm 逃がすと、板は Z 40 まで下ろせる。
#
# ⚠ **左右 2 輪は X=-350 のままでは組めない。**（もとは -350 だった）
#   車輪幅が 12 → 20 に増えたぶん、支持側の積み上げが内側へ寄る。
#   エンコーダ基板の背面は |Y|=317 まで来るが、そこはメカナム
#   （|Y| 265.5..325 / X ±250..350 / Z 0..100）の中。**幅を削っても
#   解けない**（外側はスカート内面 |Y|=360 まで 5mm しかない）ので、
#   車輪ごとメカナムの X 帯の外（X=-180）へ移した。
#   ⚠ 移しても**オドメトリの式は 1 つも変わらない**。転がり方向 +X の輪が
#     測るのは v_i = vx - ω·y_i で、x_i は式に現れない。効くのは 2 輪の
#     Y 間隔（700mm のまま）だけ。x を使うのは 90° 傾けた中央輪だけ。
ODO_AXLE_Z = P.ODO_OMNI_OD / 2   # 25.0 車軸高さ。接地点が z=0 になる
ODO_PLATE_T = 6.0        # 取付板 t6（PETG）。レールは M3 通しボルトで留める
ODO_ARM_T = 4.0          # キャリッジ金具のフランジ板厚（ブロックへの座）
ODO_WEB_T = 3.0          # キャリッジ金具の web の板厚
ODO_HUB_FLG_T = P.ODO_HUB_FLG_T     # 変換ハブのつば（形も皿もみも同じ値）
# ⚠ 積み上げの数字を**そのまま書かない**。車輪幅や軸受幅を変えたときに
#   1 か所だけ直し忘れると、ハブが web を擦る／軸受がボスからはみ出す、
#   といった形で静かに壊れる。寸法から積み上げる。
ODO_D_HUB0 = P.ODO_OMNI_W / 2                   # 10.0 オムニの内側面
ODO_D_HUB1 = ODO_D_HUB0 + ODO_HUB_FLG_T         # 13.0 つばの内面
ODO_D_WEB0 = ODO_D_HUB1 + 1.0                   # 14.0 web の外面（すきま 1mm）
ODO_D_WEB1 = ODO_D_WEB0 + ODO_WEB_T             # 17.0 web の内面
ODO_D_BOSS1 = ODO_D_WEB0 + 2 * P.ODO_BRG_W      # 22.0 軸受ボスの内端
ODO_D_MAG0 = ODO_D_BOSS1 + 1.0                  # 23.0 マグネットの外面
ODO_D_ENC = ODO_D_MAG0 + P.ODO_MAG_T + 1.0      # 26.0 基板の外面（ギャップ 1）
# レール断面の中心。**車輪の中心面より内側**へ 3mm 寄せる。こうすると
# ブロックの外縁（d=-7）がスカート内面から 8mm 離れる（車輪の外面が 5mm）。
ODO_D_RAIL = 3.0
ODO_D_BAND = 21.0        # 輪ゴムのピン中心。ブロックの内縁（13）の外で、
                         # web（14..17）とも エンコーダ台（17..26）とも
                         # X か Z で外れている位置
ODO_D_PLATE0 = -12.0     # 取付板の外側縁。**スカート内面まで 3mm**。
                         # ここを詰めないと、骨格（幅 20）の下面に取れる
                         # 留め代が 10mm しか残らず M5 の座が乗らない
ODO_D_PLATE1 = 25.0      # 取付板の内側縁
# --- ガイド側 X（車軸を 0 とする）------------------------------------------
ODO_X_PLATE0 = 35.0                             # 取付板の内面 ＝ レール取付面
ODO_X_PLATE1 = ODO_X_PLATE0 + ODO_PLATE_T       # 41.0
ODO_X_BLOCK0 = ODO_X_PLATE0 - P.ODO_LM_BLOCK_H  # 25.0 ブロック上面
ODO_X_FLANGE0 = ODO_X_BLOCK0 - ODO_ARM_T        # 21.0 フランジの外面
ODO_X_PIN = ODO_X_FLANGE0 - 4.0                 # 17.0 輪ゴムのピン中心
ODO_X_WEB1 = ODO_X_BLOCK0                       # 25.0 web はフランジまで通す
ODO_X_WEB0 = -9.0                               # 車軸より先へ 9mm（ボスの座）
# --- Z ---------------------------------------------------------------------
ODO_BLK_Z0 = 58.0                               # 中立でのブロック下端
ODO_BLK_Z1 = ODO_BLK_Z0 + P.ODO_LM_BLOCK_L      # 96.8
ODO_RAIL_Z0 = ODO_BLK_Z0 - P.ODO_STROKE - 5.0   # 45.0
ODO_RAIL_Z1 = ODO_RAIL_Z0 + P.ODO_LM_RAIL_L     # 109.0
ODO_PLATE_Z0 = 40.0
ODO_STOP_LO = ODO_BLK_Z0 - P.ODO_STROKE         # 50.0 下端ストッパの当たり面
ODO_STOP_HI = ODO_BLK_Z1 + P.ODO_STROKE         # 104.8 上端ストッパの当たり面
ODO_POST_Z = ODO_STOP_LO - 6.0                  # 44.0 固定側ピン（座の中）
ODO_HOOK_Z = ODO_POST_Z + P.ODO_BAND_SPAN       # 92.0 可動側ピン
# ⚠ **レールの端をストッパにしてはいけない。** MGN のブロックは端から
#   出るとボールが落ちる。機体を持ち上げると輪ゴムがキャリッジを下まで
#   引くので、**下端ストッパは必須**。もとの構成では「ばね端末を座グリへ
#   押し込んで引張にも効かせる」ことで代用していたが、輪ゴムは押せない
#   ので代用が効かない。ここでは輪ゴムの固定側ピン座の上面（Z 50）が
#   そのまま下端ストッパになっている（部品を増やさずに済む）。
assert ODO_STOP_LO - 5.0 >= ODO_RAIL_Z0, "下端でブロックがレール端に届く"
assert ODO_STOP_HI + 4.0 <= ODO_RAIL_Z1, "上端でブロックがレール端に届く"
assert ODO_HOOK_Z + P.ODO_STROKE < ODO_STOP_HI, "上端で輪ゴムのピンが座に当たる"


def _odo_unit(parts, k, x, y, host, si, yaw=0.0):
    """計測輪 1 組（オムニ / 軸受 / エンコーダ / リニアガイド / 輪ゴム）。

    `(x, y)` は接地する車輪の中心、`si` は支持金具を置く側（+1 なら +Y 側）、
    `yaw` は車輪の**転がり方向**を Z まわりに回す角（0 / ±90 のみ）。

    ⚠ **3 輪とも同じ向きに置いてはいけない。** 平面の運動は (vx, vy, ω) の
      3 自由度で、1 輪が測るのは「その輪の転がり方向の速度」1 つだけ。
      向きが全部同じだと、3 本の式が張るのは
        v_i = vx - ω·(車輪の Y 位置)
      という **vx と ω の 2 次元**しかなく、vy の係数がどの式でも 0 になる。
      連立しても vy は決して出てこない（行列のランクが 2）。メカナムは
      横行が主役なので、これでは補正したい成分がまるごと落ちる。
      → 1 輪を 90° 傾けて **vy の係数を立てる**。ランクが 3 になって
        (vx, vy, ω) が解ける。これが 3 輪オドメトリの前提そのもの。

    ⚠ **オムニにしたからこそ「転がり方向だけを測る」が実機で成り立つ。**
      ウレタンの丸タイヤだと、横行のとき 3 輪とも横へ引きずられる。
      摩擦円は 1 つしかないので、横に滑った瞬間に**転がり方向の把持も
      失う**＝エンコーダの読みがそのぶん嘘になる。オムニは横成分を
      ローラーの転がりで逃がすので、縦横の取り合いが起きない。

    実装は**ローカル系で組んでから最後に回す**。座標を回した値で書き直すと、
    積み上げ表（`ODO_D_*` / `ODO_X_*`）との対応が読めなくなる。
    """
    yaw = float(yaw)
    # 車輪中心まわりの回転。ローカル系（yaw=0 のときの座標）→ ワールド
    W = Pos(x, y, 0) * Rot(0, 0, yaw) * Pos(-x, -y, 0)

    def hface(axis, sgn):
        """相手部材 `host` の面座標を**ローカル系**で読む。

        ⚠ 回すのは計測輪だけで、相手（押出材）は回らない。ワールドの面を
          そのまま使うと、90° 回した側で「留め代の幅」も「折る向き」も
          別の軸の値を読んでしまう。ローカルへ引き戻してから使う。
        """
        if axis == "z" or abs(yaw) < 1e-9:
            return F.face(host, axis, sgn)
        s = 1.0 if yaw > 0 else -1.0        # +90: 局所+X=世界+Y / -90: 局所+X=世界-Y
        if axis == "x":
            return x + s * (F.face(host, "y", int(s * sgn)) - y)
        return y - s * (F.face(host, "x", int(-s * sgn)) - x)

    def yd(d):
        """車輪の中心面から si 方向へ d mm の Y 座標。"""
        return y + si * d

    def yspan(d0, d1):
        """d0..d1 の帯を占める Box の (Y 中心, Y 幅)。"""
        return yd((d0 + d1) / 2), abs(d1 - d0)

    def xspan(v0, v1):
        """車軸から v0..v1 の帯を占める Box の (X 中心, X 幅)。"""
        return x + (v0 + v1) / 2, abs(v1 - v0)

    z_ax = ODO_AXLE_Z
    y_web = yd((ODO_D_WEB0 + ODO_D_WEB1) / 2)
    y_rail = yd(ODO_D_RAIL)
    y_band = yd(ODO_D_BAND)
    x_pin = x + ODO_X_PIN
    # ハブと軸は「回すと +Z が支持側へ向く」向きに置く。
    # Rot(90,0,0) は +Z を -Y へ送るので、si=+1 の側だけ反転する。
    rot_ax = Rot(-si * 90.0, 0, 0)

    # --- 車体側 取付板（固定）------------------------------------------
    # ⚠ 板の端面（6×37 = 222mm²）で押出材に留めることはできない。
    #   上端を 90° 曲げて、骨格の下面へ**板面で**当てる。
    # ⚠ 留め代は**相手の下面のうち板の幅と重なっている帯**にしか座れない。
    #   板（37 幅）のほうが押出材（20 幅）より広いので、重なりを取って
    #   両側 2mm 逃がす。90° 回した輪でも `hface` が同じ帯を返す。
    z_top = F.face(host, "z", -1)
    plate_yc, plate_w = yspan(ODO_D_PLATE0, ODO_D_PLATE1)
    plate_xc, plate_t = xspan(ODO_X_PLATE0, ODO_X_PLATE1)
    tab_y0 = max(min(yd(ODO_D_PLATE0), yd(ODO_D_PLATE1)), hface("y", -1)) + 2.0
    tab_y1 = min(max(yd(ODO_D_PLATE0), yd(ODO_D_PLATE1)), hface("y", 1)) - 2.0
    # 留め代は**車輪の側へ**折る。逆へ折ると 44mm ぶんが機体の外へ向かって
    # 伸びる（左右輪では後方の骨格の端、中央輪では横材の端をはみ出す）。
    tab_xc, tab_w = xspan(ODO_X_PLATE0 + 1.0, ODO_X_PLATE0 + 1.0 - 44.0)
    shelf_yc, shelf_w = yspan(ODO_D_BAND - 2.0, ODO_D_BAND + 2.0)
    shelf_xc, shelf_l = xspan(ODO_X_PIN + 2.0, ODO_X_PLATE0 + 1.0)
    brk = (Pos(plate_xc, plate_yc, (ODO_PLATE_Z0 + z_top) / 2)
           * Box(plate_t, plate_w, z_top - ODO_PLATE_Z0, align=CTR)
           # 留め代。板の中へ 1mm 入れて**確実に 1 ソリッドに融合**させる
           # （面が接するだけだと「分断」で出ることがある）
           + Pos(tab_xc, (tab_y0 + tab_y1) / 2,
                 F.seat_on(host, "z", -1, ODO_PLATE_T))
           * Box(tab_w, abs(tab_y1 - tab_y0), ODO_PLATE_T, align=CTR)
           # 輪ゴムの固定側ピン座。**上面（Z 50）がそのまま下端ストッパ**
           + Pos(shelf_xc, shelf_yc, (ODO_STOP_LO + ODO_POST_Z - 6.0) / 2)
           * Box(shelf_l, shelf_w, ODO_STOP_LO - (ODO_POST_Z - 6.0), align=CTR)
           # 上端ストッパ（キャリッジのフランジ上端が当たる）
           + Pos(shelf_xc, shelf_yc, (ODO_STOP_HI + ODO_STOP_HI + 7.0) / 2)
           * Box(shelf_l, shelf_w, 7.0, align=CTR)
           # 固定側の輪ゴムピン φ4。**座と一体で印刷する**（板をレール面
           # 下向きに寝かせて刷ると、ピンは Z 方向に立つのでサポート不要）
           # ⚠ 根元は座の中へ 2mm 入れる。面で接するだけだとブーリアンが
           #   融合せず、`assembly_check` に「分断」で出る。
           + Pos(x_pin + 1.0, y_band, ODO_POST_Z) * Rot(0, 90, 0)
           * Cylinder(P.ODO_BAND_PIN_D / 2, 10.0, align=CTR))
    # レール取付ボルト（M3 通し）。レール長 64 / ピッチ 20 の 3 か所
    for dz in (-P.ODO_LM_RAIL_PITCH, 0.0, P.ODO_LM_RAIL_PITCH):
        brk -= Pos(plate_xc, y_rail, (ODO_RAIL_Z0 + ODO_RAIL_Z1) / 2 + dz) \
            * Rot(0, 90, 0) * Cylinder(1.7, plate_t + 2, align=CTR)
    # 肉抜き。レール帯（d -1.5..7.5）とピン座（d 19..23）のあいだを抜く
    win_yc, win_w = yspan(ODO_D_RAIL + 8.0, ODO_D_BAND - 3.0)
    brk -= Pos(plate_xc, win_yc, (ODO_RAIL_Z0 + ODO_RAIL_Z1) / 2) \
        * Box(plate_t + 2, win_w, ODO_RAIL_Z1 - ODO_RAIL_Z0 - 14.0, align=CTR)
    put(parts, W * L.mat(brk, "PETG"), f"odo_brk{k}", to=host, how="TSLOT",
        note="2-M5 溝ナット")
    LEDGER.add_solid(f"計測輪{k} 取付板 PETG", brk, "PETG", "センサー")

    # --- リニアガイド（レール固定 / ブロック可動）------------------------
    # ⚠ レールは板に**通しボルト**で留める。PETG に M3 をタップで立てても
    #   4kgf も掛からないうちに舐める（板厚 6 では熱圧入インサートも
    #   入らない。インサートは埋め込み長 5.7mm で、下穴の底が抜ける）。
    # ⚠ **姿勢は si に依らない。** レールの取付面は「板の内面」＝つねに
    #   ローカル +X を向く。si（軸受を +Y に置くか -Y に置くか）で回すと、
    #   片側だけレールの長手が Y を向いて、幅 64mm の板がスカートを
    #   突き抜ける（実際そうなった）。ライブラリは「幅 X・高さ Y・長手 Z /
    #   取付面 +Y」なので、Z まわりに -90° 回すだけでよい。
    x_rail = x + ODO_X_PLATE0 - P.ODO_LM_RAIL_H / 2
    put(parts, W * Pos(x_rail, y_rail, (ODO_RAIL_Z0 + ODO_RAIL_Z1) / 2)
        * Rot(0, 0, -90.0) * L.odo_lm_rail(),
        f"odo_rail{k}", to=f"odo_brk{k}", how="BOLT", note="3-M3 通し + ナット")
    LEDGER.add(f"計測輪 LM ガイド レール MGN9 L{P.ODO_LM_RAIL_L:.0f}",
               P.ODO_LM_RAIL_KG_M * P.ODO_LM_RAIL_L / 1000.0, "カタログ", "センサー")
    put(parts, W * Pos(x_rail, y_rail, (ODO_BLK_Z0 + ODO_BLK_Z1) / 2)
        * Rot(0, 0, -90.0) * L.odo_lm_block(),
        f"odo_blk{k}", to=f"odo_rail{k}", how="SLIDE", note="MGN9C（摺動）")
    LEDGER.add("計測輪 LM ガイド ブロック MGN9C", P.ODO_LM_BLOCK_MASS,
               "カタログ", "センサー")

    # --- キャリッジ金具（ブロック ⇄ 車軸）--------------------------------
    # ⚠ **フランジ 1 枚では車輪を持てない。** ブロックの上面と車軸は
    #   21mm 離れていて、そのあいだを 4mm の板で渡すと、7N の接地反力で
    #   板が曲がって車輪が首を振る。web（縦板）を車軸まで通し、
    #   フランジと L 字に組んで曲げをせん断で受ける。
    fl_xc, fl_t = xspan(ODO_X_FLANGE0, ODO_X_BLOCK0)
    fl_yc, fl_w = yspan(ODO_D_RAIL - P.ODO_LM_BLOCK_W / 2, ODO_D_BAND + 2.0)
    web_xc, web_l = xspan(ODO_X_WEB0, ODO_X_WEB1)
    arm = (Pos(fl_xc, fl_yc, (ODO_BLK_Z0 + ODO_BLK_Z1) / 2)
           * Box(fl_t, fl_w, ODO_BLK_Z1 - ODO_BLK_Z0, align=CTR)
           + Pos(web_xc, y_web, (z_ax - 12.0 + ODO_BLK_Z1) / 2)
           * Box(web_l, ODO_WEB_T, ODO_BLK_Z1 - z_ax + 12.0, align=CTR)
           # 可動側の輪ゴムピン φ4（フランジと一体。根元は 2mm 埋める）
           + Pos(x_pin + 1.0, y_band, ODO_HOOK_Z) * Rot(0, 90, 0)
           * Cylinder(P.ODO_BAND_PIN_D / 2, 10.0, align=CTR))
    # 軸受ボス φ14 の座（しめしろ 0.05）
    arm -= Pos(x, y_web, z_ax) * Rot(90, 0, 0) * Cylinder(6.95, ODO_WEB_T + 2,
                                                          align=CTR)
    # web の肉抜き。車軸まわり（ボスの座）と上端（フランジとの角）は残す
    arm -= Pos(x + 6.0, y_web, (z_ax + ODO_BLK_Z1) / 2 + 2.0) \
        * Box(20.0, ODO_WEB_T + 2, ODO_BLK_Z1 - z_ax - 26.0, align=CTR)
    put(parts, W * L.mat(arm, "PETG"), f"odo_arm{k}", to=f"odo_blk{k}",
        how="BOLT", note="4-M3（ブロック上面のタップ）")
    LEDGER.add_solid(f"計測輪{k} キャリッジ金具 PETG", arm, "PETG", "センサー")

    # --- 軸受（ボス + 玉軸受 2 個）--------------------------------------
    # ⚠ 板 t3 の穴だけでは片持ちの軸を支えられない（曲げを受けられず、
    #   車輪が 1° 傾くだけで接地位置がずれる＝計測が狂う）。ボスを立てて
    #   軸受を **幅 2 個ぶん離して** 入れ、モーメントを偶力で受ける。
    boss_yc, boss_len = yspan(ODO_D_WEB0, ODO_D_BOSS1)
    boss = (Cylinder(7.0, boss_len, align=CTR)
            # 内径は軸受外径 -0.1（しめしろ）。ちょうどにすると接線になる
            - Cylinder(P.ODO_BRG_OD / 2 - 0.05, boss_len + 2, align=CTR))
    put(parts, W * L.mat(Pos(x, boss_yc, z_ax) * Rot(90, 0, 0) * boss, "A5052"),
        f"odo_boss{k}", to=f"odo_arm{k}", how="PRESS", note="φ14 圧入")
    LEDGER.add_solid(f"計測輪{k} 軸受ボス A5052", boss, "A5052", "センサー")
    put(parts, W * Pos(x, yd(ODO_D_WEB0 + P.ODO_BRG_W / 2), z_ax) * Rot(90, 0, 0)
        * L.odo_bearing(),
        f"odo_brg{k}o", to=f"odo_boss{k}", how="PRESS", note="MR105ZZ")
    put(parts, W * Pos(x, yd(ODO_D_BOSS1 - P.ODO_BRG_W / 2), z_ax) * Rot(90, 0, 0)
        * L.odo_bearing(),
        f"odo_brg{k}i", to=(f"odo_boss{k}", f"odo_brg{k}o"), how="PRESS",
        note="MR105ZZ（同じ穴に突き合わせ）")
    LEDGER.add("計測輪 玉軸受 MR105ZZ", P.ODO_BRG_MASS, "カタログ", "センサー", qty=2)

    # --- 軸 / 変換ハブ / オムニホイール ----------------------------------
    shaft_yc, shaft_len = yspan(-P.ODO_OMNI_W / 2, ODO_D_MAG0)
    shaft = Cylinder(P.ODO_SHAFT_D / 2, shaft_len, align=CTR)
    put(parts, W * L.mat(Pos(x, shaft_yc, z_ax) * Rot(90, 0, 0) * shaft, "STEEL"),
        f"odo_shaft{k}", to=(f"odo_brg{k}o", f"odo_brg{k}i"), how="ROTATE",
        note="φ5 研磨軸")
    LEDGER.add_solid(f"計測輪{k} 軸 φ5", shaft, "STEEL", "センサー")
    hub = L.odo_hub()
    put(parts, W * Pos(x, y, z_ax) * rot_ax * hub,
        f"odo_hub{k}", to=f"odo_shaft{k}", how="PRESS", note="圧入 + 止めねじ")
    LEDGER.add_solid(f"計測輪{k} 変換ハブ PETG", hub, "PETG", "センサー")
    # ⚠ `odo_wheel{k}` は**唯一 床に触れる部品**。名前と中心座標
    #   （x, y, 25）と転動径 φ50 は据え置き、中身をオムニに替えた。
    put(parts, W * Pos(x, y, z_ax) * rot_ax * L.odo_omni(),
        f"odo_wheel{k}", to=f"odo_hub{k}", how="BOLT", note="3-M3（PCD22）")
    # ⚠ **嵌合（筒の中に筒）は `screw_place` に探させない。** 平らな接触面が
    #   無いうえ、すきま嵌め 0.1mm は内外判定の境界そのもので、実行ごとに
    #   「接触面に両方の材料がある場所が無い」になったりならなかったりする。
    #   位置はオムニの取付穴（PCD22）が決めているので計算で置く。
    flange_screws(parts, W * Pos(x, y, z_ax) * rot_ax
                  * Pos(0, 0, P.ODO_OMNI_W / 2),
                  # ⚠ **皿ねじ**。つばの外側 1mm 先が揺動アーム板なので、
                  #   六角穴付きの頭（3mm）はそのままアームに埋まる
                  #   （実測 18〜48mm³ ×9）。つば t3 なら皿もみ 1.5mm が入る。
                  f"odo_hub{k}", f"odo_wheel{k}", P.ODO_OMNI_PCD, 3, 3,
                  P.ODO_HUB_FLG_T, f"odo_hub{k}_", start=90.0, kind="FLAT")
    LEDGER.add("計測輪 双列オムニホイール φ50×W20", P.ODO_OMNI_MASS,
               "カタログ", "センサー")

    # --- エンコーダ（磁気式 AS5600 / 12bit）------------------------------
    # ⚠ 軸端のマグネットを読むので、**センサーは軸の延長線上**に来る。
    #   基板を web に直接ねじ止めすると、マグネットとのエアギャップが
    #   板厚ぶんしか取れず（AS5600 の推奨は 0.5〜3mm）、そもそも軸受ボスが
    #   邪魔で基板が座らない。台座で 9mm 逃がし、ボスとマグネットは
    #   台座の逃げ穴（φ16）の中を通す。
    put(parts, W * L.mat(Pos(x, yd(ODO_D_MAG0 + P.ODO_MAG_T / 2), z_ax)
                         * Rot(90, 0, 0)
                         * Cylinder(P.ODO_MAG_D / 2, P.ODO_MAG_T, align=CTR), "STEEL"),
        f"odo_mag{k}", to=f"odo_shaft{k}", how="PRESS", note="軸端に接着")
    LEDGER.add("計測輪 2極着磁マグネット φ6×2", 0.0005, "カタログ", "センサー")
    mnt_yc, mnt_t = yspan(ODO_D_WEB1, ODO_D_ENC)
    encmnt = (Box(20.0, mnt_t, 22.0, align=CTR)
              - Rot(90, 0, 0) * Cylinder(8.0, mnt_t + 2, align=CTR))
    put(parts, W * L.mat(Pos(x, mnt_yc, z_ax) * encmnt, "PETG"),
        f"odo_encmnt{k}", to=f"odo_arm{k}", how="BOLT", note="2-M3")
    LEDGER.add_solid(f"計測輪{k} エンコーダ台座 PETG", encmnt, "PETG", "センサー")
    put(parts, W * L.mat(Pos(x, yd(ODO_D_ENC + P.ODO_ENC_PCB[2] / 2), z_ax)
                         * Box(P.ODO_ENC_PCB[0], P.ODO_ENC_PCB[2],
                               P.ODO_ENC_PCB[1], align=CTR), "PCB"),
        f"odo_enc{k}", to=f"odo_encmnt{k}", how="BOLT", note="2-M3")
    LEDGER.add("計測輪 磁気エンコーダ基板 AS5600", P.ODO_ENC_MASS, "カタログ", "センサー")

    # --- 予圧の輪ゴム ----------------------------------------------------
    # ⚠ **接地荷重は「押し付け」ではなく「離れない」ぶんだけでよい。**
    #   もとの構成は 10.8N 押していたが、それは横へ引きずられても
    #   滑らないための摩擦を稼ぐためだった。オムニは横成分を転がりで
    #   逃がすので、その要求が消える。残るのは
    #     ・ばね下（オムニ 45g + キャリッジ 40g + ブロック 17g ≒ 0.13kg）が
    #       跳ねても離れないこと → 5G で 6.4N
    #     ・LM ブロックのシール抵抗（0.2〜0.4N）を越えて追従できること
    #   の 2 つだけ。#10 を 2 本で 7N 前後。**本数で調整する**設計にした
    #   （実機ではばね秤で測って 1〜3 本のあいだで合わせる）。
    # 車輪まわりのばね定数は 0.21N/mm（もとの構成は 1.08N/mm）。
    # ±8mm 動いても荷重は 7 ± 1.7N しか変わらないので、目地（1〜2mm）でも
    # 機体の傾き（±1°）でも浮かないし、押しすぎて転がり抵抗も増やさない。
    # ⚠ 本数を増やす向きは**ピンの軸方向（ローカル X）**。輪ゴム 1 本は
    #   ピンの上下に 2 条まわる（＝Y に 5.8mm 広がる）ので、Y にずらして
    #   並べるとゴムどうしが食い合う。ピンに沿って 1.6mm ピッチで重ねる。
    for i in range(P.ODO_BAND_N):
        put(parts, W * Pos(x_pin + (i - (P.ODO_BAND_N - 1) / 2) * 1.6, y_band,
                           (ODO_POST_Z + ODO_HOOK_Z) / 2)
            * L.odo_band(P.ODO_BAND_SPAN),
            f"odo_band{k}{i}", to=(f"odo_brk{k}", f"odo_arm{k}"), how="CLAMP",
            note=f"輪ゴム #{P.ODO_BAND_NO}（折長 {P.ODO_BAND_FLAT:.0f}）を "
                 f"{P.ODO_BAND_SPAN:.0f}mm に張る＝伸び率 "
                 f"{P.ODO_BAND_SPAN / P.ODO_BAND_FLAT - 1:.0%}")
    LEDGER.add(f"計測輪 予圧の輪ゴム #{P.ODO_BAND_NO}", P.ODO_BAND_MASS,
               "カタログ", "センサー", qty=P.ODO_BAND_N)
    LEDGER.add("計測輪 ねじ類（M5×2 / M3×12 + 溝ナット）", 0.022, "概算", "センサー")


def _lidar_leveler(parts, tag, brk, cx, cy, z_mount, base_x, top_x, plate_y,
                   notch=None, top_y=None):
    """LiDAR 1 台ぶんの **4 点レベリング座**を置き、可動板の名前を返す。

    LiDAR は**下向きに吊る**。可動板の下面に本体の取付面（＝ UST-20LX の
    「底面」）を当て、本体はそこからぶら下がる。調整ねじは**上から**入り、
    固定板を貫いて可動板のタップに入る。締めれば可動板ごと LiDAR が上がり、
    緩めればばねが押し下げる。

        base … 固定側（上）。ばかっ穴 4 つ。ねじの頭がここに座る
        top  … 可動側（下）。M4 タップ 4 つ。下面に LiDAR が付く

    ⚠ **ねじの頭が全部いちばん上に来る**のがこの向きの狙い。上に載せる形だと
      本体が調整ねじの脇に立つので、六角レンチが胴をかすめる。吊れば本体は
      板の下なので、上は完全に開いた面になる。

    なぜ要るか
    ----------
    2D LiDAR は走査面が水平でないと使えない。θ 傾くと距離 L の点は
    L·sinθ ずれるので、5m 先の壁では 1° = 87mm になる。切削と組立の公差
    だけでは 0.5〜1° 残るうえ、**残った傾きは図面からは分からない**
    （実測して直すしかない）。取付をねじ 4 本の締め代に置き換えると、
    現場で 0.076°/（1/12 回転）の粒度で追い込める。

    引数
    ----
    `tag`      … 部品名の接尾（front / rear / high）
    `brk`      … 固定板を支える相手（横材の上面 / 立板の上端面）
    `cx, cy`   … 調整ねじの座中心。**LiDAR の軸に合わせる**（4 点の作る
                 四角形の中に重心が入っていないと、ばねが片側だけ効く）
    `z_mount`  … LiDAR の取付面 Z（= 可動板の**下面**）
    `base_x` / `top_x` / `plate_y` … 板の (min, max)。X は左右で符号が
                 逆になるので、呼ぶ側は**世界座標のまま**渡してよい
                 （ここで並べ替える）
    `notch`    … base から抜く逃げ (x0, x1, y0, y1)。柱・脚をかわすときだけ

    ⚠ **調整ねじは LiDAR の胴（50 角）の外に出すこと。** 吊り下げなら胴は
      板の下なので上は開いているが、ねじが胴の真上に来ると、下から手を
      入れて本体を外すときに邪魔になる。四隅に散らすほうが素直。
    """
    x0b, x1b = sorted(base_x)
    x0t, x1t = sorted(top_x)
    y0, y1 = sorted(plate_y)
    y0t, y1t = sorted(top_y) if top_y is not None else (y0, y1)
    z_t0 = z_mount                                  # 可動板の下面 = LiDAR 取付面
    z_t1 = z_t0 + P.LIDAR_LVL_TOP_T
    z_b0 = z_t1 + P.LIDAR_LVL_GAP                   # 固定板の下面
    z_b1 = z_b0 + P.LIDAR_LVL_BASE_T
    # 板中央の逃げ穴。**配線を真上から通す**（吊り下げなので LiDAR の
    # コネクタ面は板のすぐ下にある）。穴が無いと経路が板の外を回るしかなく、
    # 板を傾けたときに配線が突っ張って**傾きを戻す方向の力**になる。
    hole = Cylinder(P.LIDAR_LVL_HOLE / 2, P.LIDAR_LVL_BASE_T + 20.0, align=CTR)

    def _plate(x0, x1, t, zc, yr):
        ya, yb = yr
        sh = Pos((x0 + x1) / 2, (ya + yb) / 2, zc) * L.plate(x1 - x0, yb - ya, t)
        sh = sh - Pos(cx, cy, zc) * hole
        if notch is not None:
            nx0, nx1, ny0, ny1 = notch
            sh = sh - Pos((nx0 + nx1) / 2, (ny0 + ny1) / 2, zc) * \
                Box(nx1 - nx0, ny1 - ny0, t + 4.0, align=CTR)
        return L.mat(sh, "A5052")

    base = _plate(x0b, x1b, P.LIDAR_LVL_BASE_T, z_b0 + P.LIDAR_LVL_BASE_T / 2,
                  (y0, y1))
    put(parts, base, f"lidar_lvl_base_{tag}", to=brk[0], how=brk[1],
        note=brk[2])
    LEDGER.add_solid(f"LiDAR レベリング座 固定板 A5052 t{P.LIDAR_LVL_BASE_T:.0f}",
                     base, "A5052", "センサー")
    top = _plate(x0t, x1t, P.LIDAR_LVL_TOP_T, z_t0 + P.LIDAR_LVL_TOP_T / 2,
                 (y0t, y1t))
    top_nm = f"lidar_lvl_top_{tag}"
    put(parts, top, top_nm, to=f"lidar_lvl_base_{tag}", how="ADJUST",
        note=f"4-M{P.LIDAR_LVL_SCREW} 調整ねじ + 圧縮ばね")
    LEDGER.add_solid(f"LiDAR レベリング座 可動板 A5052 t{P.LIDAR_LVL_TOP_T:.0f}",
                     top, "A5052", "センサー")

    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        px = cx + dx * P.LIDAR_LVL_PITCH_X / 2
        py = cy + dy * P.LIDAR_LVL_PITCH_Y / 2
        # ばねは 2 枚のあいだ。ねじと同軸に通して座から外れないようにする
        put(parts, Pos(px, py, (z_t1 + z_b0) / 2) * L.lvl_spring(),
            f"lidar_lvl_spr_{tag}{i}", to=(top_nm, f"lidar_lvl_base_{tag}"),
            how="PRESS",
            note=f"圧縮コイル φ{P.LIDAR_LVL_SPRING_OD:.0f}×"
                 f"{P.LIDAR_LVL_SPRING_FREE:.0f} k={P.LIDAR_LVL_SPRING_K:.0f}N/mm")
        # ⚠ 頭は**固定板の上面**に座り、先は可動板のタップへ入る。
        #   締めれば可動板ごと LiDAR が上がり、緩めればばねが押し下げる。
        #   呼び長さは「−3mm 下げても可動板の t6 に噛み続ける」で決める。
        put_screw(parts, Pos(px, py, z_b1) * L.screw(P.LIDAR_LVL_SCREW,
                                                     P.LIDAR_LVL_SCREW_LEN),
                  f"scr_lvl_{tag}{i}", to=(f"lidar_lvl_base_{tag}", top_nm),
                  how=("THRU", "SCREW_IN"),
                  kind="CAP", size=P.LIDAR_LVL_SCREW,
                  length=P.LIDAR_LVL_SCREW_LEN,
                  # 戻り止めは可動板の下に出た先端へナット（下から手が入る）
                  extras=(("HEXNUT", P.LIDAR_LVL_SCREW),),
                  note=f"4-M{P.LIDAR_LVL_SCREW} 調整ねじ + 戻り止めナット",
                  tool=(px, py, z_b1, 0.0, 0.0, 1.0))
    LEDGER.add(f"LiDAR レベリング座 圧縮ばね φ{P.LIDAR_LVL_SPRING_OD:.0f}×"
               f"{P.LIDAR_LVL_SPRING_FREE:.0f}",
               P.LIDAR_LVL_SPRING_MASS, "カタログ", "センサー", qty=4)
    return top_nm


def _sensors():
    parts = []
    # 厚み計測ホールセンサー ×2（重送検知と兼用）。
    # ピックローラーの揺動アーム変位から雑巾1枚の厚みを読み、圧縮率→スリップ率→
    # 初速のずれを予測して、ニップ到達までの 0.33 秒でローラー回転数を補正する。
    # 誤差バジェット上、分解能 0.2mm で命中率 27%→49% に上がる（それ以上は飽和）。
    # ⚠ アームの下端はホッパー前壁の上端(HOP_TOP_Z)に載せること。
    #   以前は z 719..745 にあり、前壁の 712 から 7mm 浮いていた。
    #   センサー本体もアームから 1mm 離れていて、二重に浮いていた。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        arm_h = 60.0
        # ⚠ フォークの根元バー（X -41..-7、Z 763..784）と当たる。
        #   アームはホッパー前壁の -X 側に付ける。
        # ⚠ フォークの根元バー（|Y|<=280）とシンギュレータ軸（|Y|<=310）を
        #   避ける。ホッパー側壁の内面（|Y|=310）に寄せる。
        # ⚠ ヒンジブロック（|Y| 288..312）とシンギュレータ軸（|Y|<=310）の
        #   両方を避ける帯は **|Y| 265..288** しかない（中心 276）。
        # ⚠ ホッパー上部の |Y| 占有を数え上げると
        #     ピックローラー  0±15 / 130±15 / 265±15
        #     ヒンジブロック  288..312
        #     シンギュレータ軸 <=310、ホッパー側壁 310..314
        #   空き帯は 15..115（100mm）と 145..250（105mm）と 280..288（8mm）。
        #   幅 3 のアームを置くなら 145..250 の中央 198 が最も余裕がある。
        # ⚠ アームは前壁（X -15..-11、法線 X）に**板面で**当てる。
        #   板厚を Y に取ると壁とは線でしか当たらない（12mm²）。
        # ⚠ シンギュレータ軸は X -14..-6（中心 HOP_X1+5、φ8）を **|Y| 全域**に
        #   走る。前壁の外面（-11）に貼ると必ず軸の X 帯に入るので、
        #   アームは軸の**下**（Z<=714）に収める。
        put(parts, L.mat(Pos(P.HOP_X1 + P.HOP_DANPLA_T + 1.5, sy * 198.0,
                             P.HOP_TOP_Z - arm_h / 2 + 2.0)
                         * L.plate(3.0, 40.0, arm_h), "A5052"),
            f"thk_arm_{sd}", to="hop_front", how="RIVET", note="2-φ3.2")
        # ホールセンサーはアームの -X 面に貼る。座面はセンサーの面積ぶん取る
        # センサーもアームの +X 面に貼り、軸（Z 716..724）より下に収める
        put(parts, L.mat(Pos(P.HOP_X1 + P.HOP_DANPLA_T + 10.5, sy * 198.0,
                             P.HOP_TOP_Z - 8.0)
                         * Box(15.0, 24.0, 20.0, align=CTR), "PCB"),
            f"thk_hall_{sd}", to=f"thk_arm_{sd}", how="BOLT", note="2-M2")
    LEDGER.add("厚み計測ホールセンサー + 揺動アーム", 0.035, "概算", "センサー", qty=2)
    # 下段 LiDAR。**下向きに吊る。**
    #
    # ⚠ 2026-08-06 に取付を 2 度作り直した。経緯を残す（同じ穴に落ちないため）:
    #   1. もとは立板（t3）の側面に φ60 の胴を抱かせて「4-M3 で留めた」と
    #      宣言していた。円筒の側面と平板は**線でしか当たらない**（座面 0mm²）
    #      うえ、その向きだと**走査面が垂直**になる。実物にならない。
    #   2. 立板の下端に水平な座を吊り、その上に載せる形にした。組めるが、
    #      調整ねじが胴の脇に立つので六角レンチが胴をかすめる。
    #   3. **吊り下げ**（この形）。可動板の下面に本体を留め、調整ねじは
    #      上から入れる。板の上は完全に開いた面になるので、レンチが素直に
    #      入る。しかも立板が要らない（固定板を横材の上面へ直に載せる）。
    #
    # ⚠ 高さは**横材の上面（Z=135）から下向きに決まる**。上へ逃がせないのは
    #   椅子の座板（Z 150..162 / X 130..460）が前の LiDAR の真上にあるため。
    #   ねじの頭の上端は 145 で、座板まで 5mm しか無い。
    #   （水平出しは**椅子を載せる前**にやること。ASSEMBLY.md 工程 10b）
    cross_top = P.BASE_Z1                        # 横材の上面 = 固定板の下面
    # 取付面 = 固定板の下面 − すきま − 可動板の厚み
    low_mount = cross_top - P.LIDAR_LVL_GAP - P.LIDAR_LVL_TOP_T
    assert abs(P.LIDAR_LOW_Z - (low_mount - P.LIDAR_PLANE_Z)) < 1e-6, \
        f"LIDAR_LOW_Z {P.LIDAR_LOW_Z} が実装位置 {low_mount - P.LIDAR_PLANE_Z} とずれている"
    # 胴の -X 端はスカート（420..420.8）の外へ 1mm 以上逃がす
    low_x0 = P.BASE_X / 2 + 2.0
    assert P.LIDAR_LOW_X + 2.0 == low_x0 + P.LIDAR_W / 2, \
        f"LIDAR_LOW_X {P.LIDAR_LOW_X} が実装位置とずれている"
    # 板は「ねじの間隔 + 座面」で決まる。胴（50）より広い
    lvl_hw = P.LIDAR_LVL_PITCH_X / 2 + P.LIDAR_LVL_EDGE
    lvl_hh = P.LIDAR_LVL_PITCH_Y / 2 + P.LIDAR_LVL_EDGE
    for sx, nm in ((1.0, "front"), (-1.0, "rear")):
        cx = sx * (P.LIDAR_LOW_X + 2.0)
        cross = f"cross_x{'p' if sx > 0 else 'm'}410_mid"
        # ⚠ 固定板は**横材の上面へ直に載せて溝ナットで留める**（立板なし）。
        #   板は横材（X 400..420）の上まで伸ばし、そこに 2-M5 を落とす。
        #   ⚠ 前側は椅子マウント脚（X 385..435 / Y 40..80 / Z 135..150）が
        #     同じ面に立っているので、その帯だけ抜く。
        base_in = sx * (P.BASE_X / 2 - P.EXT_W)      # 横材の内側の端（400）
        top_nm = _lidar_leveler(
            parts, nm, (cross, "TSLOT", "2-M5 溝ナット（横材の上面）"),
            cx, 0.0, low_mount,
            base_x=(base_in, cx + sx * lvl_hw),
            top_x=(sx * low_x0 - sx * 5.0, cx + sx * lvl_hw),
            plate_y=(-lvl_hh, lvl_hh),
            notch=((sx * 396.0, sx * 437.0, 36.0, 90.0) if sx > 0 else None))
        # 本体は**可動板の下面**に取付面を当てて吊る（`Rot(180,0,0)` で反転）。
        # 取付は UST-20LX の底面ねじ穴 2-M3 深さ6・ピッチ 40（外形図）。
        put(parts, Pos(cx, 0.0, P.LIDAR_LOW_Z) * Rot(180, 0, 0) * L.lidar(),
            f"lidar_low_{nm}", to=top_nm, how="BOLT",
            note=f"2-M3 ピッチ{P.LIDAR_MOUNT_PITCH:.0f}")
    # 上段LiDAR は側面トラス前柱に直付け（専用支柱を立てない）
    # ⚠ 柱の**内側**（Y=305）に置くと、椅子の上のマスコット規定外形
    #   300×300×600 に食い込む。マスコットは規定 3.1.3 の最小サイズで、
    #   削ることも動かすこともできない。
    #   柱の **+X 面**（X=390）に出せば、マスコットの前方に抜けて干渉しない。
    # ⚠ 上段も吊り下げ。立板は柱の +X 面に張り、**その上端面**で固定板を
    #   受ける（下段と違って横材の上面が無いので、立板が要る）。
    hi_x0 = P.FRONT_POST_X + P.EXT_W / 2         # 柱の +X 面
    hi_x1 = hi_x0 + P.LIDAR_BRK_T
    hi_mount = P.LIDAR_HIGH_Z + P.LIDAR_PLANE_Z  # 取付面 = 可動板の下面
    hi_b0 = hi_mount + P.LIDAR_LVL_TOP_T + P.LIDAR_LVL_GAP   # 固定板の下面
    hi_t0 = hi_x1 + P.LIDAR_LVL_CLEAR
    assert P.LIDAR_HIGH_X == hi_t0 + P.LIDAR_W / 2, \
        f"LIDAR_HIGH_X {P.LIDAR_HIGH_X} が実装位置とずれている"
    # 立板は柱に沿って上がり、上端面（= 固定板の下面）で座を受ける。
    # 下端は胴の下端より少し下まで伸ばして、溝ナットの留め代を稼ぐ。
    hi_brk_z0 = P.LIDAR_HIGH_Z - P.LIDAR_PLANE_Z - 40.0
    hi_brk = (Pos((hi_x0 + hi_x1) / 2, P.LIDAR_HIGH_Y, (hi_brk_z0 + hi_b0) / 2)
              * Rot(0, 90, 0) * L.plate(hi_b0 - hi_brk_z0, 70.0, P.LIDAR_BRK_T))
    put(parts, hi_brk, "lidar_high_brk", to="post_front_L", how="TSLOT",
        note="2-M5")
    LEDGER.add_solid(f"上段 LiDAR 立板 A5052 t{P.LIDAR_BRK_T:.0f}", hi_brk,
                     "A5052", "センサー")
    hi_top = _lidar_leveler(
        parts, "high", ("lidar_high_brk", "BOLT", "2-M4（立板の上端面にタップ）"),
        P.LIDAR_HIGH_X, P.LIDAR_HIGH_Y, hi_mount,
        # 固定板は立板の上端面（X 390..398）まで内側へ伸ばす
        base_x=(hi_x0 - 3.0, P.LIDAR_HIGH_X + lvl_hw),
        top_x=(hi_t0 - 5.0, P.LIDAR_HIGH_X + lvl_hw),
        plate_y=(P.LIDAR_HIGH_Y - lvl_hh, P.LIDAR_HIGH_Y + lvl_hh))
    put(parts, Pos(P.LIDAR_HIGH_X, P.LIDAR_HIGH_Y, P.LIDAR_HIGH_Z)
        * Rot(180, 0, 0) * L.lidar(),
        "lidar_high", to=hi_top, how="BOLT",
        note=f"2-M3 ピッチ{P.LIDAR_MOUNT_PITCH:.0f}")
    LEDGER.add("2D LiDAR 北陽 UST-20LX（クラス1）", P.LIDAR_MASS, "カタログ",
               "センサー", qty=3)
    # 計測輪 3 輪。**骨格の真下に置くこと。**
    # 以前は (350,0) に置いていたが、そこには真上に部材が無く、
    # 支持腕も描いていなかったので 3 輪とも床の上に浮いていた。
    #   (210,0)      … 横材 x=210（y -275..275）の下
    #   (-180,±345)  … ベース主桁 y=±350 の下
    # ⚠ **左右 2 輪の X は -350 から -180 へ移した（2026-08-07）。**
    #   オムニ（幅 20）に替えて支持側の積み上げが内側へ寄り、エンコーダが
    #   メカナム（|Y| 265.5..325 / X ±250..350 / Z 0..100）の中へ入った。
    #   外側はスカート内面（|Y|=360）まで 5mm しかないので幅では逃げられず、
    #   **車輪ごとメカナムの X 帯の外**へ出すしかなかった。
    #   ⚠ 移動しても解ける式は変わらない。転がり方向 +X の輪が測るのは
    #     v_i = vx - ω·y_i で、x_i は式に現れない（効くのは Y 間隔 700mm）。
    # ⚠ **左右 2 輪の Y も 350 → 345 へ寄せた。** 幅 20 の車輪を 350 に置くと
    #   外面がスカート内面（360）とツラになる。345 で 5mm 残る。
    #   Y 間隔は 700 のままなので ω の分解能は変わらない。
    # ⚠ 支持側 si は**機体の内側**。外側はスカート内面まで 5mm しかなく、
    #   軸受も基板も入らない（詳細は _odo_unit の頭）。
    # ⚠ **中央の 1 輪だけ 90° 傾ける（yaw=90）。** 3 輪が同じ向きだと
    #   測れるのは (vx, ω) の 2 つだけで、**vy が原理的に出ない**
    #   （どの式でも vy の係数が 0 になる＝ランク 2）。メカナムは横行が
    #   主役なので、補正したい成分がまるごと落ちる。90° 回した 1 輪が
    #   vy の係数を立てて、はじめて (vx, vy, ω) が解ける。
    #   傾けた輪はガイドが Y 方向へ出るので、受けは **x=210 の横材**
    #   （Y に通っている）。支持側は -X（si=+1）… +X は前輪モーター
    #   マウント（X 235..365）が迫っている。
    for k, (x, y, host, si, yaw) in enumerate((
            (210.0, 0.0, "cross_xp210_mid", 1, 90.0),
            (-180.0, 345.0, "rail_L_out", -1, 0.0),
            (-180.0, -345.0, "rail_R_out", 1, 0.0))):
        _odo_unit(parts, k, x, y, host, si, yaw)
    # 照準カメラ（旋回体に載せず車体前部に固定＝ヨー±30°の範囲は画角でカバー）
    # ⚠ 中央（Y=0）まで腕を渡すとマスコット規定外形を 18,000mm³ 貫く。
    #   マスコットは規定 3.1.3 の最小 300×300×600 で、削れも動かせもしない。
    #   カメラは前柱の +X 面に付け、|Y|=340 のオフセットは校正で吸収する。
    put(parts, Pos(P.FRONT_POST_X + P.EXT_W / 2 + 1.5, -340.0, 620.0)
        * Rot(0, 90, 0) * L.plate(60.0, 50.0, 3.0),
        "cam_brk", to="post_front_R", how="TSLOT", note="2-M5")
    put(parts, Pos(P.FRONT_POST_X + P.EXT_W / 2 + 18.0, -340.0, 620.0)
        * Box(30.0, 40.0, 30.0, align=CTR),
        "aim_camera", to="cam_brk", how="BOLT", note="2-M3")
    LEDGER.add("照準カメラ ブラケット A5052 t3", 0.024, "体積概算", "センサー")
    LEDGER.add("照準カメラ", 0.060, "概算", "センサー")
    LEDGER.add("IMU + 配線", 0.050, "概算", "センサー")
    return parts


def _grabber_fixed():
    """グラバーの固定側: レール（アウター/中間）+ ベルト駆動 + 駆動モーター。"""
    parts = []
    # レール本体の形状は build() 側で繰り出し量を反映して配置する（ここでは質量のみ計上）
    LEDGER.add(f"スライドレール MISUMI {P.RAIL_MODEL}", P.RAIL_MASS, "MISUMI カタログ", "装填", qty=2)
    # ベルト駆動: 駆動軸(X=+20) と アイドラ(X=-450) を Y=±240 に置く。
    # 斜路(|Y|<322, X=10..175) と 上押さえ(|Y|<200) の双方を避けた唯一の通路。
    # 軸は**軸受まで届いていなければならない**。長さ 530（|Y|<=265）では
    # |Y|=338.5 のブラケットに届かず、プーリごと 65mm 浮いていた。
    SHAFT_LEN = 2 * 338.5
    for x, tag in ((BELT_DRIVE_X, "drv"), (BELT_IDLER_X, "idl")):
        # プーリ軸受ブラケット（側面トラス上桁から吊る）。
        # 上桁の内面は |Y|=340。板厚 3 の中心を 338.5 に置いてはじめて面が接する。
        # 333.0 では 5.5mm 浮いていた（「だいたい内側」で座標を書くとこうなる）
        for sy in (1, -1):
            sd = "L" if sy > 0 else "R"
            # レール取付プレート（|Y| 336..340）の内側に置き、そこへ留める
            # 軸（φ10）が通る穴を開ける。無いと軸が板を 236mm³ 貫く
            # ⚠ 留める相手で |Y| が変わる。レール取付プレートは X -350 から
            #   しかない。その外（X<-350）で留まる相手は上桁だけで、
            #   桁の内面は 340（プレートの内面 336 より外）。
            on_plate = x > -350.0
            dx, bw = BELT_BRK_DX[tag], BELT_BRK_W[tag]
            put(parts, Pos(x + dx, sy * (334.5 if on_plate else 338.5),
                           BELT_Z + 10.0)
                * Rot(90, 0, 0)
                # ⚠ 従動側（idl）だけ外形を荷重の形に切ってある
                #   （`scripts/topo_opt.py`）。43g → 19g。駆動側（drv）は
                #   モーター取付が乗るので座面が板の 8 割を占め、削れない。
                * (L.topo_plate(f"beltbrk_{tag}_{sd}", 3.0) if tag == "idl"
                   else (L.plate(bw, 60.0, 3.0)
                         # ⚠ 穴は**軸の位置**。ブラケットをずらしたら穴も
                         #   逆にずらす。ずらし忘れると軸受が軸から dx 離れる。
                         - Pos(-dx, -10.0, 0) * Cylinder(6.0, 6.0, align=CTR))),
                f"beltbrk_{tag}_{sd}",
                # ⚠ ブラケットは |Y| 333..336。上桁（340..360）とは 4mm 空く。
                #   実際に面で当たるのはレール取付プレート（336..340）。
                to=(f"rail_plate_{sd}" if on_plate else f"topbeam0_{sd}"),
                how="BOLT", note="2-M5"),
            # ⚠ 板に穴を開けただけでは軸と接しない。**ブッシュが荷重を渡す**。
            #   穴＝逃げ、ブッシュ＝軸受。両方描かないと組立にならない。
            put(parts, L.mat(Pos(x, sy * (333.0 if on_plate else 337.0), BELT_Z)
                             * Rot(90, 0, 0)
                             * (Cylinder(6.0, 6.0, align=CTR)
                                - Cylinder(5.1, 8.0, align=CTR)), "STEEL"),
                f"beltbush_{tag}_{sd}", to=f"beltbrk_{tag}_{sd}",
                how="PRESS", note="フランジブッシュ")
        # ⚠ 左右を貫く**通し軸にしてはいけない**。キャリッジはこの軸の
        #   X 位置を通過するので、通し軸だと上押さえのガイド軸（|Y|=60、
        #   543mm³ ×2）と櫛歯 5 本（471mm³ ×5）を必ず貫く。
        #   ストロークの端（全閉・全開）では当たらないので、**中間位置を
        #   検査しないと見つからない**類の不具合。
        #   ベルトは |Y|=BELT_Y にしか無いので、そこだけ支えるスタブ軸で足りる。
        for sy in (1, -1):
            sd = "L" if sy > 0 else "R"
            # 内端は櫛歯（|Y| 210..230）の外。220 だと歯の中に入る。
            # ⚠ 外端は**レール取付プレートの内面（336）より内側**で止める。
            #   軸受ブラケット（|Y| 333..336）とブッシュ（333）を通れば
            #   足りるので 335.5 まで。338.5 だとプレートを 98mm³ ×2 貫く。
            stub = 335.0 - (BELT_Y - 8.0)
            put(parts, Pos(x, sy * (335.0 - stub / 2), BELT_Z) * Rot(90, 0, 0)
                * Cylinder(5.0, stub, align=CTR),
                f"beltshaft_{tag}{sd}", to=f"beltbush_{tag}_{sd}", how="ROTATE",
                note="フランジベアリング 1 個（スタブ軸）")
        for sy in (1, -1):
            sd = "L" if sy > 0 else "R"
            # ⚠ 胴 12 に幅 15 のベルトを掛けていたので、鍔（半径 +3）が
            #   ベルトの縁に 746mm³ ×4 食い込んでいた。17 にする。
            # ⚠ 内径は**軸径と同じ**にする。軸は φ10 なのに内径を 8 と
            #   書いていたので、全周 1mm ずつ喰い合って 452mm³ ×4。
            #   「圧入」の許容 4,000 の陰に隠れていた。
            put(parts, Pos(x, sy * BELT_Y, BELT_Z) * Rot(90, 0, 0) * L.pulley(40.0, 17.0, 10.0),
                f"beltpul_{tag}_{sd}", to=f"beltshaft_{tag}{sd}", how="PRESS", note="止めねじ")
    LEDGER.add("アイドラ軸 φ10 中空", 0.090, "体積概算", "装填")
    # ⚠ ベルトクランプは「ベルトを掴む板」なのに、**掴む相手が描かれて
    #   いなかった**。クランプの固定先を駆動プーリにしていたが、
    #   プーリは 40mm 以上離れている。ベルトを実体にして、そこに掴ませる。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        blen = abs(BELT_DRIVE_X - BELT_IDLER_X)
        put(parts, L.mat(
            Pos((BELT_DRIVE_X + BELT_IDLER_X) / 2, sy * BELT_Y, BELT_Z)
            * Rot(90, 0, 0)
            # ⚠ 枠の長さはプーリ間距離 + プーリ 2 個ぶん。+40 では
            #   ストローク全開でクランプ（X -392..-332）がベルトの端を
            #   越えて「掴む相手が無い」状態になっていた。
            * L.belt_loop(blen, 20.0, 3.0, 15.0), "RUBBER"),
            f"belt_grab_{sd}",
            to=(f"beltpul_drv_{sd}", f"beltpul_idl_{sd}"), how="MESH",
            note="HTD5M 幅 15")
        LEDGER.add("グラバー駆動ベルト HTD5M", 0.070, "カタログ", "装填")
    # ⚠ 駆動軸と同じ (X,Z) に置くと、本体（φ36×63）が軸を丸ごと包む（5,551mm³）。
    #   Z を 58mm 上げてベルトで落とす。
    gm_z = BELT_Z + 58.0
    # ⚠ 「立ち上げ板」と注記しておいて実体が無かった。モーターの取付面は
    #   Y=272 にあるのに、留め先のブラケットは Y 333..336。61mm 離れている。
    #   本体の側面がブラケットに 72mm² 触れているだけで、留まっていない。
    brk = L.plate(70.0, 70.0, 3.0) - Cylinder(9.0, 8.0, align=CTR)
    brk = Rot(90, 0, 0) * brk
    # 腕はブラケットの面（|Y|=333）まで届かせる
    # 面板の内面（269）から相手の内面（333）まで正確に渡す
    # ⚠ 腕の Z は**グラバー用ケーブルベアの通り道**を塞がない位置に置く。
    #   -33.5（Z 861..864）では |Y| 300..320 / Z 845..875 の帯を 1,800mm³
    #   横切っていた。ここは U ループの可動側が通れる**唯一の帯**で、
    #   ほかは駆動系（モーター Z 878..914 / 駆動軸 Z 833..843 /
    #   軸受ブラケット |Y| 333..336 / ベルト |Y| 232.5..247.5）と
    #   レール取付プレート（|Y| 336..340）で埋まっている（測定済み）。
    #   ⚠ **下げすぎると板から離れる。** 板は Z 861..931（70 角の中心が
    #     gm_z=896）なので、腕は 861..931 の中に無いと 2 塊に分かれる
    #     （-48 にして「分断」が出た）。上へ逃がすしかない。
    #   -22（Z 872.5..875.5）なら、モーター本体の下端 878 まで 2.5mm、
    #   通り道は Z 851..872 の 21mm が空く（φ12 の束に必要な 20mm を満たす）。
    # ⚠ **面板と腕を別部品にする（2026-08-05）。** 面板は板厚が Y、腕は
    #   板厚が Z で、1 部品にすると直交した 2 枚＝曲げ品になる。自校では
    #   アルミ板を曲げられない（`export_fab.CAN_BEND = False`）。
    #   腕を面板の**端面のタップ**へ留める（横穴加工まではできる）。
    #   ⚠ 端面に M4 を立てるので腕は **t6**（`L.TAP_MIN[4] = 6.0`）。
    #     t3 のままではタップが立たない。
    org_gm = Pos(BELT_DRIVE_X, 270.5, gm_z)
    put(parts, L.mat(org_gm * brk, "A5052"),
        "grab_motor_brk", to="grab_motor_arm", how="BOLT",
        note="2-M4（腕の端面にタップ）")
    # ⚠ **腕は面板の外面（|Y|=272）から始める。** 前は面板の内面（269）から
    #   引いていたので、面板の板厚ぶん（30×3×6 = 540mm³）貫いていた。
    #   端面にタップを立てて横から留める形なので、突き合わせが正しい。
    #   外端 333（軸受ブラケットの面）は変えない → 長さ 64 → 61。
    put(parts, L.mat(org_gm * Pos(0, 32.0, -22.0)
                     * Box(30.0, 61.0, 6.0, align=CTR), "A5052"),
        "grab_motor_arm", to="beltbrk_drv_L", how="BOLT", note="2-M4")
    LEDGER.add("グラバーモーター 面板 A5052 t3", 0.045, "体積概算", "装填")
    LEDGER.add("グラバーモーター 腕 A5052 t6", 0.031, "体積概算", "装填")
    gm_at = Pos(BELT_DRIVE_X, 272.0, gm_z) * Rot(90, 0, 0)
    put(parts, gm_at * L.m2006(),
        "grab_motor", to="grab_motor_brk", how="BOLT", note="4-M3 PCD26")
    flange_screws(parts, gm_at, "grab_motor_brk", "grab_motor",
                  P.M2006_BOLT_PCD, 4, 3, 3.0, "grab_mot", kind="FLAT")
    # Rot(90,0,0) では軸が -Y に出る。本体側（+Y）に置くとプーリが胴の中に入る
    # ⚠ 減速プーリの |Y| は 258 → 262。駆動プーリの胴を 12 → 17 に広げた
    #   ら、その鍔（|Y| 248.5..250.5）と減速プーリの鍔（251..253）が
    #   0.50mm =しきい値ちょうどになった。262 なら 4.5mm 空く。
    #   モーター軸（260..272）との噛み合いも 3 → 7mm に増える。
    put(parts, Pos(BELT_DRIVE_X, P.GRAB_RED_Y, gm_z) * Rot(90, 0, 0) * L.pulley(24.0, 10.0, 8.0),
        "grab_pul_m", to="grab_motor", how="SHAFT", note="止めねじ")
    put(parts, Pos(BELT_DRIVE_X, P.GRAB_RED_Y, BELT_Z) * Rot(90, 0, 0) * L.pulley(24.0, 10.0, 10.0),
        "grab_pul_s", to="beltshaft_drvL", how="PRESS", note="止めねじ")
    # ⚠ 角枠で描いていたので、プーリ（φ24）が枠の平らな辺を 979mm³ ×2
    #   突き抜けていた。鉛直なのでループを 90° 回して置く。
    put(parts, L.mat(Pos(BELT_DRIVE_X, P.GRAB_RED_Y, (BELT_Z + gm_z) / 2)
                     * Rot(90, 0, 0) * Rot(0, 0, 90.0)
                     * L.belt_loop(gm_z - BELT_Z, 12.0, 3.0, 10.0), "RUBBER"),
        "grab_belt", to=("grab_pul_m", "grab_pul_s"), how="MESH")
    LEDGER.add("グラバー駆動 プーリ×2 + ベルト", 0.070, "概算", "装填")
    LEDGER.add("プーリ軸受ブラケット A5052 t4", 0.038, "体積概算", "装填", qty=4)
    LEDGER.add("M2006 P36 (グラバースライド)", P.M2006_MASS, "DJI カタログ", "装填")
    LEDGER.add("駆動クロスシャフト φ10 中空", 0.090, "体積概算", "装填")
    LEDGER.add("タイミングベルト/プーリ一式", 0.220, "概算", "装填")
    return parts


BUCKET_X = -70.0    # バケツ中心 X（片持ちビーム上）
# グラバーのベルト駆動。
# ⚠ **旋回体の掃引包絡の外に置くこと**（scripts/sweep_envelope.py）。
#   Z>760 では旋回体がヨー±30° で X=-27.5 まで来るので、固定物は **X <= -38**。
#   以前は X=+60 に置いていて、コメントには「斜路と台座横梁を避けた位置」とあったが、
#   **旋回体を見ていなかった**。総当たり干渉チェック（interference_full.py）で
#   grabber_fixed × turret が 0.00mm と出て発覚した。
#   ヨー0° の姿勢だけ見ていると気付けない類の不良。
# ⚠⚠ **未解決の機構課題**（干渉チェッカーが検出）
#   プーリ間距離はストロークより長くなければならない。ベルトの直線部を
#   掴んで引く機構なので、クランプはストローク全域でベルトの上に
#   居続ける必要がある。
#     プーリ間 = 370mm（-430 .. -60）／ストローク = 425mm
#   なので全開でクランプがベルトの端を 55mm 越え、「掴む相手が無い」
#   状態になる（`belt_clamp_* ↮ belt_grab_*` の離れ 2 件）。
#   駆動プーリを前方（X 330）へ出せば 760mm 取れるが、そこは**砲塔の
#   掃引の中**（仰角側板は X 429 まで来る）で、モーターが 2,987mm³、
#   ベルトが旋回アームを 2,700mm³ 貫く。アイドラ側は既に機体最後方
#   （上桁の端 -430）で、これ以上後ろへは出せない。
#   → 取り得る案は 3 つ。どれも戦略（雑巾の掬い方）に関わるので保留:
#     (a) ストロークを 310 以下に減らす（グラバーのリーチが 115mm 減る）
#     (b) 動滑車（2:1）にしてストロークをプーリ間の 2 倍にする
#     (c) ワイヤ + ドラム駆動にする（ストロークの上限が無くなる）
# ⚠⚠ **1:1 のクランプ式ベルトでは、この機体でストローク 424.6mm は
#   閉じない。** プーリ間を広げる方向は実際に試して行き詰まった記録を残す。
#
#   使える走行距離 = プーリ間 − プーリ径(40) − クランプ幅
#   現状: 370 − 40 − 60 = 270mm（必要 424.6）
#
#   駆動プーリは前へ出せる（掃引包絡 -27.5 は高い Z の砲塔サイドプレートの
#   話で、ベルトの帯 Z 818..858 を |Y|=240 で塞ぐのは台座横梁 X 75..95 と
#   350..370 だけ。旋回リングは半径 178 まで、旋回アームは Z 862 以上）。
#   ただし **4 つの制約が 5mm 単位で噛み合って閉じない**:
#     (1) ベルトループはプーリ中心の**外側 40mm**まで張り出す
#         （Box(blen+80)）。駆動 +50 ではループ端が +90 で台座横梁を
#         706mm³ ×2 削る。+30 まで下げると span 495 で 15mm 足りない
#     (2) クランプの全閉位置はキャリッジ後端横梁（X 1.6..61.6）を避ける
#         必要がある。+5 に置くと 1,055mm³ ×2 重なる。-30 まで下げると
#         走行の上端が -8 になり、また span が足りなくなる
#     (3) アイドラを -465 に下げると上桁（-430 まで）に届かず座面 0mm²
#     (4) 駆動モーターを前へ動かすと砲塔プレート受け金具（Z 868..892）に
#         2,640mm³ 入る。モーターの新しい置き場所も要る
#   実測: この 4 つを同時に満たす組み合わせは無く、1 つ直すと別が壊れる
#   （13 件・最大 2,640mm³ まで増えた）。
#   → **駆動方式そのものを変えるしかない**（ラック&ピニオン or ワイヤ+
#     ドラム）。パラメータの調整では閉じないので再挑戦しないこと。
#
# ■ 連続掃引検査（scripts/sweep_fine.py）による定量化 2026-07-30
#   以前は「離れ 2 件」＝ベルトが届かない、という形でしか出ていなかったが、
#   連続掃引では**クランプがアイドラを貫く**として出る:
#     5,231mm³  beltpul_idl_{L,R} ↔ belt_clamp_{L,R}  @ grab=274.4
#       628mm³  beltshaft_idl{L,R} ↔ belt_clamp_{L,R} @ grab=256.4
#   使える走行距離 = (Xd − Xi) − 2×(プーリ半径+ベルト厚+すきま) − クランプ幅
#                  = 370 − 56 − 60 = 254mm（必要 424.6）
#
# ■ 置き場所を全部当たった結果（2026-07-30）
#   アイドラは **-462 まで下げられる**（X -520..-400 / |Y| 220..350 /
#   Z 790..900 の帯は完全に空。機体後端は LiDAR の -485 なので包絡も
#   増えない）。ブラケットを +25 オフセットして 90 幅にすれば上桁
#   （-430 まで）に 38mm 重なって座面も出る。→ span 402、走行 316mm。
#   駆動側は前へ出せない:
#     ・X 75..95 に台座横梁（Z 818..838、Y 全幅）
#     ・ベルト帯の上限は旋回アームの下面 862、下限は car_brk の上面 813.5
#       → 高さ 48.5mm の窓しかなく、φ40 プーリ（40mm）でぎりぎり
#     ・モーター（φ36×63）を X 50 へ動かすと Z 896 は**旋回アームの中**
#   モーターを窓（Z 813.5..862、|Y| 195..261）に寝かせれば入るが、
#   そこから機体へ**留めに行く道が無い**:
#     ・外側へ伸ばすと |Y| 311.9..316.9 / Z 766..826 のキャリッジ側板が
#       X 32..68 を通過する（腕が切られる）
#     ・上は旋回アーム 862、下は car_brk 813.5
#     ・|Y| 340..360 へ抜けるには上桁（Z 818..838）を貫くことになる
#   → ワイヤ+ドラムでも**モーターの置き場所が解けない**。
#     機構の選択（下記 a/b/c）はユーザーの判断待ち。
#   ⚠ (b) 動滑車は**市販 3 段レールでは組めない**。中間段はタップが無く、
#     外段と内段に挟まれて面が出ていないので、倍動の滑車を付けられない。
#     やるなら 2 段レール 2 組 + 中間キャリッジ（構造から作り直し）。
BELT_DRIVE_X = -60.0    # 掃引包絡 -27.5 から 32mm 逃げる
# ⚠ -430（上桁の端）から -462 へ下げた。X -520..-400 / |Y| 220..350 /
#   Z 790..900 の帯は完全に空で、機体後端は LiDAR の -485 なので**包絡も
#   増えない**。走行距離が 254 → 316mm に伸びる。
#   ブラケットは +25 オフセットして 90 幅にし、上桁（-430 まで）に 38mm
#   重ねて座面を出す（-465 では 0mm² になる）。
BELT_IDLER_X = -462.0   # グラバー アイドラ X（後端・LiDARエンベロープ内）
# ブラケットの X オフセットと幅（アイドラだけ後方へ出るので上桁へ寄せる）
BELT_BRK_DX = {"drv": 0.0, "idl": 25.0}
BELT_BRK_W = {"drv": 70.0, "idl": 90.0}
BELT_Y = 240.0          # ベルト通路 |Y|（上押さえ200 と 斜路側壁290 の間）
# ⚠ ベルトのループは φ40 のプーリに巻くので**高さ 40mm を占める**
#   （BELT_Z ±20）。上下 2 つの制約に挟まれている:
#     下 … スライドレールの上端 814.6
#     上 … 旋回アームの平面 862（ヨー ±20° で後ろのアームが X -53 まで
#           来るので、ベルトの X 範囲と必ず重なる）
#   845 だとループ上端が 865 でアームに 907mm³ 入る。838 なら
#   818..858 で、下に 3.4mm・上に 4mm 残る。
BELT_Z = 838.0          # ベルト高さ（レール上端と旋回アーム平面のあいだ）
# クランプは**キャリッジの範囲内**かつ**ベルトの範囲内**に居ること。
# クランプ（全閉時の**中心**）と幅。
# ⚠ 走行距離 = (駆動 − アイドラ) − 2×(プーリ半径20 + ベルト厚3 + すきま5)
#              − クランプ幅  =  402 − 56 − 30 = 316mm
#   クランプは全閉で X -118..-88（駆動プーリの -37..-83 まで 5mm）、
#   全開で X -434..-404（アイドラの -485..-439 まで 5mm）。
#   ⚠ 幅を 60 に戻すと 286mm しか走れない。幅と走行距離は 1:1 で効く。
BELT_CLAMP_X = -103.0   # 全閉時のクランプ中心
BELT_CLAMP_W = 30.0


def _mast():
    """マスト上部（主柱は _side_frames の後柱を共用）。"""
    parts = []
    cross_len = 2 * P.MAST_Y - P.EXT_W
    put(parts, Pos(P.MAST_X, 0, P.MAST_BEAM_Z) * Rot(0, 0, 90) * L.ext2020(cross_len),
        "mast_cross", to=("post_rear_L", "post_rear_R"), how="BRACKET", note="各端 2-M5")
    L.add_ext("マスト上部横梁", cross_len, "マスト")
    # 横梁は柱の内面（|Y|=340）に突き合わせるだけなので、ブラケットが要る。
    # 「BRACKET」と宣言しておいて実体を描いていなかった。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ 以前ここに「L 金具が**入らない**継手」と書いていたのは誤り。
        #   探していた面の組が足りなかった。柱と横梁の ±X 面どうしは
        #   確かに同一平面で L 字にならないが、**横梁の下面（z=1152）と
        #   柱の内面（|y|=340）**は直交していて、その内側は空いている。
        #   ここが本来の入隅。平板（ジョイントプレート）は ±X 面に貼るので
        #   鉛直せん断をねじの摩擦だけで受けるが、L 金具は横梁を**下から
        #   受けて**柱へ渡すので、バケツ＋水の鉛直荷重の経路がまっすぐ通る。
        #   平板は X 方向のモーメント（前後の首振り）に効くので両方残す。
        #   位置の妥当性は scripts/corner_bracket.py が幾何から数え直して
        #   検査する（内隅が 4 つのうちどれか・当て面 2 枚・食い込み 0）。
        loc = (Pos(P.MAST_X, sy * (P.MAST_Y - P.EXT_W / 2),
                   P.MAST_BEAM_Z - P.EXT_W / 2)
               * Rot(0, 0, -90 * sy) * Rot(-90, 0, 0))
        bnm = f"brk_mast_corner_{sd}"
        put(parts, loc * L.bracket(), bnm,
            to=("mast_cross", f"post_rear_{sd}"), how="TSLOT",
            note="1-M5（腕 1 枚に 1 本）")
        LEDGER.add("マスト横梁 内隅ブラケット HBLFSN5", 0.012,
                   "MISUMI カタログ", "マスト")
        bracket_screws(parts, loc, bnm, ("mast_cross", f"post_rear_{sd}"))
        for sx in (1, -1):
            # 平板（ジョイントプレート）は柱と横梁の ±X 面にまたがって貼る。
            # ⚠ **高さ 20mm の帯にしていたので、柱側にねじが 1 本しか入らない。**
            #   柱の ±X 面の溝は Z 方向に走るので、帯の高さ 20mm の中に並ぶ
            #   溝ナットは 1 個だけ。「4-M5」と注記して実体は 0〜1 本、という
            #   状態が続いていた（1 本留めの板は、そのねじを軸に回る）。
            #   → **柱側だけ下へ伸ばした L 形**にする。横梁側は溝が Y 方向に
            #     走るので 2 本、柱側は Z 方向に 2 本、合わせて注記どおり 4 本入る。
            x_pl = P.MAST_X + sx * (P.EXT_W + P.MAST_PLATE_T) / 2
            y_beam = sy * (P.MAST_Y - P.EXT_W / 2 - P.MAST_PLATE_L / 2)
            put(parts, L.mat(
                Pos(x_pl, y_beam, P.MAST_BEAM_Z)
                * Box(P.MAST_PLATE_T, P.MAST_PLATE_L, P.EXT_W, align=CTR)
                + Pos(x_pl, sy * P.MAST_Y,
                      P.MAST_BEAM_Z + P.EXT_W / 2 - P.MAST_PLATE_H / 2)
                * Box(P.MAST_PLATE_T, P.EXT_W, P.MAST_PLATE_H, align=CTR),
                "A5052"),
                f"brk_mast_cross_{sd}{'p' if sx > 0 else 'm'}",
                # ⚠ note は**その組 1 つに要る本数**。「4-M5」は板ぜんたいの
                #   本数のつもりだったが、相手が 2 つ（横梁と柱）あるので
                #   8 本の宣言になっていた。板 1 枚は 横梁へ 2 本・柱へ 2 本。
                to=("mast_cross", f"post_rear_{sd}"), how="TSLOT", note="2-M5")
            LEDGER.add("マスト横梁ジョイントプレート A5052 t6", 0.024,
                       "体積概算", "マスト")
            # ⚠ 位置は溝の芯から決める。横梁側は Z=MAST_BEAM_Z の線上、
            #   柱側は Y=±MAST_Y の線上でなければ後入れナットに入らない。
            pnm = f"brk_mast_cross_{sd}{'p' if sx > 0 else 'm'}"
            x_seat = P.MAST_X + sx * (P.EXT_W / 2 + P.MAST_PLATE_T)
            # ⚠ 横梁側の 2 本は **Y=304 と 320**。中心 ±8（310/326）にすると
            #   J8 のボルト（Y=334）と内隅ブラケットのねじ（Y=329）に頭が
            #   当たる（M5 の頭は φ8.5 なので芯間 8.5mm 以上要る）。
            seats = [((x_seat, y_beam + sy * dy, P.MAST_BEAM_Z), "mast_cross")
                     for dy in (-14.0, 2.0)]
            seats += [((x_seat, sy * P.MAST_Y, P.MAST_BEAM_Z - dz),
                       f"post_rear_{sd}") for dz in (0.0, 30.0)]
            for i, (pos, tgt) in enumerate(seats):
                ray = (*pos, float(sx), 0.0, 0.0)
                put_screw(parts,
                          Pos(*pos) * Rot(*_axis_rot((float(sx), 0.0, 0.0)))
                          * L.screw_tnut(5, P.MAST_PLATE_T),
                          f"scr_{pnm}_{i}", to=(pnm, tgt),
                          how=("THRU", "TNUT"), kind="CAP", size=5,
                          length=L.screw_len(P.MAST_PLATE_T, 6.0),
                          extras=(("TNUT", 5),),
                          note="1-M5 + 後入れナット", tool=ray)
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # 横梁の +X 面から始める。以前は横梁の中心から伸ばしていたので
        # 10mm ぶん（942mm³）横梁に食い込んでいた
        # ⚠ 410 → 300（2026-08-07）。受け板を 340 角から受け円環＋帯
        #   （X 方向 190）へ切り直したので、先端 110mm には**何も載らない**。
        #   帯の端（BUCKET_X + 95 = +25）に 10mm 足した 300 で切る。
        ln = P.MAST_ARM_LEN
        x0 = P.MAST_X + P.EXT_W / 2
        assert x0 + ln >= BUCKET_X + P.BUCKET_SEAT_RAIL_L / 2 + 5.0, (
            f"片持ちビーム（{x0 + ln:.0f} まで）が受け板の帯"
            f"（{BUCKET_X + P.BUCKET_SEAT_RAIL_L / 2:.0f} まで）に届かない")
        put(parts, Pos(x0 + ln / 2, sy * P.MAST_CANTILEVER_Y, P.MAST_BEAM_Z) * L.ext2020(ln),
            f"mast_arm_{sd}", to="mast_cross", how="BRACKET", note="金具 2 個・各 2-M5")
        L.add_ext("バケツ片持ちビーム", ln, "マスト")
        # ⚠ 金具は**アームの側面**（|Y| ±10）と横梁の +X 面をつなぐ。
        #   上下面（Z ±10）に付けると、横梁もアームも同じ高さ帯
        #   （Z 1130..1150）にあるので、金具は横梁の**上／下**へ逃げてしまい
        #   横梁の側面に 0mm² しか当たらない。同じ高さの部材どうしを
        #   つなぐ金具は、高さ方向ではなく**幅方向**に出すこと。
        for sy2 in (1, -1):
            bnm = f"brk_mast_arm_{sd}_{'p' if sy2 > 0 else 'm'}"
            loc = Pos(x0, sy * P.MAST_CANTILEVER_Y + sy2 * P.EXT_W / 2,
                      P.MAST_BEAM_Z)
            put(parts, loc * L.bracket_m(flip_y=sy2 < 0), bnm,
                to=(f"mast_arm_{sd}", "mast_cross"), how="TSLOT",
                note="1-M5（腕 1 枚に 1 本）")
            LEDGER.add("片持ちビーム根元ブラケット HBLFSN5", 0.012, "MISUMI カタログ", "マスト")
            bracket_screws(parts, loc, bnm,
                           (f"mast_arm_{sd}", "mast_cross"), flip_y=sy2 < 0)
    # バケツ受け板。⚠ **340 角の板に φ44 の丸穴を 12 個並べていた**（〜2026-08-06）。
    #   中央 φ210 の抜きと穴が重なって三日月形の切れ端が並び、円環から
    #   締結へ向かう荷重の通り道を横切って穴が開いていた。板を先に決めて
    #   から穴で軽くすると、こうなる。**板の形そのものを荷重の通り道に
    #   する**（`L._bucket_seat_outline` に形の理由がある）。
    seat = put(parts,
               Pos(BUCKET_X, 0.0, P.MAST_TOP_Z + P.BUCKET_PLATE_T / 2)
               * L.bucket_seat_plate(P.BUCKET_PLATE_T),
               "bucket_seat", to=("mast_arm_L", "mast_arm_R"), how="TSLOT",
               note="各 2-M5")
    LEDGER.add_solid("バケツ受けプレート A5052 t3", seat, "A5052", "マスト")
    # 受け板 → 片持ちビーム（溝ナット 4 本）。**位置は手で決める。**
    # ⚠ 自動配置（`screw_place`）に任せると、接触域の**両端いっぱい**に
    #   置かれる。この板は帯の端（局所 ±95）が接触域の端そのものなので、
    #   ねじの芯が板の縁から 0.35mm の所に来て、頭 φ8.5 の座面が
    #   12 点中 7 点しか材料に載らなかった（`scripts/screw_seat_check.py`）。
    #   帯を伸ばしても「端いっぱい」は変わらないので、ここは実体で置く。
    grip = P.BUCKET_PLATE_T
    tlen = L.screw_len(grip, 6.0)
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for k, dx in enumerate((-70.0, 70.0)):
            sx_, sy_ = BUCKET_X + dx, sy * P.MAST_CANTILEVER_Y
            sz_ = P.MAST_TOP_Z + P.BUCKET_PLATE_T
            put_screw(parts, Pos(sx_, sy_, sz_) * L.screw_tnut(5, grip),
                      f"scr_bkseat_{sd}{k}",
                      to=("bucket_seat", f"mast_arm_{sd}"),
                      how=("THRU", "TNUT"), kind="CAP", size=5, length=tlen,
                      extras=(("TNUT", 5),),
                      note="1-M5 + 後入れナット（帯の端から 25mm）",
                      tool=(sx_, sy_, sz_, 0.0, 0.0, 1.0))
    return parts


def _bucket_group():
    """移動バケツと、その固定一式 [RULE 3.2.3]。

    ⚠ **以前はバケツが受け板に「載っているだけ」だった。** 質量台帳に
      「バケツ固定バンド/金具 0.150kg（概算）」の 1 行があるだけで、
      形もボルトも無く、`assembly_check` の REST_OK（置いてあるだけで
      よい部品）に逃がしてあった。規定 3.2.3a は「競技中は常に水平を
      維持できるようにしっかりと固定すること」なので、載せるだけでは
      規定を満たさない。「概算で計上した」は「設計した」ではない。

    いまの固定（下から順に）:
        受け板 t3  →  M5 皿ボルト 4 本  →  バケツの底（φ5.5 を 4 か所）
                   →  押さえリング PETG t4（バケツの中）
        位置決めタブ PETG 4 個（胴を 16mm ぶん抱える）
    規定の根拠は `tr_params` の「移動バケツの固定」節に書いた。要点は
    **3.2.3c が禁じているのは 2L 目盛りより「上」の加工**で、底に穴を
    あけるのは許されること、**3.2.3d は固定のみを目的とした部品なら
    中に入れてよい**と書いてあること。
    """
    parts = []
    put(parts, Pos(BUCKET_X, 0.0, P.BUCKET_SEAT_Z) * L.bucket(), "bucket",
        to="bucket_seat", how="BOLT",
        note="4-M5 皿（底面 φ5.5 を 4 か所。規定 3.2.3c は 2L 目盛りより下）")
    LEDGER.add("移動バケツ エンテック PO-24A", P.BUCKET_MASS, "ルールブック 5.3", "マスト")
    ring = Pos(BUCKET_X, 0.0, P.BUCKET_SEAT_Z) * L.bucket_2l_ring()
    put(parts, ring, "bucket_2l_datum", to="bucket", how="CONTAIN",
        note="2L 目盛りを示す検証用の基準リング（実体ではない）")

    # --- 押さえリング（バケツの中） -------------------------------------
    hold_z = P.BUCKET_SEAT_Z + P.BUCKET_WALL_T          # バケツの内底
    hold = put(parts, Pos(BUCKET_X, 0.0, hold_z) * L.bucket_hold_ring(),
               "bucket_hold", to="bucket", how="BOLT",
               note="4-M5 皿（バケツ底を貫いて受け板へ）")
    LEDGER.add_solid("バケツ押さえリング PETG t4", hold, "PETG", "マスト")

    # --- 位置決めタブ 4 個 ----------------------------------------------
    for k in range(P.BUCKET_TAB_N):
        ang = 360.0 / P.BUCKET_TAB_N * k
        tab = put(parts,
                  Pos(BUCKET_X, 0.0, P.BUCKET_SEAT_Z) * Rot(0, 0, ang)
                  * L.bucket_tab(),
                  f"bucket_tab_{k}", to=("bucket_seat", "bucket"),
                  how=("BOLT", "CLAMP"), note="1-M5")
        LEDGER.add_solid("バケツ位置決めタブ PETG", tab, "PETG", "マスト")
        # タブ → 受け板。頭はタブの座（z=+5）に載り、板の下でナットを掛ける。
        hx = BUCKET_X + P.BUCKET_TAB_R * cos(radians(ang))
        hy = P.BUCKET_TAB_R * sin(radians(ang))
        hz = P.BUCKET_SEAT_Z + 5.0
        grip = 5.0 + P.BUCKET_PLATE_T
        put_screw(parts, Pos(hx, hy, hz) * L.screw(5, L.screw_len(grip, 6.0)),
                  f"scr_bktab{k}", to=(f"bucket_tab_{k}", "bucket_seat"),
                  how=("THRU", "THRU"), kind="CAP", size=5,
                  length=L.screw_len(grip, 6.0),
                  extras=(("HEXNUT", 5), ("WASHER", 5)),
                  note="1-M5（タブ → 受け板。板の下でナット）",
                  tool=(hx, hy, hz, 0.0, 0.0, 1.0))

    # --- バケツ固定ボルト 4 本 ------------------------------------------
    # ⚠ 皿ねじの原点は**頭の上面**（`L.screw_flat`）。押さえリングの上面に
    #   合わせて置くと、頭がリングの皿ざぐりにちょうど沈む。
    bolt_top = hold_z + P.BUCKET_HOLD_T
    grip = P.BUCKET_HOLD_T + P.BUCKET_WALL_T + P.BUCKET_PLATE_T
    blen = L.screw_len(grip, 6.0)
    for k, (bx, by) in enumerate(bolt_circle(P.BUCKET_BOLT_N,
                                             2 * P.BUCKET_BOLT_R, 45.0)):
        put_screw(parts,
                  Pos(BUCKET_X + bx, by, bolt_top) * L.screw_flat(5, blen),
                  f"scr_bkt{k}",
                  to=("bucket_hold", "bucket", "bucket_seat"),
                  how=("THRU", "THRU", "THRU"), kind="FLAT", size=5,
                  length=blen, extras=(("HEXNUT", 5), ("WASHER", 5)),
                  note="1-M5 皿（リング → バケツ底 → 受け板。板の下でナット）",
                  tool=(BUCKET_X + bx, by, bolt_top, 0.0, 0.0, 1.0))
    return parts


def _chair_group():
    parts = []
    LEDGER.add("椅子 新JIS 5号（脚切除・座面/背もたれ/接続部のみ）", P.CHAIR_MASS, "実測", "椅子")
    # 椅子マウント脚 4 本。**今まで質量 0.15kg だけ計上して形が無かった。**
    # 形が無いので整合性チェックからも見えず、椅子は骨格の上 15mm を
    # 何にも触れずに浮いていた。「概算で計上した」は「設計した」ではない。
    hosts = ("rail_L_in", "cross_xp210_mid", "rail_L_in", "cross_xp410_mid")
    for i, (mx, my) in enumerate(P.CHAIR_MOUNT_XY):
        # ⚠ **中に閉じた空洞があった。** `- Box(38, 28, H-4)` は上下に 2mm ずつ
        #   肉を残す引き算なので、外から入口の無い空洞になる。切削でも
        #   鋳造でもない限り作れない形で、しかも図の上では成立してしまう。
        # ⚠ 自校の加工は **2D 切り抜き + 穴** まで（`export_fab.CAN_*`）。
        #   肉抜きは**貫通**にすること。貫通なら厚さ 15 の板を輪郭で
        #   切り抜くだけになる。
        # ⚠ **肉抜きは 38×28 → 30×20（2026-08-07）。** 座板を留める M6 の頭は
        #   φ10 で、壁が 6mm しか無いと**どこに置いても頭が壁からはみ出す**
        #   （`screw_seat_check` が 4 本 × 4 個ぶん「座面が足りない」と出す）。
        #   壁 10mm あれば頭が載る。増えるのは 4 個で 75g。
        leg = Pos(mx, my, P.BASE_Z1 + P.CHAIR_MOUNT_H / 2) * (
            Box(50.0, 40.0, P.CHAIR_MOUNT_H, align=CTR)
            - Box(30.0, 20.0, P.CHAIR_MOUNT_H + 2.0, align=CTR)
        )
        put(parts, L.mat(leg, "A5052"), f"chair_mount_{i}", to=hosts[i],
            how="TSLOT", note="2-M5")
        LEDGER.add_solid("椅子マウント脚 A5052", leg, "A5052", "椅子")
    put(parts, Pos(P.CHAIR_X, P.CHAIR_Y, P.CHAIR_SEAT_Z) * L.chair_5go(), "chair_5go",
        to=tuple(f"chair_mount_{i}" for i in range(len(P.CHAIR_MOUNT_XY))),
        how="BOLT", note="座板を貫通する 4-M6 + 大座金")
    # マスコット「シボリ」— **規定 3.1.3 の実体**。
    # ⚠ ここには 300×300×600 の箱（`mascot_envelope`）を置いていた。
    #   規定を満たす「大きさ」は表せても、**当日そこに置く物が無い**。
    #   3.1.3 は「マスコットを製作して座らせること」なので、形を持たせる。
    #   外形は箱と同じ 300×300×600 にちょうど内接するので、周りの部品の
    #   逃げ（上段 LiDAR・照準カメラ・配線）は今までのままでよい。
    # ⚠ 座面中心に置くと**背もたれと接続部**に 59,520mm³ 食い込む。
    #   規定 3.1.3 の最小外形 300×300×600 は削れないので、
    #   座面上で前（-X）へ寄せて背もたれの手前に収める。
    seat = Pos(P.MASCOT_X, P.CHAIR_Y, P.CHAIR_SEAT_Z + P.CHAIR_SEAT_T)
    mp = L.mascot_parts()
    put(parts, seat * mp["mascot_suit"], "mascot_suit", to="chair_5go",
        how="SIT",
        note="尻の M6 鬼目ナット 2 個 ← 座板の下から蝶ボルト（工具無しで外せる）")
    put(parts, seat * mp["mascot_head"], "mascot_head", to="mascot_suit",
        how="SEW", note="首の座（φ92）に差して周囲を縫う")
    for sd in ("L", "R"):
        put(parts, seat * mp[f"mascot_eye_{sd}"], f"mascot_eye_{sd}",
            to="mascot_head", how="SEW", note="ボタン（糸穴 4-φ5）")
        put(parts, seat * mp[f"mascot_foot_{sd}"], f"mascot_foot_{sd}",
            to="mascot_suit", how="SEW", note="太ももの前端面へ接着")
    put(parts, seat * mp["mascot_bandana"], "mascot_bandana",
        to="mascot_head", how="SEW", note="三角巾。後ろで結ぶ")
    put(parts, seat * mp["mascot_badge"], "mascot_badge", to="mascot_suit",
        how="SEW", note="胸のゼッケン（チーム名を入れる）")
    put(parts, seat * mp["mascot_rag"], "mascot_rag", to="mascot_suit",
        how="CLAMP", note="両手のミトンで挟む")
    return parts


def link_base():
    """base_link のローカル形状（= ワールド、ポーズ0）。"""
    groups = {
        "base_frame": _frame_members(),
        "side_frame": _side_frames(),
        "deck": _decks(),
        "skirt": _skirt(),
        "drive_mount": _drive_mounts(),
        "fasteners": _fasteners(),
        "hopper": _hopper(),
        "feed_ramp": _feed_ramp(),
        "electronics": _electronics(),
        "sensors": _sensors(),
        "grabber_fixed": _grabber_fixed(),
        "mast": _mast(),
        "bucket": _bucket_group(),
        "chair": _chair_group(),
        # ⚠ 配線は**最後**に作る。端点を相手の面から決める（F.face）ので、
        #   参照する部品がすべて置かれた後でないと座標が取れない。
        "cables": _cables(),
    }
    # 自動配置のねじは**さらに後**。留める相手が全部置かれていないと
    # 固定宣言（put の to=）が解決できない。
    # ⚠ 空のグループを Compound にしてはいけない（children=[] の Compound は
    #   wrapped を持たず、親に付けた瞬間に AssertionError で落ちる）。
    auto = auto_screws([], "base")
    if auto:
        groups["auto_fasteners"] = auto
    children = []
    for name, parts in groups.items():
        labeled = []
        for i, part in enumerate(parts):
            # 既にラベルが付いている形状（bucket / mascot_envelope など）はそのまま残す
            if not (getattr(part, "label", "") or ""):
                label_shape(part, f"{name}_{i}")
            labeled.append(part)
        children.append(Compound(label=name, children=labeled))
    return Compound(label="base_link", children=children)


# ===========================================================================
# 可動リンク
# ===========================================================================
def link_wheel(hand: int, name: str = "fl", sy: int = 1):
    """車輪リンク（原点 = 車軸中心）。**ハブアダプタもここに置く。**

    ⚠ アダプタは車輪と一緒に回る。base_link 側（_drive_mounts）に描くと
      「回らないリンクの部品に、回る車輪をボルト留めしている」ことになり、
      リンクをまたぐ固定の検査に落ちる。実機でもその組み方はできない。
    """
    parts = []
    # ⚠ 逃げ穴 φ6 は M4 の**軸**用。そこへ来るのは頭（φ7、座面から 4mm）。
    #   端面 261 だと頭（Y 260..264）が 3mm 入り、片側 0.5mm の輪を
    #   31mm³ ×8 削る。穴を φ8 に広げると残肉 0.5mm で破断するので、
    #   ハブを頭の外へ逃がす。**接触判定のしきい値 0.5mm ちょうどでは
    #   「接触」になる**ので 1.5mm 空けて 265.5 に。軸の掛かり 16.3mm（1.6d）。
    # ⚠ **「4-M5 でハブアダプタへ」と宣言して、ボルトの立つ場所が無かった。**
    #   アダプタは φ44 の丸棒で、車輪のハブ（内径 φ44.4）にすきま嵌めで
    #   差さっているだけ。**面で当たる所がどこにも無い**ので、ねじの座面が
    #   取れない（`screw_place` が「接触面に両方の材料がある場所が無い」で
    #   4 輪とも落としていた）。
    #   → アダプタの**内側端に鍔（φ72 × t6）**を付け、車輪の内側ディスクの
    #     外面（軸方向 25mm）に当てる。ねじは内側から鍔を通し、ディスクへ
    #     ねじ込む（PCD60）。車輪は「アダプタごと組んでから軸に差す」小組
    #     なので、この向きで組める。
    hub_at = Pos(0, -sy * 23.5, 0) * Rot(90, 0, 0)
    # 鍔の局所 z。車輪の内側ディスクの外面（世界 |Y| = 25）に当てる
    z_flg = sy * (P.WHEEL_WIDTH / 2 - 23.5 + P.HUB_FLANGE_T / 2)
    put(parts, hub_at
        * (Cylinder(22.0, P.HUB_ADAPTER_LEN, align=CTR)
           + Pos(0, 0, z_flg) * Cylinder(P.HUB_FLANGE_DIA / 2,
                                         P.HUB_FLANGE_T, align=CTR)
           - Cylinder(P.M3508_SHAFT_DIA / 2, P.HUB_ADAPTER_LEN + 2, align=CTR)
           # モーター取付ねじ（4-M4 PCD35 → 半径 17.5）は外径 22 の中に入る。
           # 逃げ穴が無いと 4 本ともアダプタを貫く（115mm³ ×4）
           - reduce(lambda a, b: a + b,
                    [Pos(px, py, 0) * Cylinder(3.0, P.HUB_ADAPTER_LEN + 2, align=CTR)
                     for px, py in bolt_circle(4, P.M3508_BOLT_PCD, 45.0)])),
        f"hub_{name}", to=f"motor_{name}", how="SHAFT", note="φ10 出力軸に圧入 + 止めねじ")
    put(parts, Rot(90, 0, 0) * L.mecanum_wheel(hand), f"wheel_{name}",
        to=f"hub_{name}", how="BOLT", note="4-M5 ハブアダプタへ")
    # ⚠ `flange_screws` の `loc` は「**当たり面**が原点・+Z が工具を差す向き」。
    #   鍔の外面を原点にすると、座面が鍔の外へさらに t だけ出て宙に浮く。
    flange_screws(parts, hub_at * Pos(0, 0, z_flg - sy * P.HUB_FLANGE_T / 2)
                  * (Rot(0, 0, 0) if sy > 0 else Rot(180, 0, 0)),
                  f"hub_{name}", f"wheel_{name}", P.HUB_BOLT_PCD, 4, 5,
                  P.HUB_FLANGE_T, f"hub_{name}_", link=f"wheel_{name}")
    auto_screws(parts, f"wheel_{name}")
    return Compound(label="wheel", children=parts)


def link_turret_yaw():
    """ヨー旋回体（原点 = ヨー軸 × 台座上面 z=PEDESTAL_TOP_Z）。"""
    parts = []
    put(parts, Pos(0, 0, 10.0) * L.bearing(P.YAW_RING_DIA, P.YAW_RING_DIA - 40.0, 20.0),
        "yaw_ring", to=("pedestal_ring0", "pedestal_ring1"), how="ROTATE",
        note="V リング + カムフォロア 3 個")
    LEDGER.add("旋回支持（Vリング＋カムフォロア3個）", 0.350, "概算/カタログ", "砲塔")
    table_z = P.YAW_TABLE_Z - P.PEDESTAL_TOP_Z + P.YAW_TABLE_T / 2
    # 回転側の骨格。**円板にはできない**（射出ローラーが真ん中を通る）。
    # ローラーの占有域（ニップ軸から半径 91mm）の外、X=±150 に前後 2 本の
    # 梁を渡し、リングの上面と砲塔サイドプレートをつなぐ。
    for sx in (1, -1):
        tag = "f" if sx > 0 else "r"
        # ⚠ 丸穴の肉抜きをやめて、外形そのものを荷重の形に切る
        #   （`scripts/topo_opt.py`）。617g → 319g（r）/ 597g → 391g（f）。
        # ⚠ ヨー駆動の延長軸（Y=190、φ18）の逃げ穴は**もう手で開けない**。
        #   締結の宣言に無い相手なので、境界条件を作るときに
        #   「板と同じ場所を占める部品」として拾って逃げ穴になる。
        #   手で開けたままにすると、輪郭と二重に抜けて座面が消える。
        arm_pl = L.topo_plate(f"yaw_arm_{tag}", P.YAW_TABLE_T)
        arm = put(parts, Pos(sx * P.YAW_ARM_X, 0, table_z) * arm_pl,
                  f"yaw_arm_{tag}", to="yaw_ring", how="BOLT",
                  note="4-M5。リング上面へ")
        # ⚠⚠ **ここは長いあいだ手書きの 0.062kg で、実体と 10 倍ずれていた。**
        #   60×640×t6 のアルミ板はどう数えても 1 枚 600g で、桁を 1 つ落とした
        #   打ち間違いがそのまま残っていた（`plate_audit.py` の台帳照合が検出）。
        #   「直すと 35kg 規定を超えるので、アームを作り直すのとセットでしか
        #   直せない」と書いて据え置いていたが、**その作り直しがこれ**
        #   （トポロジー最適化で 617→320g / 597→413g、DESIGN.md §35）。
        #   ⚠ もう手書きに戻さない。実体から計算させる。総質量は +0.61kg
        #     増えるが、それは**元から積んでいた質量が見えるようになった**
        #     だけで、増えたわけではない。規定の帳尻は §7 の削減計画で合わせる。
        LEDGER.add_solid("旋回アーム A5052 t6 60×640（トポロジー最適化）",
                         arm, "A5052", "砲塔")
    # 大プーリはテーブルの**上**に載せる。リング（局所 0..20）の中に置くと重なる。
    # 大プーリはリングの外周に共締めする（中央はローラーが通るので使えない）
    # 内径は**ベアリング外径より大きく**すること。-4 では 2mm 食い込む（37,963mm³）
    # 台座プレート（〜842）とのすきまを 3mm 取る。回る側なので触れさせない
    # 台座プレート上面(842)と旋回アーム下面(862)のあいだ 20mm に、
    # 鍔込み 19mm の歯付きリングを収める（上下 0.5mm ずつ）
    # ⚠ 歯付きリングは**扇形**にする。全周だと -X 側で給送の通り道
    #   （|Y| 144..164 の帯）を塞ぐ。ヨーは ±30° しか回らないので、
    #   小プーリが噛む範囲（角度 22°〜82°）＋巻き付きぶんがあれば足りる。
    big = L.pulley(P.YAW_PULLEY_BIG, 15.0, P.YAW_RING_DIA)
    big -= Pos(-200.0, 0, 0) * Box(400.0, 600.0, 40.0, align=CTR)
    put(parts, Pos(0, 0, P.YAW_RING_H / 2) * big, "yaw_pulley_big",
        to="yaw_ring", how="BOLT", note="8-M4。リング外周に被せる歯付き扇形")
    # ⚠ **「8-M4」と書いて実体が 1 本しか入っていなかった。** 扇形は +X 側
    #   半分しか無いのに、全周のボルト円で探していたので大半が空を指す。
    #   しかも扇形とリングは**円筒面で接している**（プーリの内径 = リングの
    #   外径）ので、軸方向のねじはどちらか一方しか通らない。
    #   → 半径方向に打つ。頭は歯先より内側へ沈めたいので**皿ねじ**にし、
    #     座面を歯元（半径 173）に取る。扇形の角度範囲（±90°）の内側で
    #     8 本を等間隔に散らす。
    for i in range(8):
        ang = -70.0 + i * 20.0
        r_seat = P.YAW_PULLEY_BIG / 2 - 2.0
        yloc = Rot(0, 0, ang) * Pos(r_seat, 0.0, P.YAW_RING_H / 2) \
            * Rot(*_axis_rot((1.0, 0.0, 0.0)))
        grip = r_seat - P.YAW_RING_DIA / 2
        ray = tool_ray(LINK_LOC.get("turret", Location()) * Rot(0, 0, ang),
                       (r_seat, 0.0, P.YAW_RING_H / 2), (1.0, 0.0, 0.0))
        put_screw(parts, yloc * L.screw_flat(4, L.screw_len(grip, 6.0)),
                  f"scr_yawpul{i}", to=("yaw_pulley_big", "yaw_ring"),
                  how=("THRU", "SCREW_IN"), kind="FLAT", size=4,
                  length=L.screw_len(grip, 6.0), extras=(),
                  note="1-M4 皿（歯元へ沈める）", tool=ray)
    LEDGER.add("HTD5M 60T プーリ", 0.120, "カタログ", "砲塔")
    # 砲塔サイドプレート（仰角ユニットを軸支）
    # ⚠ **置くのは後ろへ回した**（この関数の末尾、仰角軸受ボスの直後）。
    #   肉抜きを荷重経路で決めるには、受け金具（brk_yaw）と荷重点
    #   （pitch_pivot・brk_lift）の位置が要る。板を先に置いていたときは
    #   相手がまだ無いので、保護する場所を手打ちするしかなかった。
    #   相手はどれも板の bbox を参照していないので、順番を入れ替えても
    #   位置は 1mm も動かない。
    dz = P.NIP_Z - P.PEDESTAL_TOP_Z
    # ⚠ 下端は旋回リングの上面（局所 20）まで下げる。26 のままだと
    #   昇降ガイドの受け金具が届く高さに板が無い。
    # ⚠ 上端は仰角軸（局所 dz）を超えていること。130 高では下が 9.5mm 浮き、
    #   上も軸に届かなかった。
    sp_h = dz - 20.0 + 30.0
    # ⚠ サイドプレート（X ±115）と旋回アーム（X 120..180）は **X で 5mm
    #   離れている**。「アームにボルト留め」と宣言していたが、板とアームは
    #   どこでも触れていない。アームの上面とプレートの内面をつなぐ金具を置く。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for sx in (1, -1):
            tg = "f" if sx > 0 else "r"
            # 板 1 = アームの上面に載る（法線 Z）、板 2 = プレートの内面に
            # 当たる（法線 Y）。2 枚は隅で重なって 1 部品になる。
            # ⚠ 金具は**両方に重なる X 範囲**を持つこと。プレート（±115）と
            #   アーム（YAW_ARM_X±30）は X で空いているので、その隙間を
            #   またぐ幅にする。アームを 150→160 に動かしたら幅 60 では
            #   プレートと点でしか接しなくなった（座面 0mm²）。
            # ⚠ 金具の内端は**仰角側板の外面より外**に収める。仰角側板は
            #   仰角軸から半径 165 で掃引し、この金具（軸から 108..132）は
            #   その円の中にある。内側へ 24mm 伸ばすと側板の |Y| 帯
            #   （300..303.5）に入り、仰角 20°/70° で 313〜348mm³ 削る。
            #   yaw_side の内面から 8mm（座面 60×8 = 480mm²）で足りる。
            # ⚠ **立ち上がりを 24 → 50 に伸ばす。** 板は下端の 24mm だけで
            #   3.4kg の仰角ユニットを 150mm 上から受けていた
            #   （`joint_load.py`: てこ比 8.2、ボルト 1 本に 207N の引張、
            #     片持ち比 5.8）。座面の半幅が 12 → 25 になり、てこ比は
            #   8.2 → 4.0、引張は 207N → 100N に落ちる。
            #   伸ばす向きは +Z（|Y| は 307.5..312.5 のまま）なので、
            #   仰角側板の掃引（|Y| <= 303.5）には入らない。
            # ⚠ **X 方向へは動かせない。** 板側の掛かりは 10mm しか無く
            #   （座面 10×50）、30mm に増やそうと内側へ 20mm 寄せると、
            #   昇降ガイドの受け金具（brk_lift、局所 X -112.5..-82.8 を
            #   斜めに上る板）に 316mm³ 食い込む。掛かりを増やすには
            #   brk_lift の逃げとセットで見直すこと。
            in_len = P.YAW_SIDE_IN_Y - (P.PITCH_SIDE_Y + P.PITCH_SIDE_T / 2) - 1.0
            brk_w = (P.YAW_ARM_X - 30.0) - 115.0 + 60.0   # 隙間 + 両側の掛かり
            brk_rise = 50.0
            # ⚠ **L 字の 1 部品にしない。** 自校ではアルミ板を曲げられない
            #   （`export_fab.CAN_BEND = False`）。水平板と立板を**別々の
            #   平板**にして、立板の**端面にタップ**を立てて留める
            #   （横穴加工まではできる）。板 2 枚 + M4 2 本になるが、
            #   曲げ品は 1 個も作れないので、これが唯一実物になる形。
            # ⚠ **-10.0 → -7.0（2026-08-05）。** 曲げをやめて立板を別部品に
            #   したあと、`yaw_side` を立板へ留める M5 のナットが
            #   仰角モーターへ **0.35mm** まで近づいた（ヨー30°・仰角70°。
            #   `sweep_fine` の「行程の途中で接する」）。
            #   ⚠ 旋回体と仰角ユニットは**相対運動する**ので、0.35mm では擦れる。
            #   実測: ねじ X 478.98..493.33 / モーター X 406.93..480.75 で
            #   X だけが分離軸（Y と Z は完全に重なる）。金具ごと 3mm 外へ
            #   出せば X で 1.2mm 離れ、すきまは 3.35mm になる。
            #   `yaw_side`（X 291..492）との座面は 3mm 動かしても十分残る。
            org = Pos(sx * (115.0 + brk_w / 2 - 7.0), sy * P.YAW_SIDE_IN_Y,
                      table_z + P.YAW_TABLE_T / 2)
            # ⚠ **立板は水平板の「上」に立てる（Y で並べない）。** 前は
            #   水平板 Y -in_len..0 / Z 0..5、立板 Y -5..0 / Z 0..50 で、
            #   2 枚が brk_w×5×5 = 1,875mm³ 重なっていた。
            #   ⚠ 水平板を 5mm 詰めて Y で突き合わせる案は**駄目**。
            #     `in_len` は 8mm しかない（仰角側板との隙間そのもの）ので、
            #     詰めると幅 3mm の短冊になり M5 の座金すら載らない。
            #   立板の下端を水平板の上面（Z 5）に載せ、水平板を貫く M4 を
            #   立板の**下の端面**へ立てる。
            # ⚠ **立板は t5 → t6。** t5 の小口には M4 のタップが立たない
            #   （`L.TAP_MIN[4] = 6.0`）。座面も brk_w×5 → brk_w×6 になる。
            #   側板が当たる外面（Y=0）は動かさない。
            flat = Pos(0, -sy * in_len / 2, 2.5) * Box(brk_w, in_len, 5.0,
                                                       align=CTR)
            # ⚠ **t6 では足りない（2026-08-07）。** 端面に立てる M4 の下穴は
            #   φ4.5 で、その外に **0.3d = 1.2mm** の壁が要る（= t6.9）。
            #   `TAP_MIN[4] = 6.0` は「ねじ山が何山掛かるか」の下限であって、
            #   側面が裂けないかは見ていない。t6 だと壁 0.75mm しか残らない。
            rise = Pos(0, -sy * 4.0, 5.0 + (brk_rise - 5.0) / 2) \
                * Box(brk_w, 8.0, brk_rise - 5.0, align=CTR)
            fp = put(parts, L.mat(org * flat, "A5052"),
                     f"brk_yaw_{sd}{tg}", to=f"yaw_arm_{tg}", how="BOLT",
                     note="2-M5")
            put(parts, L.mat(org * rise, "A5052"),
                f"brk_yaw_{sd}{tg}v", to=f"brk_yaw_{sd}{tg}", how="BOLT",
                # ⚠ **皿にする。** 立板は側板の内面（Y=0）に貼り付いているので、
                #   その真上に六角穴付きの頭が出ると側板をかすめる（実測
                #   `yaw_side_L` ↔ `scr_a0026` が 0.22mm）。水平板は t5 なので
                #   M4 の皿もみ（深さ 2.2mm）が入り、頭は面一になる。
                note="2-M4 皿（立板の下の端面にタップ。水平板は皿もみ）")
            LEDGER.add_solid("砲塔プレート受け金具 水平板 A5052 t5", fp, "A5052", "砲塔")
            LEDGER.add("砲塔プレート受け金具 立板 A5052 t8", 0.020, "体積概算", "砲塔")
    # サイドプレートを結ぶ横梁
    # ⚠ 砲塔横梁は X=-60 に置いていたが、そこは**ローラーの通り道**（|X|<=91）。
    #   下ローラーと 5 組・上ローラーと 5 組が重なっていた。旋回アームが
    #   同じ役目（サイドプレートを結ぶ）を X=±150 で果たすので横梁は廃止する。
    L.add_ext("砲塔横梁", 0.0, "砲塔", qty=0)
    # --- 昇降コンベア（斜路 → ニップ）------------------------------------
    # ⚠ 平らな「漏斗」では届かない。斜路の出口(805)からニップ(1000)まで
    #   **195mm 上る**必要がある。板を置いても布は上らない。
    #   旋回体と一緒に回る挟み込みベルトで持ち上げる。
    #   （以前の漏斗は板 1 枚で、しかも全長が下ローラーの占有域の中にあった）
    lx0, lx1 = P.LIFT_X0, P.LIFT_X1
    lz0, lz1 = P.LIFT_Z0 + dz, P.LIFT_Z1 + dz
    lift_len = hypot(lx1 - lx0, lz1 - lz0)
    lift_ang = degrees(atan2(lz1 - lz0, lx1 - lx0))
    # ⚠ 板を短くしてプーリを逃がすと、今度は**軸受座が板に届かない**
    #   （4 か所とも「板にボルト留め」と宣言しながら浮いていた）。
    #   板は端まで張り、プーリの位置に逃げ穴を開ける。
    # ⚠ 逃げ穴の位置は**プーリの位置と同じ式**で出すこと。片方だけ
    #   斜面に沿った寄せ方（step）に直すと、穴とプーリが 420mm³ ずれる。
    slope0 = (lz1 - lz0) / (lx1 - lx0)
    step0 = 14.0 / hypot(1.0, slope0)
    ends0 = (("lo", lx0 + step0, lz0 + step0 * slope0),
             ("hi", lx1 - step0, lz1 - step0 * slope0))
    guide = (Pos((lx0 + lx1) / 2, 0, (lz0 + lz1) / 2) * Rot(0, -lift_ang, 0)
             * Box(lift_len, P.LIFT_W, 1.5, align=CTR))
    for _tag, _px, _pz in ends0:
        for _sy in (1, -1):
            # 逃げるのは**プーリ**（|Y|=180、φ22+鍔）。軸受座（|Y|=246）の
            # ところは板を残す。そこが座面になる。
            # ⚠ プーリの位置は板の中心線ではなく、そこから逃がした先。
            #   lo は (+4, +26)、hi は法線方向に +26。穴もそこへ開ける。
            if _tag == "lo":
                _ox, _oz = _px + 4.0, _pz + 26.0
            else:
                _ox = _px - 26.0 * sin(radians(lift_ang))
                _oz = _pz + 26.0 * cos(radians(lift_ang))
            guide -= Pos(_ox, _sy * 180.0, _oz) * Rot(90, 0, 0) * Cylinder(
                20.0, 70.0, align=CTR)
    # ⚠ 駆動プーリ（|Y| 47..61）とベルトの逃げ。lo 側の軸は板中心面から
    #   8.12mm しかないので、鍔 φ30 が板を弦 25mm で横切る。
    guide -= Pos(ends0[0][1] + 4.0, 54.0, ends0[0][2] + 26.0) * Rot(90, 0, 0) \
        * Cylinder(20.0, 40.0, align=CTR)
    # ⚠ 旋回リング（半径 140..160、Z 842..862）を横切る帯だけは板を抜く。
    #   600mm 幅の板はリングの内径（Ø280）を通れない。布は |Y|=180 の
    #   挟み込みベルトが運ぶので、中央の板は無くてよい。
    # ⚠ ここは**旋回体の局所座標**。世界座標 852 をそのまま書くと、
    #   開口が Z=1694 に開いて何の役にも立たない。dz を引いて局所へ直す。
    guide -= Pos(0, 0, 852.0 - P.PEDESTAL_TOP_Z) * Box(
        400.0, 2 * 170.0, 34.0, align=CTR)
    # ⚠ 板の下端は**台座横梁（Z 818..838）より上**で止める。下端は傾斜の
    #   まま world 834.6 まで下がっていて、ヨー -5..-20° で |Y| 148..258 の
    #   帯が横梁に 3.4mm 潜っていた（329mm³。ヨー 10° 刻みの標本では
    #   -10° の 284mm³ すら見ておらず、連続掃引で発覚）。
    #   ⚠ リングの開口（|Y| <= 170）を下へ広げても消えない。潜っているのは
    #     その**外側**（|Y| 148..258）で、旋回すると 半径 231〜250 の帯が
    #     梁の上を通る。下端を水平に切り落とすのが正しい。
    #     斜面に沿って 5.3mm 短くなるだけで、端部プーリは 14mm 内側なので
    #     プーリ・軸受座には影響しない。
    guide -= Pos(0, 0, (839.0 - P.PEDESTAL_TOP_Z) - 100.0) * Box(
        600.0, 2 * 300.0, 200.0, align=CTR)
    # 砲塔系ハーネスの逃げ穴（φ30）
    guide = guide - Pos(P.CABLE_THRU_X - P.TURRET_X, 0, lz0 + 20.0) * Cylinder(
        P.CABLE_THRU_DIA / 2, 400.0, align=CTR)
    # ⚠ ガイド板は |Y|<=281（仰角ウォームを避けて 562 幅に絞った）、砲塔
    #   サイドプレートの内面は 312。31mm 空いているので「側板にリベット」
    #   では留まらない。板の裏に沿う受け金具でその 31mm を渡す。
    put(parts, guide, "lift_guide",
        to=tuple(f"brk_lift_{sd}0" for sd in ("L", "R")),
        how="RIVET")
    gc = ((lx0 + lx1) / 2, (lz0 + lz1) / 2)
    gn = (-sin(radians(lift_ang)), cos(radians(lift_ang)))    # ガイド面の法線
    for syg in (1, -1):
        sdg = "L" if syg > 0 else "R"
            # ⚠ 受け金具は**仰角ユニットの掃引の外**に置く。仰角側板は
            #   仰角軸から半径 161 の円内を掃く。板の上半分に置くと、
            #   仰角を振ったとき側板と 789mm³ 交差する。
            #   ガイド板の上端はニップの入口＝仰角ユニットの内側なので、
            #   そもそも支持できない。下半分の 2 点で片持ちにする。
        #   さらに、留める先の砲塔サイドプレートは下端が局所 20 なので、
        #   掃引の外かつ側板の高さに収まる位置は板の中央やや下の 1 点しか
        #   ない。板は t1.5 と軽いので片持ちで受ける。
        for k, sg in enumerate((-0.15,)):
            gl = sg * lift_len
            gx = gc[0] + gl * cos(radians(lift_ang)) - 2.75 * gn[0]
            gz = gc[1] + gl * sin(radians(lift_ang)) - 2.75 * gn[1]
            # ⚠⚠ **未解決**: この金具は |Y| 281（ガイド板の端）から 312.5
            #   （砲塔サイドプレートの内面）まで渡すので、あいだにある
            #   仰角側板のスラブ（300.5..303.5）を必ずまたぐ。仰角 20° で
            #   138mm³ ×2 交差する。
            #   仰角側板は旋回体の内側（|Y|<300.5）と外側（>303.5）を
            #   分断しているので、ガイド板を旋回体へ留める経路は
            #   「側板の掃引の外（Z 局所 9.3 未満）を回り込む」しかない。
            #   実機では金具に切り欠きを入れて逃がす。CAD 上の再現は
            #   L 字 + 脚の作り込みが必要なので次の課題とする。
            # ⚠ **ガイド板の縁ではなく裏面へ留める（2026-08-05）。**
            #   前は |Y| 281（ガイド板の端）から始めていたので、金具と
            #   ガイド板は**板の縁どうし**で突き合っていた。`screw_place` は
            #   接触の法線を Y に取り、リベット軸がガイド板の長手方向
            #   （562mm）を向く。深さ 30mm を測って **φ3.2×43mm** という
            #   買えないリベットが生まれ、その尾がヨー 9.5° で台座横梁へ
            #   46mm³ 食い込んでいた（`sweep_fine`）。
            #   t1.5 の板の縁にリベットは打てない。**内側へ 15mm 伸ばして
            #   ガイド板の裏に重ね、面でリベット留めする。**
            #   金具はすでに法線方向へ 2.75mm（= t1.5/2 + t4/2）オフセット
            #   してあるので、重ねてもガイド板を食わない。
            LAP = 15.0
            gw = P.YAW_SIDE_IN_Y - P.LIFT_W / 2 + LAP
            # ⚠ 長さ 80 では**斜面の上端が仰角側板のスラブに 6mm³ 入る**
            #   （ヨー ±30°・仰角 20° のときだけ。連続掃引で検出）。
            #   重なりは +X 端の角（X 318.4..321.7 / Z 887.5..890.3）だけ
            #   なので、上端を 14mm 詰めて 66 にする（下端は動かさないよう
            #   中心を斜面に沿って 7mm 下げる）。上端は Z 891.2 → 879.5。
            gcut = 7.0
            # ⚠ 下端も**台座横梁（Z 818..838）より上**で止める。斜面のまま
            #   world 822 まで下がっていて、ヨー -7..+7° で梁に 1,221mm³
            #   潜っていた（斜路支柱にも 25mm³）。ガイド板と同じく world 839
            #   で水平に切る。斜面に沿って 20mm 短くなるが、残る 46mm に
            #   2-M4 は入る。
            put(parts, L.mat((Pos(gx - gcut * cos(radians(lift_ang)),
                                  syg * (P.LIFT_W / 2 - LAP + gw / 2),
                                  gz - gcut * sin(radians(lift_ang)))
                              * Rot(0, -lift_ang, 0)
                              * Box(80.0 - 2 * gcut, gw, 4.0, align=CTR))
                             - Pos(0, 0, (839.0 - P.PEDESTAL_TOP_Z) - 100.0)
                             * Box(600.0, 2 * 400.0, 200.0, align=CTR), "A5052"),
                # ⚠ ガイド板との締結は**`lift_guide` の側で宣言済み**
                #   （`to=brk_lift_*0, how=RIVET`）。ここでも書くと
                #   相互宣言＝固定の循環になる。金具の親は側板だけ。
                f"brk_lift_{sdg}{k}", to=f"yaw_side_{sdg}", how="BOLT",
                note="2-M4")
            LEDGER.add("昇降ガイド受け金具 A5052 t4", 0.017, "体積概算", "砲塔")
    # ⚠ 仰角の軸受ボス。砲塔サイドプレート（|Y| 312..316）と仰角側板
    #   （300..304）のあいだ 8mm を埋める。**回らない側（旋回体）の部品**
    #   なのでここに置く。穴＝逃げ、ボス＝荷重を渡すもの。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # 板と板のあいだ（|Y| 304..312）にちょうど収める。幅 12 だと両側に
        # 2mm ずつ食い込む（785 + 224mm³）
        # ⚠ ボス幅は**板と板のあいだの実寸**。板厚を変えると隙間も変わる。
        #   8 のままだと両側に 0.5mm ずつ空き、「軸受ボス」なのに
        #   どちらの板にも当たっていない状態になる。
        gap_pv = P.YAW_SIDE_IN_Y - (P.PITCH_SIDE_Y + P.PITCH_SIDE_T / 2)
        put(parts, L.mat(Pos(0, sy * (P.YAW_SIDE_IN_Y - gap_pv / 2), dz)
                         * Rot(90, 0, 0)
                         * (Cylinder(15.0, gap_pv, align=CTR)
                            - Cylinder(10.0, gap_pv + 2, align=CTR)), "A5052"),
            f"pitch_pivot_{sd}", to=f"yaw_side_{sd}", how="BOLT", note="4-M4 軸受ボス")
    LEDGER.add("仰角軸受ボス A5052", 0.011, "体積概算", "砲塔", qty=2)

    # --- 砲塔サイドプレート（受けと荷重点が揃ったので、ここで作る）--------
    # この板は**面内のせん断で持つ板**（下端 2 か所の受け金具から、仰角軸と
    # 昇降ガイドの受けまで力を流すだけ）。等間隔の丸穴で抜くと、力の通る
    # 帯を削って通らない隅を残す。`plate_audit.py` の使用率は 22% だった。
    # → `L.lighten_path` で荷重経路（受け ↔ 荷重点を結ぶ帯）を残して抜く。
    cz = 20.0 + sp_h / 2                     # 板の中心（旋回体局所 Z）
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"

        def _discs(nm_, pad: float = 3.0):
            """すでに置いた部品の座を、板の局所座標 (x=世界X, y=世界Z−cz) の
            保護円**の列**にする。

            ⚠ 座標は手で書かない。相手の bbox から取れば、相手が動いても
              保護円が追従する（手打ちすると座面の真ん中に穴が開く）。
            ⚠ 長方形の座を**1 個の円**で囲むと、対角線の半分が半径になって
              座の 2 倍近くを守ってしまう（75×27 の金具で r=40）。実際
              それで肉抜きが 102g → 68g に**減った**（等間隔より重くなった）。
              短辺で決めた円を長辺方向に並べて、座の形に沿わせる。
            """
            b = F.BOX[nm_]
            x0, x1 = b.min.X, b.max.X
            y0, y1 = b.min.Z - cz, b.max.Z - cz
            r = min(x1 - x0, y1 - y0) / 2 + pad
            n = max(1, int(max(x1 - x0, y1 - y0) / max(r, 1.0)))
            out = []
            for i in range(n):
                u = (i + 0.5) / n
                out.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, r)
                           if (x1 - x0) >= (y1 - y0) else
                           (x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, r))
            return out

        # ⚠ **帯を張る点は部品ごとに 1 つ**にする。保護円を全部そのまま
        #   経路の端点に使うと、8 個 × 10 個 = 80 本の帯が扇状に広がって
        #   板全体が「経路」になり、肉抜きが 1 つも入らなくなる。
        #   経路は重心どうし、保護は座の形どおり、と役割を分ける。
        def _one(nm_):
            b = F.BOX[nm_]
            return ((b.min.X + b.max.X) / 2, (b.min.Z + b.max.Z) / 2 - cz,
                    min(b.max.X - b.min.X, b.max.Z - b.min.Z) / 2)

        brk_lifts = [n_ for n_ in sorted(F.BOX) if n_.startswith(f"brk_lift_{sd}")]
        anchors = [_one(f"brk_yaw_{sd}{t_}") for t_ in ("f", "r")]
        # 荷重点と、そこに掛かる力 [N]。仰角軸は仰角ユニット 3.4kg を
        # 動的 3 倍で受ける（左右 2 枚で分けるので 1 枚 50N）。昇降ガイドは
        # ベルト張力とガイド自重で 20N 見当。
        loads = [(*_one(f"pitch_pivot_{sd}"), 50.0)]
        loads += [(*_one(n_), 20.0) for n_ in brk_lifts]
        keep = [d for t_ in ("f", "r") for d in _discs(f"brk_yaw_{sd}{t_}")]
        keep += _discs(f"pitch_pivot_{sd}")
        keep += [d for n_ in brk_lifts for d in _discs(n_)]
        # ⚠ 上の `_discs` / `_one` / `anchors` / `loads` は**もう使っていない**。
        #   丸穴を並べる方式（`L.lighten_path`）をやめて、外形そのものを
        #   荷重の形に切るようにした（`scripts/topo_opt.py`）。境界条件は
        #   ここで組むのではなく `tr_fix` の宣言から自動で作られる。
        #   残してあるのは、比較のために元の方式へ戻せるようにするため。
        #   233g → 93g（-60%）。DESIGN.md §35。
        sp_plate = L.topo_plate(f"yaw_side_{sd}", P.YAW_SIDE_T)
        sp = put(parts, Pos(0, sy * P.YAW_SIDE_Y, cz) * Rot(90, 0, 0) * sp_plate,
                 f"yaw_side_{sd}",
                 # ⚠ 留まるのは**立板**（`…v`）のほう。サイドプレートも立板も
                 #   板厚が Y なので面で合う。水平板は Y に厚みが無いので、
                 #   そこへ「ボルト」と書くと端面 5mm に留めることになる。
                 to=tuple(f"brk_yaw_{sd}{t_}v" for t_ in ("f", "r")), how="BOLT",
                 note="各 2-M5")
        LEDGER.add_solid("砲塔サイドプレート A5052 t3（トポロジー最適化）", sp,
                         "A5052", "砲塔")
    LEDGER.add_solid("昇降コンベア ガイド板 A5052 t1.5", guide, "A5052", "砲塔")
    slope = (lz1 - lz0) / (lx1 - lx0)
    # ⚠ 端部プーリを板の端から内側へ寄せる量は**斜面に沿った距離**で
    #   取る。X 方向に 14 動かすと、傾斜 63.4° では斜面に沿って 31.3mm
    #   （14·√(1+slope²)）進んでしまう。板を 96mm に縮めたとき、
    #   プーリ中心間が 96 − 62.6 = 33.4mm になって φ44 のプーリ 2 個が
    #   1,261mm³ ×2 重なった。
    step = 14.0 / hypot(1.0, slope)
    ends = (("lo", lx0 + step, lz0 + step * slope),
            ("hi", lx1 - step, lz1 - step * slope))
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        for tag, px, pz in ends:
            # ⚠ 逃げは**ガイド面に垂直**に取る。鉛直に +24 しても、傾斜 62° では
            #   面までの距離が 24·cos62° = 11mm しかなく、φ32 のプーリが板を貫く。
            # ⚠ 逃げの向きは**旋回アーム（X 55..115）から離れる側**に取る。
            #   -X 側に出すと下端プーリがアームに 3,984mm³ 食い込む。
            # ⚠ 逃げは面に垂直に取るが、傾斜 63° では **-X 方向に 23mm** 動く。
            #   下端プーリが旋回アーム（X 55..115）へ戻ってしまう。
            #   下端だけは X を戻さず、Z だけ上げてアームの上面（868）を越す。
            if tag == "lo":
                ppos = Pos(px + 4.0, sy * 180.0, pz + 26.0)
            else:
                ppos = Pos(px - 26.0 * sin(radians(lift_ang)), sy * 180.0,
                           pz + 26.0 * cos(radians(lift_ang)))
            # ⚠ 内径 6 に軸 φ5.2 では 0.4mm 空く。「圧入」にならない。
            # ⚠ 胴の幅はベルト幅（60）より広くする。60 のままだと鍔が
            #   ベルトの縁に食い込む。64 で片側 2mm 空ける。
            put(parts, ppos * Rot(90, 0, 0)
                * L.pulley(22.0, 64.0, 5.2),
                f"lift_pul_{sd}_{tag}", to=f"lift_shaft_{tag}{sd}", how="PRESS")
        # ベルトは 2 つのプーリを結ぶ。下端だけ逃げ方を変えたので、
        # ベルトも両端のプーリ中心を結ぶ線に合わせる
        p_lo = (ends[0][1] + 4.0, ends[0][2] + 26.0)
        p_hi = (ends[1][1] - 26.0 * sin(radians(lift_ang)),
                ends[1][2] + 26.0 * cos(radians(lift_ang)))
        b_ang = degrees(atan2(p_hi[1] - p_lo[1], p_hi[0] - p_lo[0]))
        # ⚠ 平らな板 1 枚で描いていたので、プーリ（φ22）の中心を通って
        #   1,014mm³ ×4、端部軸も貫いていた。プーリに巻き付くスタジアム形に
        #   直すと重なりは 0 になる（宣言 MESH の陰に隠れていた分）。
        put(parts, L.mat(
            Pos((p_lo[0] + p_hi[0]) / 2, sy * 180.0, (p_lo[1] + p_hi[1]) / 2)
            * Rot(90, 0, 0) * Rot(0, 0, b_ang)
            * L.belt_loop(hypot(p_hi[0] - p_lo[0], p_hi[1] - p_lo[1]),
                          11.0, 2.0, 60.0), "RUBBER"),
            f"lift_belt_{sd}",
            to=(f"lift_pul_{sd}_lo", f"lift_pul_{sd}_hi"), how="MESH",
            note="端部プーリに巻き付く")
    # ⚠ 通し軸は旋回リング（半径 140..160、Z 842..862）の帯を横切る。
    #   ベルトのある |Y| 148..212 だけを支える**スタブ軸**にすれば、
    #   リングの内径側（|Y| < 140）には何も通らない。
    for tag, px, pz in ends:
        for sy in (1, -1):
            sd = "L" if sy > 0 else "R"
            bx = (px + 4.0) if tag == "lo" else (px - 26.0 * sin(radians(lift_ang)))
            bz = (pz + 26.0) if tag == "lo" else (pz + 26.0 * cos(radians(lift_ang)))
            # 軸受座は**ブッシュ**。内径を軸径にして軸を受ける
            # ⚠ 軸はガイド板の中心線から 26mm 離れたところにある（プーリを
            #   逃がすため）。ボスだけ置くと板に届かないので、板まで届く
            #   脚を一体に付ける。「板にボルト留め」と宣言するなら、
            #   板に当たる面をその部品が持っていること。
            dxl, dzl = px - bx, pz - bz
            lnl = hypot(dxl, dzl)
            angl = degrees(atan2(dzl, dxl))
            # ⚠ 脚は板の**裏面**で止める。板の中心線まで伸ばすと貫くが、
            #   斜めに伸ばすので「経路長 − 板厚/2」では合わない。
            #   板の**法線方向の距離**から長さを出す。
            gnx, gnz = -sin(radians(lift_ang)), cos(radians(lift_ang))
            ndist = abs(dxl * gnx + dzl * gnz)
            # ⚠ 脚が板面と斜めに交わるとき、**箱の半厚が法線方向に
            #   1.5·cosφ だけ張り出す**（φ = 経路と板面のなす角）。
            #   lo 側は φ=18° しかないので 1.43mm 張り出して板を削る。
            #   定数で引くと hi 側（φ=90°、張り出し 0）が浮くので、
            #   角度から出すこと。
            extra = 1.5 * sqrt(max(0.0, 1.0 - (ndist / lnl) ** 2)) if lnl > 1e-6 else 0.0
            lnl2 = (lnl * (ndist - 0.75 - extra) / ndist
                    if ndist > 1e-6 else lnl)
            # ⚠ 脚は軸受座の**外周**（半径 9）から始める。中心から始めると
            #   穴の中を通る軸と 117mm³ 重なる。
            leg_l = max(lnl2 - 6.5, 1.0)
            leg_c = (6.5 + leg_l / 2) / lnl
            # 脚は板と平行な面を持たせる（角が板を削らないよう薄くする）
            brg_leg = (Pos(dxl * leg_c, 0, dzl * leg_c)
                       * Rot(0, -angl, 0)
                       * Box(leg_l, 12.0, 3.0, align=CTR))
            # ⚠ **脚は端面（12×3 ＝ 36mm²）でしか板に当たっていなかった。**
            #   「2-M3 の軸受座」と宣言しても、ボルト 2 本の座面がどこにも
            #   無い（`screw_place` が 4 組とも「接触面に両方の材料がある
            #   場所が無い」で落としていた）。板と平行な**足**を付けて
            #   座面を作る。足の内面は板の裏面（法線 0.75mm）に合わせ、
            #   厚み 3 で脚の端（lo 側は extra ぶん手前で止まる）を確実に
            #   飲み込む（離すと 2 つのソリッドに分断される）。
            gnx_, gnz_ = -sin(radians(lift_ang)), cos(radians(lift_ang))
            brg_leg = brg_leg + (Pos(dxl + 2.25 * gnx_, 0, dzl + 2.25 * gnz_)
                                 * Rot(0, -lift_ang, 0)
                                 * Box(20.0, 12.0, 3.0, align=CTR))
            # ⚠ ボス半径の上限は**板中心面までの法線距離 − 板厚/2**。
            #   lo 側の逃げは (+4,+26) で法線ではないので、余裕は 26 では
            #   なく (4,26)·n = 8.12mm しかない。r9 だと t1.5 の板を
            #   局所的に切断していた（112mm³ ×2）。
            put(parts, L.mat(Pos(bx, sy * 246.0, bz)
                             * (Rot(90, 0, 0)
                                * (Cylinder(6.5, 10.0, align=CTR)
                                   - Cylinder(2.7, 12.0, align=CTR))
                                + brg_leg), "A5052"),
                f"lift_brg_{tag}{sd}", to="lift_guide", how="BOLT", note="2-M3 の軸受座")
            # 足 → ガイド板（t1.5 なのでタップは立たない。貫通させてナット）
            for i_, du_ in enumerate((-6.0, 6.0)):
                sx_, sz_ = cos(radians(lift_ang)), sin(radians(lift_ang))
                seat = (bx + dxl + 3.75 * gnx_ + du_ * sx_, sy * 246.0,
                        bz + dzl + 3.75 * gnz_ + du_ * sz_)
                face_screws(parts, [Pos(*seat) * Rot(0, -lift_ang, 0)],
                            f"lift_brg_{tag}{sd}", "lift_guide", 3, 0.0,
                            f"lift_brg_{tag}{sd}_{i_}", link="turret",
                            how_b="THRU", length=8.0,
                            extras=(("HEXNUT", 3), ("WASHER", 3)))
            # ⚠ 駆動プーリ（|Y|=74）が付く L 側の下端軸だけは、そこまで
            #   届かせる。195±56 では 65mm 手前で終わっていた。
            drive = (tag == "lo" and sy > 0)
            sh_y, sh_l = ((145.5, 211.0) if drive else (195.0, 112.0))
            put(parts, L.mat(Pos(bx, sy * sh_y, bz) * Rot(90, 0, 0)
                             * Cylinder(2.6, sh_l, align=CTR), "STEEL"),
                f"lift_shaft_{tag}{sd}", to=f"lift_brg_{tag}{sd}", how="ROTATE",
                note="フランジブッシュ")
    LEDGER.add("昇降コンベア 軸受座 A5052", 0.006, "体積概算", "砲塔", qty=4)
    # ⚠ モーターの置き場所は**旋回リングの内径の中（半径 138 未満）**しかない。
    #   軸の延長線上（|Y|>316）には砲塔サイドプレートと側面上桁があり、
    #   内側（|Y| 245..285）は射出ローラーが占めている。残るのは
    #   ガイド板の下・リングの穴の中で、そこは旋回しても固定物に当たらない。
    lo_x, lo_z = ends[0][1], ends[0][2] + 20.0
    # ⚠ ガイド板の**背面（-X 側）**かつ旋回リングより上に置く。
    #   前面側・リングより下はマスコット規定外形と旋回リングが占めている。
    # 下端プーリの真下・旋回リングの内径の中（半径 138 未満）。
    # 前へ出すと射出ローラーに当たり、下げるとマスコット規定外形に当たる。
    # ⚠ -X 側には台座横梁（X 75..95）と斜路の上端軸がある。
    #   リング内径（半径 138 未満）の **+X 寄り・ガイド板の裏** に置く。
    # ⚠ 以前は lo_z-58（台座プレートより 37mm 下）に置いていた。そこは
    #   旋回体の外＝固定側の空間で、旋回すれば必ず当たる。
    #   ガイド板の裏（-X）かつ台座より上に上げる。
    # ⚠ 駆動系は**旋回リングの帯（半径 140..160）を避ける**。X 方向に
    #   40mm 逃がすと、ベルトが半径 160 に達してリングを 691mm³ 削り、
    #   旋回アーム（Z 20..26）にも 180mm³ 入る。
    #   同じ X のまま Z を上げれば、半径は 106 に収まる。
    # ⚠ +76 だと world Z 900..937 に来て、仰角 20° に下がってきた
    #   ニップ入口ガイド（Z 906..950）と 4,174mm³ 重なる。モーターは
    #   コンベアの**下端寄り**に置く（下端プーリの近くならベルトも短い）。
    # ⚠⚠ **未解決の機構課題**: この駆動系を置ける空間が無い。
    #   3 つの制約が同時に満たせない:
    #     (1) ニップ入口ガイドの掃引を避ける → モーター上端 < world 906
    #     (2) 駆動プーリから軸間距離を取る → 中心間 ≥ 18+15 = 33mm
    #     (3) 旋回リングの内径に収める → 軸から半径 < 140mm
    #   Z 方向: プーリ（Z 868.8）から 33mm 上げると上端 919.8 で (1) に反する
    #   X 方向: プーリ（X -98）から 30mm ずらすと外周が半径 151 で (3) に反する
    #   Y 方向: ベルトが掛からない
    #   → モーターを旋回リングの**外**（半径 > 160）へ出してベルトを
    #     長くするか、ウォーム減速にしてモーターを寝かせるかの作り直しが
    #     要る。いまは (1) を捨てて +76 に置いている（仰角 20° で
    #     4,174mm³ 干渉）。
    # ⚠ +76 では取付板の上端が world 942.7 に来て、仰角 60° に下がってきた
    #   ニップ入口ガイド（下面 940.5）の隅に 17mm³ 入る（ヨー ±30°）。
    #   板を小さくすると 4-M3 PCD26 の縁が薄くなるので **モーターごと
    #   3mm 下げる**。駆動プーリとの中心間は 50 → 47mm で (2) は満たす。
    # ⚠ 3mm では足りなかった。仰角 60 / 55 / 70 の 3 点で 0 を確認して
    #   済ませたが、連続掃引が **61.5°** で 7mm³ を見つけた。
    #   点で確かめて「直った」と言ってはいけない。6mm 下げ、仰角 20..70 を
    #   0.5° 刻みで走査して 0 を確認する（中心間 44mm で下限 33 は満たす）。
    # ⚠ 70 でも仰角 70°・ヨー -30° で **0.07mm まで詰まる**（重なりは 0 だが
    #   擦れる）。連続掃引の「宣言の無い組が途中で接する」で出た。
    #   66 にして 4mm 空ける。駆動プーリとの中心間は 40mm（下限 33 は満たす）。
    mtr = (ends[0][1] + 4.0, 40.0, ends[0][2] + 66.0)
    # 取付板はモーターの取付面（Y=60）に接する板 + ガイド板まで届く脚。
    # ⚠ 「4-M4 + スペーサ」と注記して実体が無く、板は 36mm 浮いていた。
    tpar = ((mtr[0] - gc[0]) * cos(radians(lift_ang))
            + (mtr[2] - gc[1]) * sin(radians(lift_ang)))
    near = (gc[0] + tpar * cos(radians(lift_ang)),
            gc[1] + tpar * sin(radians(lift_ang)))
    dxm, dzm = near[0] - mtr[0], near[1] - mtr[2]
    lnm = hypot(dxm, dzm)
    angm = degrees(atan2(dzm, dxm))
    # ⚠ 脚は板の裏面で止める（経路長 − 板厚/2 では斜めのぶん合わない）
    gnx2, gnz2 = -sin(radians(lift_ang)), cos(radians(lift_ang))
    nd2 = abs(dxm * gnx2 + dzm * gnz2)
    #   `near` は垂線の足なので nd2 == lnm。脚の軸は板法線と一致し、
    #   端面は板と平行なので角の張り出しは無い。余分に引くと脚が浮く
    #   （-3.0 のとき 3mm 浮いていて、板を刺していた隅だけで接触が
    #   成立していた）。
    lnm2 = lnm * (nd2 - 0.75) / nd2 if nd2 > 1e-6 else lnm
    # ⚠ 脚を板と同じ |Y| 帯に置くと、モーター本体（Y -3..60）へ 2,654mm³
    #   食い込む。脚は板の**外側**（Y 64..72）へ寄せる。
    #   板そのものにも軸（φ8）の逃げ穴が要る。
    put(parts, L.mat(Pos(mtr[0], 42.0, mtr[2])
                     # ⚠ 板の軸は世界 X-Z だが相手の面は 63° 傾いている。
                     #   70 角だと隅の法線方向到達距離が 35·(sin+cos) = 47mm
                     #   になり、中心の余裕 30.6mm を **16.4mm 越える**。
                     #   「中心が 30mm 離れているから 70 角でも入る」は誤り。
                     #   h·1.343 ≤ 29.9 → h ≤ 22.2（44 角が限界）なので 40 角。
                     * (Rot(90, 0, 0)
                        * (L.plate(40.0, 40.0, 4.0)
                           - Cylinder(6.0, 8.0, align=CTR))
                        # ⚠ 脚は板と同じ Y 帯に。+4 だとモータープーリの
                        #   鍔（Y 47..49）と 1mm 被る。
                        # ⚠ 内側端は軸の外（半径 6）から。中心から生やすと
                        #   出力軸 φ8 を 74mm³ 削る。
                        + Pos(dxm * (6.0 + lnm2) / (2 * lnm), 0.0,
                              dzm * (6.0 + lnm2) / (2 * lnm))
                        * Rot(0, -angm, 0)
                        * Box(lnm2 - 6.0, 4.0, 5.0, align=CTR)
                        # ⚠ **脚は端面（4×5 ＝ 20mm²）でしか板に当たって
                        #   いなかった。**「ガイド板の裏へ 4-M4」と宣言しても
                        #   ボルトの座面がどこにも無い。板と平行な足を付ける。
                        #   足の内面は板の裏面（法線 0.75mm）、厚み 3 で
                        #   脚の端を飲み込む（離すとソリッドが分断される）。
                        # ⚠ 足は **Y に広げる**。斜面に沿って振ると、
                        #   ガイド板の穴（駆動プーリの逃げ φ40 と旋回リングの
                        #   開口）に当たって片方のねじが板に届かない
                        #   （実測: 斜面 +10 のねじが「離れ」になった）。
                        #   どちらの穴も **XZ 面の形**なので、Y に振れば
                        #   脚が当たっている所と同じ材料の上に乗る。
                        + Pos(dxm + 2.25 * gnx2, 0.0, dzm + 2.25 * gnz2)
                        * Rot(0, -lift_ang, 0)
                        * Box(30.0, 28.0, 3.0, align=CTR)), "A5052"),
        # ⚠ 「4-M4」は入らない。足は Y 16mm 幅なので M4（頭 φ7）は 1 列、
        #   斜面方向に 2 本。
        "lift_mot_brk", to="lift_guide", how="BOLT", note="ガイド板の裏へ 2-M4")
    for i_, dy_ in enumerate((-9.0, 9.0)):
        seat = (mtr[0] + dxm + 3.75 * gnx2, 42.0 + dy_,
                mtr[2] + dzm + 3.75 * gnz2)
        face_screws(parts, [Pos(*seat) * Rot(0, -lift_ang, 0)],
                    "lift_mot_brk", "lift_guide", 4, 0.0, f"lift_motbrk_{i_}",
                    link="turret", how_b="THRU", length=8.0,
                    extras=(("HEXNUT", 4), ("WASHER", 4)))
    lm_at = Pos(*mtr) * Rot(-90, 0, 0)
    put(parts, lm_at * L.m2006(),
        "lift_motor", to="lift_mot_brk", how="BOLT", note="4-M3 PCD26")
    flange_screws(parts, lm_at, "lift_mot_brk", "lift_motor",
                  P.M2006_BOLT_PCD, 4, 3, 4.0, "lift_mot", link="turret",
                  kind="FLAT")
    put(parts, Pos(mtr[0], 54.0, mtr[2]) * Rot(90, 0, 0)
        * L.pulley(24.0, 10.0, P.M2006_SHAFT_DIA),
        "lift_pul_m", to="lift_motor", how="SHAFT", note="止めねじ")
    put(parts, Pos(ends[0][1] + 4.0, 54.0, ends[0][2] + 26.0) * Rot(90, 0, 0)
        * L.pulley(24.0, 10.0, 5.2),
        "lift_pul_d", to="lift_shaft_loL", how="PRESS", note="止めねじ")
    dpx, dpz = ends[0][1] + 4.0, ends[0][2] + 26.0
    # ⚠ 角枠（Box − Box）で描いていたので、プーリ（φ24）が枠の平らな辺を
    #   979mm³ ×2 突き抜けていた。2 プーリを結ぶ線に沿うループで描く。
    put(parts, L.mat(Pos((mtr[0] + dpx) / 2, 54.0, (mtr[2] + dpz) / 2)
                     * Rot(90, 0, 0)
                     * Rot(0, 0, degrees(atan2(dpz - mtr[2], dpx - mtr[0])))
                     * L.belt_loop(hypot(dpx - mtr[0], dpz - mtr[2]),
                                   12.0, 3.0, 10.0), "RUBBER"),
        "lift_belt_drv", to=("lift_pul_m", "lift_pul_d"), how="MESH")
    LEDGER.add("昇降コンベア駆動 プーリ×2 + ベルト", 0.060, "概算", "砲塔")
    LEDGER.add("昇降コンベア ベルト/プーリ/軸", 0.240, "概算", "砲塔")
    LEDGER.add("M2006 P36 (昇降コンベア)", P.M2006_MASS, "DJI カタログ", "砲塔")
    # ヨー駆動 M3508 + 小プーリ
    # モーターはテーブルの上に**支柱 2 本 + 天板**で持ち上げ、軸を下へ向ける。
    # 小プーリが大プーリと同じ高さに来るように天板の高さを決める。
    # ⚠ 支柱の足元は**旋回アーム**。円板テーブルは廃止したので、
    #   そこに立てると宙に浮く（固定先が存在しない＝検査で即 NG になる）。
    #   モーターはアーム上に立て、小プーリを大プーリの高さに合わせる。
    deck_z = table_z + P.YAW_TABLE_T / 2 + 19.0
    pul_z = P.YAW_RING_H / 2
    for sx in (1, -1):
        put(parts, Pos(P.YAW_ARM_X, 190.0, (table_z + P.YAW_TABLE_T / 2 + deck_z) / 2
                       + sx * 0.0)
            * Pos(sx * 22.0, 0, 0) * Box(16.0, 40.0, 19.0, align=CTR),
            f"yaw_motor_post_{'p' if sx > 0 else 'm'}", to="yaw_arm_f", how="BOLT", note="2-M4")
    # ⚠ **モーター軸の逃げ（φ22）だけは手で開ける。** 境界条件は部品の
    #   bbox から作るので、モーターの**ボスの太さ**が分からない（bbox は
    #   モーター全体で、板の面より上にある）。宣言は BOLT なので座面として
    #   扱われ、穴が開かない。実際これで yaw_motor が板に 455mm³ 食い込んだ。
    #   板の最小部材幅は 51mm あるので φ22 を抜いても製造制約は割らない。
    dk = put(parts, Pos(P.YAW_ARM_X, 190.0, deck_z + P.YAW_MOTOR_DECK_T / 2)
             * (L.topo_plate("yaw_motor_deck", P.YAW_MOTOR_DECK_T)
                - Cylinder(11.0, 8.0, align=CTR)),
             "yaw_motor_deck",
             # ⚠ 「4-M4」は板ぜんたいの本数のつもり。note は**組 1 つ**の
             #   本数として読まれるので、支柱 1 本あたりの数を書く。
             #   支柱は 16(X)×40(Y) で、M4（頭 φ7）は X に 1 列・Y に 2 本。
             to=("yaw_motor_post_p", "yaw_motor_post_m"), how="BOLT", note="2-M4")
    LEDGER.add("ヨーモーター支柱 A5052", 0.033, "体積概算", "砲塔", qty=2)
    LEDGER.add_solid("ヨーモーター取付板 A5052 t4（トポロジー最適化）", dk,
                     "A5052", "砲塔")
    # 取付板は Z 887..891。モーターの取付面はその上面に置く
    # ⚠ 取付面は天板の**上面**。+8 では 4mm 浮いていた。
    ym_at = Pos(P.YAW_ARM_X, 190.0, deck_z + P.YAW_MOTOR_DECK_T) * Rot(180, 0, 0)
    put(parts, ym_at * L.m3508(), "yaw_motor",
        to="yaw_motor_deck", how="BOLT", note="4-M4 PCD35")
    flange_screws(parts, ym_at, "yaw_motor_deck", "yaw_motor",
                  P.M3508_BOLT_PCD, 4, 4, P.YAW_MOTOR_DECK_T, "yaw_mot",
                  link="turret", kind="FLAT")
    # ⚠ 小プーリは大プーリと同じ高さ（局所 10）でなければベルトが掛からない。
    #   モーターの軸端はそこまで届かないので、延長軸で渡す。
    #   「出力軸に止めねじ」と宣言しながら 25mm 空いていた。
    shaft_end = deck_z + P.YAW_MOTOR_DECK_T - P.M3508_SPIGOT_H - P.M3508_SHAFT_LEN
    put(parts, L.mat(Pos(P.YAW_ARM_X, 190.0, (shaft_end + pul_z) / 2)
                     * Cylinder(9.0, shaft_end - pul_z + 15.0, align=CTR)
                     - Pos(P.YAW_ARM_X, 190.0, shaft_end + 10.0)
                     * Cylinder(5.0, 24.0, align=CTR), "STEEL"),
        "yaw_ext_shaft", to="yaw_motor", how="SHAFT", note="延長軸・止めねじ 2")
    LEDGER.add("ヨー駆動 延長軸", 0.060, "体積概算", "砲塔")
    put(parts, Pos(P.YAW_ARM_X, 190.0, pul_z) * L.pulley(P.YAW_PULLEY_SMALL, 15.0, 18.0),
        "yaw_pulley_small", to="yaw_ext_shaft", how="PRESS", note="止めねじ")
    LEDGER.add("M3508 P19 (ヨー)", P.M3508_MASS, "DJI カタログ", "砲塔")
    LEDGER.add("HTD5M 20T プーリ + ベルト", 0.090, "カタログ", "砲塔")
    auto_screws(parts, "turret")
    return Compound(label="turret_yaw", children=parts)


def link_pitch():
    """仰角ユニット（原点 = ニップ中心 = 仰角軸、+X が射出方向）。"""
    parts = []
    off = (P.ROLLER_DIA + P.NIP_GAP) / 2
    pitch_plates = []
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ ウォーム（軸 X、φ24、Y=-290.5）は板厚 4 を貫いて噛む。
        #   歯車の噛み合いに板を挟むことはできないので、逃げ穴を開ける。
        # ⚠ 高さ 140（Z ±70）では、ローラー駆動モーターの受け座（Z 54..114）
        #   も、ウォームホイール（Z 963..1037 = 局所 ±37 の外）も**板の外**に
        #   出ていた。「側板にボルト留め」と宣言していた 10 部品が、
        #   実体としては空中に浮いていた。
        #   受け座の上端（off + ROLLER_BELT_R + 34）まで板を伸ばす。
        # ⚠⚠ **矩形板の掃引半径は対角**。上下対称に ±164.75 まで伸ばすと
        #   掃引半径が √(110² + 164.75²) = 198.1mm になる。仰角軸から
        #   台座横梁の上面までは 162mm しかないので 36mm 足りず、
        #   ヨー ±30° のほぼ全域・仰角 20〜68° で梁を貫いていた
        #   （梁の断面全高 Z 818..838 が板の中に入る＝完全貫通）。
        #   「半径 165」というコメントは Z 半高を掃引半径と混同していた。
        #   下側は受け座（局所 -78.75）より下は要らないので -100 で切る。
        #   掃引半径 148.66 → 横梁まで 13.3mm の余裕。板 2 枚で 210g 軽くなる。
        sp_top = off + P.ROLLER_BELT_R + 34.0
        sp_bot = P.PITCH_SIDE_Z_BOT
        sp_h = sp_top + sp_bot
        sp_off = (sp_top - sp_bot) / 2      # 板の中心を仰角局所で持ち上げる量
        # ⚠ 高さを 140→236 に伸ばしたので、板厚を落とさないと質量が
        #   1.30kg（2 枚）に膨らむ。荷重は雑巾 48g と自重だけなので t3 で足りる。
        # ⚠ 肉抜き径 56 ではリブが 6mm しか残らず、10mm 角の受け柱が穴に
        #   落ちる。柱の位置には材料を戻すので、径は 52（リブ 10）まで戻せる。
        sp_t = P.PITCH_SIDE_T
        # ⚠ 丸穴の肉抜き（`L.lighten` 径 48）をやめて、外形そのものを荷重の
        #   形に切るようにした（`scripts/topo_opt.py`）。384g → 233g（L）/
        #   373g → 146g（R）。DESIGN.md §35。
        # ⚠ **輪郭は `Pos(0, sp_off, 0)` を掛けたまま置くこと。** 輪郭の原点は
        #   板の bbox 中心で、この板の中心は仰角局所の原点から `sp_off` だけ
        #   上にある。掛け忘れると板が丸ごと 33mm ずれる。
        # 以前ここで手当てしていたものは、全部いまは境界条件から出る:
        #   ・受け柱（10 角）の下に材料を戻す → 座面（消させない領域）
        #   ・ローラー軸 φ12 の軸受穴       → 逃げ穴（PRESS / ROTATE 宣言）
        #   ・仰角モーターの逃げ穴（R 側だけ）→ 同上
        sp_plate = Pos(0, sp_off, 0) * L.topo_plate(f"pitch_side_{sd}", sp_t)
        if sy < 0:
            # ⚠ **仰角モーターの逃げ穴だけは手で開ける。** 境界条件は相手の
            #   外接箱から作るので、モーターの箱がねじの座面まで伸びている。
            #   箱をそのまま抜くと座面が 432mm² 欠けて「ねじが留められない」、
            #   座面を守ると今度は板が残ってモーターに当たる（実際どちらも
            #   起きた）。**箱では胴体（φ36）と取付フランジを区別できない**
            #   のが原因なので、ここは形を知っている側で開ける。
            # ⚠ 上端はモーターの取付面（-66）より上で止める。-24 まで開けると
            #   軸受ブロック（Z -66..-24）の座面がまるごと無くなる。
            sp_plate -= Pos(-P.PITCH_WORM_CENTER, -83.0, 0) * Box(
                40.0, 38.0, 10.0, align=CTR)
        sp = put(parts, Pos(0, sy * P.PITCH_SIDE_Y, 0) * Rot(90, 0, 0) * sp_plate,
            f"pitch_side_{sd}", to=f"pitch_pivot_{sd}", how="ROTATE", note="仰角軸（φ20 軸受）")

        pitch_plates.append(sp)
    # ⚠ 左右の板は**同じ形ではない**（R 側だけ仰角モーターの逃げ穴がある）。
    #   片方を qty=2 で数えていたので、穴のぶん 12g を余計に計上していた。
    #   質量は 1 枚ずつ実体から取る。
    for sp_ in pitch_plates:
        LEDGER.add_solid("仰角ユニット側板 A5052 t3（肉抜き）", sp_, "A5052", "射出")
    for sz in (1, -1):
        up = "u" if sz > 0 else "d"
        # 駆動モーター（M3508 ギアボックスレス）。
        # **軸の延長線上には置けない**（|Y|=408 になり側面上桁 340..360 を貫通する）。
        # HTD5M ベルトでオフセットし、上ローラー用は +Z、下ローラー用は -X へ逃がす。
        # ⚠ 下ローラーの駆動を -X に置くと、仰角 20° で振ったとき
        #   昇降コンベアのガイド板に 3,285mm³ 食い込む。+X 側へ逃がす。
        mx, mz = (0.0, off + P.ROLLER_BELT_R) if sz > 0 else (P.ROLLER_BELT_R, -off)
        # ⚠ Rot(90,0,0) だと本体が +Y へ 63mm 伸びて側板（|Y| 300..304）を貫く。
        #   軸を +Y に向け、本体を内側（-Y）へ逃がす。プーリも +Y 側に来る。
        # ⚠ モーター取付面は |Y|=250 で、側板の内面 300 とは 50mm 離れている。
        #   直接「側板にボルト留め」とは書けない。角スペーサで受ける。
        # ⚠ 受け座の中をベルトが通る。角スペーサのままだとベルトを 720mm³
        #   飲み込むので、ベルト幅（15+余裕）の窓を開ける。
        # ⚠ 窓を全幅で抜くと受け座が 2 つに切れる（分断検査で検出）。
        #   ベルトが通る側（-Z 側の壁）だけを抜き、残る 3 面で保つ。
        # 受け座はモーター取付面（|Y|=ROLLER_MOTOR_Y）から側板の内面（300）まで。
        # ベルトの窓はベルトが通る -Z 側の壁だけを抜く（全幅で抜くと 2 つに割れる）
        stand_h = P.PITCH_SIDE_IN_Y - P.ROLLER_MOTOR_Y
        # ⚠ 角スペーサ 1 個で受けようとすると矛盾する。モーターの
        #   フランジ（φ42）を受けるには内抜きを 32 まで絞る必要があるが、
        #   その中を駆動プーリ（φ44.2）とベルトが通るので、絞ると
        #   4,427mm³ 食い込む。逆に広げるとフランジが穴に落ちる。
        #   → **取付板 1 枚 + 四隅の柱 4 本**に分ける。プーリは柱のあいだを
        #     通る（対角の柱まで 35.4mm > プーリ半径 22.1mm）。
        # 柱は取付板の**外面**（Y 254）から側板の内面（300）まで
        post_y0 = P.ROLLER_MOTOR_Y + 4.0
        # ⚠ 受け柱はベルトを**跨いで**立てる。ベルトを正しい経路（スタジアム
        #   形）で描き直したら外周半径が 22.1 になり、±25 では柱の内側の面
        #   （20）が 2.1mm 食い込んだ（315mm³ ×4）。
        #   柱の内側の面がベルトの外周より外に来るよう ±29 にする。
        #   ⚠ ベルトの向きは上下で違う。上は鉛直（mx=0）、下は水平
        #     （mz = sz*off なので Δz=0）。跨ぐべき方向が入れ替わるので、
        #     **ベルトに垂直な側**を 29、沿う側を 25 にする。
        po_x, po_z = (29.0, 25.0) if sz > 0 else (25.0, 29.0)
        for k, (px, pz) in enumerate(((po_x, po_z), (-po_x, po_z),
                                      (-po_x, -po_z), (po_x, -po_z))):
            put(parts, L.mat(Pos(mx + px, (post_y0 + P.PITCH_SIDE_IN_Y) / 2, mz + pz)
                             * Box(10.0, P.PITCH_SIDE_IN_Y - post_y0, 10.0,
                                   align=CTR), "A5052"),
                # ⚠ 注記は「2-M5」だった。**10×10 の柱に M5 は 1 本しか
                #   入らない**（頭 φ8.5）。宣言だけ 2 本にしておくと、
                #   員数表はいつまでも「1 本足りない」と言い続ける。
                #   4 本の柱で四角を作っているので 1 本ずつで回り止めになる。
                f"roller_mot_post_{up}{k}", to="pitch_side_L", how="BOLT", note="1-M5")
            LEDGER.add("ローラー駆動 受け柱 A5052", 0.013, "体積概算", "射出")
        put(parts, L.mat(Pos(mx, P.ROLLER_MOTOR_Y + 2.0, mz) * Rot(90, 0, 0)
                         * (L.plate(2 * po_x + 12.0, 2 * po_z + 12.0, 4.0)   # 柱を全幅で受ける
                            - Cylinder(P.M3508_SPIGOT_DIA / 2 + 1.0, 8.0, align=CTR)),
                         "A5052"),
            f"roller_mot_stand_{up}",
            to=tuple(f"roller_mot_post_{up}{k}" for k in range(4)),
            # ⚠ 「4-M5」は**板ぜんたいの本数**のつもりで書かれていたが、
            #   note は**その組 1 つに要る本数**として読まれる（4 柱 ×4 本
            #   ＝16 本の宣言になっていた）。柱は 10×10 で 1 本しか入らない。
            how="BOLT", note="1-M5")
        LEDGER.add("ローラー駆動 取付板 A5052 t4", 0.039, "体積概算", "射出")
        # ⚠ **柱の芯へ 1 本ずつ手で置く。** 10×10 の端面は、頭の座面ぶん
        #   （半径 + 1mm）収縮させると格子が全滅するので、`screw_place` は
        #   収縮前の格子に落ちて縁を選び、隣のねじと当たって 8 組とも
        #   0 本になっていた。柱の位置は分かっているので探す必要がない。
        for k, (px, pz) in enumerate(((po_x, po_z), (-po_x, po_z),
                                      (-po_x, -po_z), (po_x, -po_z))):
            face_screws(parts,
                        [Pos(mx + px, P.ROLLER_MOTOR_Y, mz + pz) * Rot(90, 0, 0)],
                        f"roller_mot_stand_{up}", f"roller_mot_post_{up}{k}",
                        5, 4.0, f"roller_stand_{up}{k}_", link="pitch")
        rm_at = Pos(mx, P.ROLLER_MOTOR_Y, mz) * Rot(-90, 0, 0)
        put(parts, rm_at * L.m3508(with_gearbox=False),
            f"roller_motor_{up}", to=f"roller_mot_stand_{up}", how="BOLT", note="4-M4 PCD35")
        flange_screws(parts, rm_at, f"roller_mot_stand_{up}",
                      f"roller_motor_{up}", P.M3508_BOLT_PCD, 4, 4, 4.0,
                      f"roller_mot_{up}", link="pitch")
        LEDGER.add("M3508 ギアボックスレス", 0.290, "DJI カタログ(GB分を除く)", "射出")
        # 駆動ベルトとプーリ（1:1、HTD5M 幅15）
        pul_y = P.ROLLER_PUL_Y
        # ⚠ プーリの内径は**軸径**。24 のままだと M3508 の軸（φ14）との
        #   あいだに 5mm 空き、「軸に止めねじ」と宣言しながら触れていない。
        put(parts, Pos(mx, pul_y, mz) * Rot(90, 0, 0)
            * L.pulley(38.2, 15.0, P.M3508_SHAFT_DIA),
            f"roller_pul_m_{up}", to=f"roller_motor_{up}", how="SHAFT", note="止めねじ")
        # ⚠ 斜めに掛かるベルトを**軸に平行な矩形の枠**で描いていた。
        #   ベルトの経路ですらないうえ、プーリ（φ38.2）が枠の平らな端を
        #   突き抜けて 2,872mm³ ×4 重なっていた。
        #   2 プーリの中心を結ぶ線に沿ったスタジアム形で描く。
        b_span = hypot(mx - 0.0, mz - sz * off)
        b_ang = degrees(atan2(mz - sz * off, mx - 0.0))
        belt = (Pos(mx / 2, pul_y, (mz + sz * off) / 2)
                * Rot(90, 0, 0) * Rot(0, 0, b_ang)
                * L.belt_loop(b_span, 38.2 / 2, 3.0, 15.0))
        # ⚠ ベルトが触れるのは 2 つのプーリだけ。受け座は掛かっていない。
        put(parts, L.mat(belt, "RUBBER"), f"roller_belt_{up}",
            to=(f"roller_pul_m_{up}", f"roller_pul_s_{up}"),
            how="MESH", note="受け座の窓を通ってプーリに巻き付く")
        LEDGER.add("HTD5M 24T プーリ ×2 + ベルト（ローラー駆動）", 0.150, "カタログ", "射出")
    # ニップ隙間調整: テーパくさび 1:20（上ローラー軸受ブロックを押し下げる）
    # 偏心カムでは分解能 209µm で目標 15µm に届かない（scripts/nip_calibration.py）
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ 上ローラーの駆動ベルトが |Y| 281.5..296.5 を通る。くさびは
        #   その内側（<=281）に収める（幅 24 → 中心 268）。
        # ⚠ 射出ローラーは |Y| 0±20 / 130±20 / 265±20。駆動ベルトは 281.5..296.5。
        #   くさび（幅 24）が入る帯は **150..245 の空き**しかない。
        # ⚠ くさびは側板（内面 |Y|=300）に沿って**滑る**部品なので、
        #   側板に接していなければならない。同時に端の射出ローラー
        #   （|Y| 245..285）と駆動ベルト（281.5..296.5）を避ける必要がある。
        #   幅 24 のくさびが入る帯は **|Y| 276..300 のみ**（中心 288）。
        #   ベルトとは 5.5mm 重なるので、くさびの厚みを 18 に減らして
        #   |Y| 282..300（中心 291）に収める。
        # ⚠ ローラー軸プーリ（|Y| 259.5..278.5）の外に収める。幅 24 で
        #   中心 288.5 だと 276.5 まで来てプーリに 42mm³ かかる。
        put(parts, L.mat(Pos(-30.0, sy * (P.PITCH_SIDE_IN_Y - 10.0), off + 26.0)
                         * Rot(0, math.degrees(math.atan2(1.0, P.NIP_ADJ_TAPER)), 0)
                         * Box(70.0, 20.0, 9.0, align=CTR), "A5052"),
            f"nip_wedge_{sd}", to=f"pitch_side_{sd}", how="SLIDE", note="調整ねじで進退")
        # ⚠ 調整ねじは X 方向に進退する。側板は XZ 面の板なので、ねじが
        #   側板に直接ねじ込まれることはない（4 件とも実体が離れていた）。
        #   側板の内面から受けブロックを出し、そこにねじを切る。
        put(parts, L.mat(Pos(26.0, sy * (P.PITCH_SIDE_IN_Y - 8.0), off + 28.0)
                         * (Box(20.0, 16.0, 24.0, align=CTR)
                            - Rot(0, 90, 0) * Cylinder(2.2, 24.0, align=CTR)), "A5052"),
            f"nip_snut_{sd}", to=f"pitch_side_{sd}", how="BOLT", note="2-M3")
        LEDGER.add("ニップ調整ねじ 受けブロック A5052", 0.012, "体積概算", "射出")
        # ⚠ Rot(0,-90,0) だと軸が +X（外側）へ伸びて、押す相手と反対を向く。
        put(parts, Pos(36.0, sy * (P.PITCH_SIDE_IN_Y - 12.0), off + 28.0)
            * Rot(0, 90, 0) * L.screw(4, 56),
            f"nip_screw_{sd}", to=(f"nip_wedge_{sd}", f"nip_snut_{sd}"), how="SCREW_IN")
    LEDGER.add("ニップ調整くさび A5052（1:20）", 0.042, "体積計算", "射出", qty=2)
    # ウォーム減速（仰角）
    # ⚠ 仰角ウォームの駆動系は**旋回体の一部**。旋回半径が側面上桁の内側（|Y|=330）を
    #   超えると、ヨーを振ったときに上桁と当たる。
    #   以前は |Y|=320 に置いていたが、M2006（全長63）が -338 まで伸びて
    #   旋回半径 349mm になり、上桁を 19mm 超えていた。
    #   ヨー0° では 2mm 空くので、静止姿勢だけ見ていると気付けない。
    # ⚠ |Y|=290 は端のローラー（245..285）の縁に当たる。292 まで外へ
    # 鍔込み 19mm。側板の内面（|Y|=300）に接して内側へ収まる位置
    # 側板の内面(300)と端ローラー(285)のあいだは 15mm。鍔込み 16mm に収める
    # 鍔込み 16mm。側板の内面 300 に接して内側へ → 中心 -292
    # ⚠ ウォームホイールは仰角軸（= ニップ軸）と一体で回る。
    #   φ80 なので、そこから 45.75mm 離れたローラー軸（φ12）と重なる。
    #   ホイール径を φ68 に落として軸をかわす（減速比は 40:1 のまま）。
    # ローラー軸の軸受。**側板の穴に圧入する部品なので側板と同じリンク**に置く。
    #   ローラー側に置くと「回るリンクの部品を回らない板に圧入」になる。
    for upb in ("u", "d"):
        for syb in (1, -1):
            sdb = "L" if syb > 0 else "R"
            szb = 1 if upb == "u" else -1
            put(parts, L.mat(Pos(0, syb * P.PITCH_SIDE_Y, szb * off) * Rot(90, 0, 0)
                             * (Cylinder(P.ROLLER_SHAFT_DIA / 2 + 2.0, 8.0, align=CTR)
                                + Pos(0, 0, -syb * 5.0)
                                * Cylinder(P.ROLLER_SHAFT_DIA / 2 + 5.0, 2.0, align=CTR)
                                - Cylinder(P.ROLLER_SHAFT_DIA / 2, 12.0, align=CTR)),
                             "STEEL"),
                f"roller_brg_{upb}{sdb}", to=f"pitch_side_{sdb}", how="PRESS",
                note="フランジ付き軸受を板の穴へ圧入")
            LEDGER.add("フランジベアリング φ12", 0.012, "カタログ", "射出")
    # --- ウォームの軸線と中心距離 -----------------------------------------
    # ⚠ **ウォーム軸はホイールの歯幅の中心平面に乗っていなければ噛まない。**
    #   以前はホイールの中心 |Y|=292.5、ウォーム軸 |Y|=288.5 で 4mm ずれていた。
    #   「MESH」と宣言してあったので検査は通っていたが、実物では歯幅の
    #   片側 2mm しか当たらない（面圧が倍）。両方をこの `wh_y` から取る。
    # ⚠ |Y| は「側板の内面から 15mm」。ウォームの歯先半径が 13.5 なので、
    #   これで側板まで 1.5mm 空く。12mm（＝以前の値）だと歯先が側板に
    #   触れてしまい、回る部品が板を擦る。
    wh_y = -(P.PITCH_SIDE_IN_Y - 15.0)
    # ⚠ 中心距離は**基準円半径の和**。以前は「円筒半径の和」で 46 にして
    #   いたので、基準円どうしが 4mm 離れていた（歯を描いた瞬間に噛まない）。
    worm_x = -P.PITCH_WORM_CENTER
    worm_y = wh_y
    # ホイール（歯部 + ボス）。歯部は |Y| 279.5..291.5、ボスがそこから
    # 側板の内面（300.5）までを埋めて 6-M4 で共締めされる。
    # ⚠ ボスが無いと歯部は側板から 9mm 浮く。以前は「鍔付きプーリ」の鍔が
    #   たまたま板に届いていただけで、締結の座面にはなっていなかった。
    wh_hub_l = P.PITCH_SIDE_IN_Y - (-wh_y + P.PITCH_WHEEL_FACE / 2)   # 9.0
    wheel = (L.worm_wheel(P.PITCH_WORM_MODULE, P.PITCH_WORM_STARTS,
                          int(P.PITCH_WORM_RATIO * P.PITCH_WORM_STARTS),
                          P.PITCH_WHEEL_FACE, 20.0, P.PITCH_WORM_CENTER,
                          P.PITCH_WORM_BL, P.PITCH_WORM_D1, P.PITCH_WORM_ALPHA)
             + Pos(0, -(P.PITCH_WHEEL_FACE + wh_hub_l) / 2, 0) * Rot(90, 0, 0)
             * Cylinder(18.0, wh_hub_l, align=CTR)
             # 肉抜き。ウェブを 5mm 残し、歯元（半径 28.1）には掛けない。
             # ⚠ 掘るのはボスと反対の面だけ。両面から掘るとウェブが
             #   抜けて、歯部とボスが**別々のソリッドに分断**される。
             - Pos(0, P.PITCH_WHEEL_FACE / 2 - 3.5, 0) * Rot(90, 0, 0)
             * (Cylinder(25.0, 7.0, align=CTR) - Cylinder(13.0, 9.0, align=CTR))
             - Rot(90, 0, 0) * Cylinder(10.0, 40.0, align=CTR))
    # ⚠ ホイールは**青銅ではなくアルミ（A5052）**にした。ウォームギヤの
    #   相手歯車は摩耗を嫌って青銅（CAC702）を使うのが定石で、実際
    #   φ63×15 の青銅ホイールなら 220g になる。ここをアルミにすると 70g で、
    #   150g の差は 35.0kg の規定に対して無視できない。
    #   使ってよいと判断した根拠:
    #     ・歯元曲げ応力 = Ft/(b·m·Y)。定格時 Ft = 2×16.8/0.060 = 560N →
    #       560/(12×1.5×0.42) = 74MPa。ストール時（933N）でも 124MPa で
    #       A5052 の耐力 195MPa を超えない（＝塑性変形しない）。
    #     ・摩耗は**稼働時間**で決まる。仰角を動かすのは 1 射ごとに数秒、
    #       1 試合で 30 秒に満たない。青銅が要るのは連続運転の減速機。
    #     ・二硫化モリブデングリスを塗る前提（かじり止め）。
    #   摩耗が出たら、同じ歯切りで CAC702 に差し替えられる（+150g）。
    put(parts, L.mat(Pos(0, wh_y, 0) * wheel, "A5052"),
        "pitch_worm_wheel", to="pitch_side_R", how="BOLT",
        note="6-M4 PCD28。ボスで側板へ共締め（＝仰角軸と一体）")
    LEDGER.add_solid("ウォームホイール m1.5 z40 A5052（肉抜き）",
                     wheel, "A5052", "射出")
    # ⚠⚠ **この減速機は「実機として成立している」が、モデルの上では
    #   ウォームもホイールも同じ仰角リンクに置いてある。** 実機では
    #   ホイールは仰角軸（＝旋回体に固定された軸）とキーで一体、ウォームと
    #   モーターは仰角ユニット側に載る＝両者は別リンクで、モーターを回すと
    #   ウォームがホイールの周りを歩いて仰角が変わる。
    #   CAD で別リンクにしなかったのは、連続掃引（sweep_fine.py）が
    #   「リンクは剛体」を前提にしていて、仰角 0.5° 刻みでウォームを
    #   40 倍（20°/step）自転させる経路を持たないため。別リンクにすると
    #   途中の姿勢で歯の頂どうしが当たった判定になる。
    #   ヨー駆動（yaw_pulley_big と駆動モーターが両方 turret_yaw にある）も
    #   同じ簡略化をしている。**リンク割りを直すなら sweep_fine 側に
    #   「ウォームの自転」を足すのが先**。
    # --- ウォーム軸 --------------------------------------------------------
    # 軸 φ12・ねじ部 z ±14。両端に 6001（φ12×φ28×8）を背面組合せで置き、
    # 軸方向はこう受ける（**ウォームは軸方向に押される**）:
    #   +Z 推力 : 軸の鍔(z 24..28) → 6001p 内輪 → 玉 → 外輪(z 36) → ブロックの段
    #   −Z 推力 : 軸の鍔(z -26..-22) → 6001m 内輪 → 玉 → 外輪(z -34) → ブロックの段
    # 反対向きは相手側の軸受が受けるので、2 個で両方向を拘束できる（両端固定）。
    # ⚠ 以前はウォーム（φ24×60、z ±30）がブロックの端面（z=±30）に
    #   突き当たっているだけで、**軸受も鍔も無かった**。「ROTATE」と
    #   宣言してあったので接触検査は通るが、実機では軸方向にも半径方向にも
    #   何も受けていない。
    # 定格時の軸方向力 = ホイール接線力 = 2×16.8N·m/0.060m = 560N。
    # 6001 の許容アキシアル荷重（0.5·C0r = 0.98kN）に対して 1.75 倍の余裕。
    # 軸は φ12 の 1 本もの（z -52..+42）。ねじ部（歯底 φ20.25）が軸を包むので、
    # 単純に足せば 1 ソリッドに融合する。
    shaft = Pos(0, 0, -5.0) * Cylinder(6.0, 94.0, align=CTR)
    for z_sh in (26.0, -24.0):          # 鍔: z 24..28 と z -26..-22
        shaft += Pos(0, 0, z_sh) * Cylinder(10.0, 4.0, align=CTR)
    worm_body = shaft + L.worm(P.PITCH_WORM_MODULE, P.PITCH_WORM_STARTS,
                               P.PITCH_WORM_D1, P.PITCH_WORM_THREAD_L,
                               P.PITCH_WORM_BL, P.PITCH_WORM_ALPHA)
    put(parts, L.mat(Pos(worm_x, worm_y, 0) * worm_body, "STEEL"),
        "pitch_worm",
        to=("pitch_worm_wheel", "pitch_wbrg_p", "pitch_wbrg_m"),
        how=("MESH", "PRESS", "PRESS"),
        note="m1.5 1条 リード4.712・進み角3.58°（セルフロック）")
    LEDGER.add_solid("ウォーム m1.5 1条 S45C 焼入", worm_body, "STEEL", "射出")
    # ウォームの両端を受ける軸受ブロック。側板の内面に留まる。
    #   p 側 : 軸受だけ（z 24..44）
    #   m 側 : 軸受 + カップリング室 + モーター取付フランジ（z -66..-24）
    # ⚠ ブロックの内側の角は**ホイールの歯先円（半径 31.5）の外**に置く。
    #   z を ±30 にすると角が半径 29.7 に来てホイールを削る。
    # ⚠ 6001 の外輪座が φ28 なので、**幅 36 のブロックではボアの縁から外面まで
    #   4mm しか残らず、M4 のタップが立たない**（呼び径 4 + 肉厚 2×2 で 8mm）。
    #   長らく「幅 36 のまま 4-M4 と宣言し、質量に余裕ができたら直す」で
    #   止めてあったが、**留まっていない継手を図に残すほうが害が大きい**
    #   （員数表では 4 本ぶん買うことになっているのに、実機では組めない）。
    #   ⚠ **2026-08-07 に幅 52 を試して戻した。** タップは立つようになるが、
    #     広げた角が side plate に 240mm³ 食い込み、ウォームホイールと
    #     ニップ入口ガイドにも触れる（`assembly_check` で 4 件）。X 方向は
    #     もう空いていない。残る手はコメント上段の**取付面側フランジ**
    #     （厚 8・幅 56 を側板側だけに出す）だが、それも |X| 26 を超えるので
    #     同じ相手に当たる。**side plate 側を先に作り直す必要がある**。
    #     いまは幅 36 のまま「4-M4」と宣言している（実体は 2〜3 本）。
    BLK_W = 36.0
    for szw in (1, -1):
        tag = "p" if szw > 0 else "m"
        if szw > 0:
            blk = (Pos(0, 0, 34.0) * Box(BLK_W, 30.0, 20.0, align=CTR)
                   # 外輪の座（φ28）と、+Z 推力を受け止める段（φ24）
                   - Pos(0, 0, 30.0) * Cylinder(14.0, 12.0, align=CTR)
                   - Pos(0, 0, 40.0) * Cylinder(12.0, 8.0, align=CTR))
        else:
            # ⚠ カップリング室は**角材ではなく筒**にする。Box(36,30,22) の
            #   ままだと、ここだけで 41g（ブロック全体の 6 割）になる。
            #   側板に当たるのは軸受側の角材なので、座面は失わない。
            # ⚠ **筒（φ30）の端面にモーターは留められなかった。**
            #   M2006 の 4-M3 は PCD26（半径 13）で、筒の肉は半径 11..15 の
            #   4mm しかない。しかもねじは +Z 側から差すので、**筒そのものが
            #   工具の通り道**になる。「BOLT・4-M3 PCD26」と宣言して実体が
            #   1 本も無かったのは、置ける場所がどこにも無かったから。
            #   → 筒を**コの字**（取付面板 t4 + 側壁 2 枚 t5）に作り替える。
            #     ねじは面板を通り、頭は側壁のあいだ（|X| 13 の内側）に立つ。
            #     45° 位相のボルト円なので、頭（半径 2.75）も六角レンチ
            #     （半径 3.5）も |X| 9.19 + 3.5 = 12.7 < 13 で収まる。
            blk = (Pos(0, 0, -34.0) * Box(BLK_W, 30.0, 20.0, align=CTR)
                   # ⚠ 取付面板の Y は**ブロックの幅（30）を超えない**こと。
                   #   40 角にすると側板（ブロックの +Y 面に当たる）へ 5mm
                   #   はみ出して 240mm³ 食い込む。X は 40 でよい（側板は
                   #   Y 方向の相手なので）。
                   + Pos(0, 0, -64.0) * Box(40.0, 30.0, 4.0, align=CTR)
                   + Pos(15.5, 0, -53.0) * Box(5.0, 30.0, 18.0, align=CTR)
                   + Pos(-15.5, 0, -53.0) * Box(5.0, 30.0, 18.0, align=CTR)
                   - Pos(0, 0, -29.0) * Cylinder(14.0, 10.0, align=CTR)
                   # −Z 推力を受け止める段（φ26）＋カップリング／軸の逃げ（φ22）
                   - Pos(0, 0, -39.0) * Cylinder(13.0, 10.0, align=CTR)
                   - Pos(0, 0, -55.0) * Cylinder(11.0, 22.0, align=CTR))
        put(parts, L.mat(Pos(worm_x, worm_y, 0) * blk, "A5052"),
            f"pitch_wbrk_{tag}", to="pitch_side_R", how="BOLT",
            note="4-M4。側板の内面へ")
        LEDGER.add_solid("ウォーム軸受ブロック A5052", blk, "A5052", "射出")
        # 6001ZZ（φ12×φ28×8）。外輪はブロックの座へ、内輪は軸へ圧入。
        #   p 側 z 28..36（外輪の +Z 面がブロックの段 z=36 に当たる）
        #   m 側 z -34..-26（外輪の −Z 面がブロックの段 z=-34 に当たる）
        brg_z, col_z = (32.0, 38.5) if szw > 0 else (-30.0, -36.5)
        put(parts, Pos(worm_x, worm_y, brg_z) * L.bearing(28.0, 12.0, 8.0),
            f"pitch_wbrg_{tag}", to=f"pitch_wbrk_{tag}",
            how="PRESS", note="6001ZZ 外輪をブロックの座へ")
        LEDGER.add("深溝玉軸受 6001ZZ", 0.020, "カタログ", "射出")
        # スラストカラー（鍔）。軸側の鍔とで内輪を挟み、軸方向を締める。
        col = Cylinder(10.0, 5.0, align=CTR) - Cylinder(6.0, 7.0, align=CTR)
        put(parts, L.mat(Pos(worm_x, worm_y, col_z) * col, "STEEL"),
            f"pitch_wcol_{tag}", to=("pitch_worm", f"pitch_wbrg_{tag}"),
            how=("PRESS", "PRESS"), note="止めねじ 2。内輪を鍔とで挟む")
        LEDGER.add_solid("ウォーム軸 スラストカラー", col, "STEEL", "射出")
    # カップリング（M2006 の φ8 軸 ↔ ウォームの φ12 軸）。
    # ⚠ 以前はモーターの軸端（z -50..-38）が軸受ブロックの穴に刺さっている
    #   だけで、ウォームとは 8mm 離れていた。「BOLT」でハウジングを留めた
    #   宣言はあっても、**トルクを渡す実体が無かった**。
    # ⚠ M2006 の軸は φ8×12 しか無い。掴み代 10mm を残すと、カップリングの
    #   全長 20mm・ウォーム軸との隙間 2mm がぎりぎり。ここを詰めると
    #   いもねじが 1 本しか掛からない。
    # ⚠ 外径は **φ18**（φ20 ではない）。M2006 の取付ボルト円は PCD26 ＝
    #   半径 13 で、皿ねじの頭（φ6 → 半径 3）の内側の縁がちょうど半径 10 に
    #   来る。φ20 のままだと回るカップリングにねじの頭が触れる（実測 0.00mm）。
    #   φ18 なら 1mm 空く。φ12 の軸に対する肉は 3mm 残るので掴みは足りる。
    put(parts, L.mat(Pos(worm_x, worm_y, -54.0)
                     * (Cylinder(9.0, 20.0, align=CTR)
                        - Pos(0, 0, 5.0) * Cylinder(6.0, 12.0, align=CTR)
                        - Pos(0, 0, -5.0) * Cylinder(4.0, 12.0, align=CTR)),
                     "A5052"),
        "pitch_wcplg", to="pitch_worm", how="PRESS", note="止めねじ 2")
    # MST-20C（φ20×L25・A2017）はカタログ 24g。ここは L20 なので按分する。
    LEDGER.add("カップリング MST-20C 相当（φ8-φ12・L20）", 0.020,
               "カタログ 24g×20/25", "射出")
    # モーターはウォームの軸線上（-Z 側）。取付面は軸受ブロックの -Z 面。
    ptm_at = Pos(worm_x, worm_y, -66.0)
    put(parts, ptm_at * L.m2006(),
        "pitch_motor", to=("pitch_wbrk_m", "pitch_wcplg"),
        how=("BOLT", "SHAFT"), note="4-M3 PCD26")
    flange_screws(parts, ptm_at, "pitch_wbrk_m", "pitch_motor",
                  P.M2006_BOLT_PCD, 4, 3, 4.0, "pitch_mot", link="pitch",
                  kind="FLAT")
    LEDGER.add("M2006 P36 (仰角)", P.M2006_MASS, "DJI カタログ", "射出")
    # 入口ガイド（漏斗受け）
    # ニップ入口ガイド: 3Dプリンタのベッドに収まるよう Y方向に3分割（1枚 185mm）
    GUIDE_T = 2.0            # 板厚
    GUIDE_FLG_T = 4.0        # 耳の厚み。2 枚合わせて M4 の掴み 8mm
    GUIDE_FLG_H = 16.0       # 耳の高さ（板の面から）。M4 の座面 φ7 が載る
    for sz in (1, -1):
        # 3 分割の合計幅は**側板の内面（|Y|=300）まで届くこと**。
        # 185×3 では端が 282.5 で止まり、17.5mm 浮いていた。
        # ⚠ 3 分割の合計は側板の内面（|Y|=300）まで届くこと。
        #   受け座の逃げを抜いたぶん端が 188 で終わっていた。
        #   分割位置を広げ、逃げは受け座の X 範囲だけに限る。
        gext = P.PITCH_SIDE_IN_Y - 300.0     # 内面までの足りぶん
        for k, gy in enumerate((-200.0 - gext / 2, 0.0, 200.0 + gext / 2)):
            gwid = 200.0 if k == 1 else 200.0 + gext
            # ⚠ ガイドの長さは**掃引帯の幅**を決める。仰角ユニットに付いて
            #   いて仰角 20〜70° で振れるので、外端が仰角軸から遠いほど
            #   広い帯を掃き、その帯にある固定物（昇降コンベアの駆動系・
            #   旋回リング）を全部なぎ倒す。
            #   80 だと軸から 69..141mm、40 なら 69..104mm。
            #   入口で雑巾を導くのに 40mm あれば足りる。
            # 板ローカル系（x = 傾斜に沿う幅 / y = Y / z = 板の法線）。
            # 耳もこの系で作る（板と一緒に傾く）。
            m = Pos(-75.0, gy, sz * 42.0) * Rot(0, sz * 12.0, 0)
            g = m * Box(40.0, gwid, GUIDE_T, align=CTR)
            # 継ぎ手／側板取付の耳（フランジ）。
            # ⚠ **板厚 2mm の端面には M4 をねじ込めない。** 以前は 3 枚を
            #   端面どうしで突き合わせるだけで、`screw_place` が「bbox の
            #   重なりが最も薄い軸」＝ Y を接触の法線に選び、t2 の断面へ
            #   M4×20 を 8 本打っていた（ビューアではボルトが板の面に
            #   寝て刺さって見える）。座面が取れないことは `_erode()` の
            #   収縮で弾けるはずだが、あれは**全滅したら収縮前を返す**ので
            #   幅 2mm の帯がそのまま候補として生き残る。
            # ⚠ 板は Y 軸まわりに 12° 傾いているが、**耳の面の法線は Y の
            #   まま**（曲げ線が板面内の傾斜方向に沿うため）。ここが軸平行
            #   でないと `screw_place` は接触面として扱えない。
            # 耳は**通路と反対側**（上ガイドは上、下ガイドは下）へ立てる。
            #   雑巾の通り道に段差を出さないため。
            for e in (-1, 1):
                # 外端側（側板に留まる端）はウォーム軸受ブロックの逃げを
                # 避けて幅を詰める。継ぎ手側は板の全幅を使える。
                outer = (k == 0 and e < 0) or (k == 2 and e > 0)
                fw, fx = (22.0, -8.0) if outer else (36.0, -1.0)
                g += (m * Pos(fx, e * (gwid - GUIDE_FLG_T) / 2,
                              sz * (GUIDE_T + GUIDE_FLG_H) / 2)
                      * Box(fw, GUIDE_FLG_T, GUIDE_FLG_H, align=CTR))
            # ⚠ **ローラー駆動の受け座の逃げは消した。** 受け座の実体は
            #   `roller_mot_stand_u` が X -35..35 / Z 100..162、`_d` が
            #   X 54..116 / Z -81..-11 で、ガイド板（X -95..-55 / |Z| 37..47）
            #   とは 1 度も交わらない。逃げ（X -163..-73）はガイド板の外端を
            #   幅 22mm ぶん削るだけで、**外端の耳を立てる場所を潰していた**
            #   （-Y 側はウォームの逃げと挟まれて幅 5mm しか残らなかった）。
            # ウォーム軸受ブロック（X -60..-24、Y -300.5..-270.5）の逃げ。
            # ⚠ 逃げの中心は**ブロックの中心（= ウォーム軸）に合わせる**。
            #   以前は中心 X=-46 / Y=-290 で、ブロックの縁とのすきまが
            #   0.5mm（＝接触判定のしきい値ちょうど）しか無かった。
            #   軸に合わせると 4〜5mm 空き、ガイド板の削り量もむしろ減る。
            g -= Pos(-P.PITCH_WORM_CENTER, -285.5, sz * 42.0) * Box(
                52.0, 40.0, 40.0, align=CTR)
            put(parts, L.mat(g, "PETG"),
                f"nip_guide_{'u' if sz > 0 else 'd'}{k}",
                # ⚠ 3 分割したのに全部が「両方の側板に留まる」と宣言していた。
                #   中央（|Y|<=100）はどちらの側板にも 200mm 届かない。
                #   端の 2 枚は側板へ、中央は端の 2 枚へ耳どうしで留める。
                to=(("pitch_side_R",) if k == 0 else
                    ("pitch_side_L",) if k == 2 else
                    (f"nip_guide_{'u' if sz > 0 else 'd'}0",
                     f"nip_guide_{'u' if sz > 0 else 'd'}2")),
                how="BOLT", note="2-M4")
    LEDGER.add("ニップ入口ガイド PETG t2 + 継ぎ耳（3Dプリント・200mm×3分割）",
               0.025, "体積概算", "射出", qty=6)
    auto_screws(parts, "pitch")
    return Compound(label="shooter_pitch", children=parts)


def link_roller(up="u"):
    """射出ローラー1軸ぶん（原点 = 回転軸、軸は Y）。

    ⚠ **軸と従動プーリもここに置く**。軸はローラーと一体で回るので、
      仰角リンク側に描くと「回らない板に回るローラーを圧入している」
      ことになる。仰角側にあるのは軸受（ROTATE）だけ。
    """
    parts = []
    # 側板の内面は |Y|=300。軸はそこまでで止め、板の穴の軸受で受ける
    # ⚠ 側板に軸受穴（φ16）を開けたので、軸は板に触れなくなった。
    #   「フランジベアリング 2 個」と注記していた実体をここで描く。
    #   穴に入るブッシュが軸と板の両方に接して、はじめて軸が受かる。
    put(parts, Rot(90, 0, 0) * (
            Cylinder(P.ROLLER_SHAFT_DIA / 2, 2 * 301.0, align=CTR)
            - Cylinder(P.ROLLER_SHAFT_DIA / 2 - 2.0, 2 * 303.0, align=CTR)),
        f"roller_shaft_{up}", to=(f"roller_brg_{up}L", f"roller_brg_{up}R"),
        how="ROTATE", note="フランジベアリング 2 個")
    LEDGER.add("ローラー軸 A2017 φ12 中空×620", 0.105, "体積概算", "射出")
    put(parts, Pos(0, P.ROLLER_PUL_Y, 0) * Rot(90, 0, 0)
        * L.pulley(38.2, 15.0, P.ROLLER_SHAFT_DIA),
        f"roller_pul_s_{up}", to=f"roller_shaft_{up}", how="PRESS", note="止めねじ")
    for k, y in enumerate(P.ROLLER_Y):
        put(parts, Pos(0, y, 0) * Rot(90, 0, 0) * L.shooter_roller(), f"roller_{up}{k}",
            to=f"roller_shaft_{up}", how="PRESS", note="ハブを軸に圧入 + 止めねじ")
    LEDGER.add("射出ローラー φ90×40 (3Dプリント+ウレタン)", 0.078, "体積計算", "射出",
               qty=len(P.ROLLER_Y))
    return Compound(label="roller", children=parts)


def link_singulator():
    """シンギュレータ（原点 = 分離ローラー軸、軸は Y）— **回る側だけ**。

    捌きは「送る側（このリンク：高摩擦の分離ローラー）」と「戻す側（車体側の
    分離ゲート＝リタードパッド）」の**対**で成立する。片方だけでは原理的に
    捌けない。戻す側は回らないので `_hopper()` にある。

    ⚠ このリンクに置く部品は**すべて軸対称**にすること。`spin_check.py` は
      リンク全体の bbox の中心を回転軸と見なすので、軸から外れた実体を
      1 つ置くと、掃引の中心がずれて全部の判定が狂う。
    """
    parts = []
    # --- 軸 ---------------------------------------------------------------
    # ⚠ Y の端は左右で**違う**。-Y 側はブッシュの外面で終わり、+Y 側は
    #   継手の中まで SING_CPLG_GRIP だけ入る。以前は ±310 の対称で切って
    #   いたので、継手とは端面が触れているだけ＝**掴み代 0**だった。
    #   「カップリング」と注記した実体があっても、軸が入っていなければ
    #   トルクは伝わらない。
    y_lo = F.face("sing_bush_R", "y", -1)
    y_c0 = F.face("hop_side_L", "y", 1) + 2.0        # 継手の -Y 端（側壁から 2mm 逃がす）
    y_hi = y_c0 + P.SING_CPLG_GRIP
    put(parts, L.mat(Pos(0, (y_lo + y_hi) / 2, 0) * Rot(90, 0, 0)
                     * (Cylinder(P.SING_SHAFT_DIA / 2, y_hi - y_lo, align=CTR)
                        - Cylinder(P.SING_SHAFT_ID / 2, y_hi - y_lo + 2, align=CTR)),
                     "A5052"),
        "sing_shaft", to=("sing_bush_L", "sing_bush_R"), how="ROTATE",
        note="無給油フランジブッシュ 2 個で受ける")
    LEDGER.add("分離ローラー軸 A2017 φ8×t1.5 中空",
               math.pi * ((P.SING_SHAFT_DIA / 2) ** 2 - (P.SING_SHAFT_ID / 2) ** 2)
               * (y_hi - y_lo) * 2.79e-3 / 1000, "A2017 体積計算", "装填")
    # --- 分離ローラー（摩擦材とハブを分ける）------------------------------
    # ⚠ タイヤ（ウレタン）は布と擦れて必ず減る**消耗品**。ハブと一体の
    #   1 部品で描くと、減ったときにローラーごと作り直す設計になる。
    #   スリーブとして抜けるよう別部品にする。
    # ⚠ ハブは薄肉の中空軸に**止めねじで締めてはいけない**（肉厚 1.5 の管が
    #   潰れる）。φ2 スプリングピンを軸ごと貫通させて回り止めにする。
    for k, y in enumerate(P.PICK_ROLLER_Y):
        hub = put(parts, Pos(0, y, 0) * Rot(90, 0, 0) * L.pick_hub(),
                  f"sing_hub{k}", to="sing_shaft", how="PRESS",
                  note="φ2 スプリングピン貫通（薄肉軸なので止めねじ不可）")
        tire = put(parts, Pos(0, y, 0) * Rot(90, 0, 0) * L.pick_tire(),
                   f"sing_tire{k}", to=f"sing_hub{k}", how="PRESS",
                   note="ウレタンスリーブをハブへ圧入（摩耗したら単独で交換）")
    LEDGER.add_solid("分離ローラー ハブ PETG（肉抜き）", hub, "PETG", "装填",
                     qty=len(P.PICK_ROLLER_Y))
    LEDGER.add_solid("分離ローラー タイヤ ウレタン t3（消耗品）", tire, "URETHANE",
                     "装填", qty=len(P.PICK_ROLLER_Y))
    # --- 軸継手 -----------------------------------------------------------
    # ⚠ 以前は「OD20/ID8 の管」を置いて「止めねじ 2」と注記していたが、
    #   (1) 軸もモーター軸も**中に入っていない**（端面が触れるだけ）
    #   (2) 中空軸に止めねじは使えない
    #   の二重に成立していなかった。両端クランプ式として、割りと締めねじの
    #   実体まで描く。
    y_c1 = y_c0 + P.SING_CPLG_LEN
    cplg = (Cylinder(P.SING_CPLG_OD / 2, P.SING_CPLG_LEN, align=CTR)
            - Cylinder(P.SING_SHAFT_DIA / 2, P.SING_CPLG_LEN + 2, align=CTR))
    for sz in (1, -1):
        # 割り: 外周から穴まで、端から 12mm ぶん
        cplg -= Pos(0, P.SING_CPLG_OD / 4 + P.SING_SHAFT_DIA / 4,
                    sz * (P.SING_CPLG_LEN / 2 - 6.0)) \
            * Box(1.5, (P.SING_CPLG_OD - P.SING_SHAFT_DIA) / 2 + 2.0, 12.0, align=CTR)
        # 締めねじ M3（割りを跨いで通る）
        cplg -= Pos(0, 7.0, sz * (P.SING_CPLG_LEN / 2 - 6.0)) * Rot(0, 90, 0) \
            * Cylinder(1.7, P.SING_CPLG_OD + 2, align=CTR)
    put(parts, L.mat(Pos(0, (y_c0 + y_c1) / 2, 0) * Rot(90, 0, 0) * cplg, "A5052"),
        "sing_cplg", to="sing_shaft", how="PRESS",
        note="両端クランプ式軸継手（2-M3）")
    LEDGER.add("軸継手 クランプ式 φ8-φ8 (MISUMI CPSW20)", 0.030, "カタログ", "装填")
    # --- モーターの出力軸 -------------------------------------------------
    # ⚠ ここに置くのは**軸だけ**。ハウジングは回らないので `_hopper()` 側。
    #   以前は M2006 をまるごとここに描き、固定先は継手だけだった＝
    #   自分の出力軸で自分をぶら下げていた。
    y_m = F.face("sing_motor", "y", -1)
    put(parts, L.mat(Pos(0, y_m - P.M2006_SHAFT_LEN / 2, 0) * Rot(90, 0, 0)
                     * Cylinder(P.M2006_SHAFT_DIA / 2, P.M2006_SHAFT_LEN, align=CTR),
                     "MOTOR_SHAFT"),
        "sing_rotor", to=("sing_motor", "sing_cplg"), how=("ROTATE", "SHAFT"),
        note="M2006 の出力軸（本体の軸受で受け、継手で掴む）")
    auto_screws(parts, "singulator")
    return Compound(label="singulator", children=parts)


def link_carriage():
    """グラバーのキャリッジ + フォーク（原点 = 車体座標、全閉位置）。"""
    parts = []
    # 可動レール（インナー）は固定側とセットで表現するため、ここではキャリッジのみ
    # 櫛歯は前方へ露出させる必要があるので、キャリッジ板は「後端の横梁」に留める。
    # （板が櫛歯の上を覆っていると、山の下ではなく山の前面を押してしまう）
    # 幅は側金具の内側まで（インナーレール |Y|=320.9 に当てない）
    # ⚠ 以前は dia=46 pitch=76 margin=22。短辺 90 に対して 90-44=46 < 76 で
    #   **穴が 1 つも開いていなかった**（削減率 0%）。短辺に 1 列入る値にする。
    # 幅は**フォーク根元（+X 端 -15.4）と斜路（X=72 で Z=765）のあいだ**に収める。
    # 90 幅だとどちらかに必ず当たる（根元バー 7,635mm³ / 斜路 2,407mm³）。
    # ⚠ 幅を 626（|Y|=313）と手で書いていたが、側金具の内面は
    #   CAR_SIDE_Y - t/2 = 311.9。1.1mm 過走していた。面から取る。
    beam_w = 2 * (P.CAR_SIDE_Y - 2.5)
    # ⚠ φ30 ピッチ 34 では残肉が 4mm しかない。t3 のアルミで幅 4mm の
    #   リブは曲げと振動で裂ける。
    # ⚠ **φ16 を 25 個一列に並べるのをやめた（2026-08-07）。** 幅 40 の帯に
    #   φ16 が 24mm ピッチで 25 個並ぶ形は、機体でいちばん「パンチング板」に
    #   見えていた。しかも 1 個あたり 201mm² しか抜けないので、25 個
    #   開けても削減は 15g。長穴（50×20）にすると 1 個 941mm² で、
    #   7 個で 4.4 倍抜ける。残るのは幅 10mm の梯子状のリブ。
    # ⚠ 補強リブ（`car_rib_*`、|Y|=260 に 40×3 でリベット留め）の座面を
    #   `guards` で残す。ここを穴にすると 4-φ3.2 が空を打つ。
    plate, _rep = L.plate_slots(
        40.0, beam_w, 3.0, length=56.0, width=20.0, rib=12.0, edge=9.0,
        along="y", guards=[(0.0, sy_ * 260.0, 24.0) for sy_ in (1, -1)],
        label="car_beam")
    # 横梁は必ずヒンジより後ろ（+X側）に置く。櫛歯の上に張り出すと、
    # 山の下に入る前に山の前面を押してしまう（シミュレーションで 235mm 押した）
    # ⚠ +20 だとフォークの根元バー（X -43..-15）とヒンジ軸（X -29..-21）を
    #   横梁が貫く（7,635 + 5,459mm³）。フォークは傾斜で動くので必ず逃がす。
    #   横梁の -X 端が根元バーの +X 端（= フォーク原点）より後ろに来る位置へ。
    # ⚠ 斜路の押さえベルト（X 22.9..88.7、Z 719..809）と Z が重なる帯を避ける。
    #   ベルトは斜路に沿って上がるので、X<=20 なら Z<=740 で横梁の下を通る。
    # ⚠ t3 の横梁の**端面**（60×1.1 = 66mm²）で側板に「4-M4」と宣言していた。
    #   側板（Z 766..826）と梁（Z 764..767）は 1mm しか重なっていない。
    #   端を立ち上げて、側板の内面へ板面で当てる。
    # 耳の高さは斜路の座金具（Z 790..818）に触れない 20mm に抑える
    # ⚠ 幅は 40。60（X 1.6..61.6）だと斜路の押さえベルトを**輪として**
    #   描いたときに上側の直線部と 370mm³ ×2 重なる（法線 15.2 以上が
    #   干渉帯に入る。輪は法線 27 まで張り出す）。40（X 1.6..41.6）なら
    #   干渉帯は法線 31.4 以上になり、輪も上げた側板（18.5）も入らない。
    # ⚠ **耳は別部品にする。** 曲げられないので（`export_fab.CAN_BEND`）、
    #   横梁の板と立ち上げの耳を 1 部品にすると作れない。耳は梁の**端面に
    #   タップ**を立てて留める（横穴加工まではできる）。
    org_cb = Pos(P.RAIL_X0 + P.FORK_LEN + 37.0, 0, P.FORK_Z + 2.5)
    plate = put(parts, org_cb * plate,
                "car_beam", to=("car_beam_eL", "car_beam_eR"), how="BOLT",
                note="各 2-M4（耳の下の端面にタップ。梁は貫通）")
    LEDGER.add_solid("キャリッジ後端横梁 A5052 t3（肉抜き）", plate, "A5052", "装填")
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ **耳は t6。タップは耳の側に立てる。** 梁は t3 なので「梁の端面に
        #   M4」は立たない（`L.TAP_MIN[4] = 6.0`）。耳の**下の端面**（横穴）へ
        #   立て、梁はバカ穴にして下から通す。t6 にすると梁との座面も
        #   40×3 = 120mm²（要 150 に足りない）→ 40×6 = 240mm² になる。
        #   側板の内面（|Y| 311.9）に当てる外面は動かさない（中心 310.4 →
        #   308.9）。幅 40 は押さえベルトの輪を避けた値なので変えない。
        # ⚠ **t6 → t8（2026-08-07）。** 梁を貫く M4 は耳の下の端面へ立てる。
        #   下穴 φ4.5 の外に 0.3d = 1.2mm の壁が要る（= t6.9）ので、t6 では
        #   壁が 0.75mm しか残らず、締めた瞬間に側面が裂ける。
        #   側板の内面に当てる外面（|Y| 311.9）は動かさない（中心 308.9 → 307.9）。
        put(parts, L.mat(org_cb * Pos(0, sy * 307.9, 11.5)
                         * Box(40.0, 8.0, 20.0, align=CTR), "A5052"),
            f"car_beam_e{sd}", to=f"car_side_{sd}", how="BOLT", note="4-M4")
        LEDGER.add("キャリッジ後端横梁 耳板 A5052 t8", 0.017, "体積概算", "装填")
    # 曲げ補強リブ（上向き。机天板より上の空間だけを使う）
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ **「4-φ3.2 でリベット留め」は打てない。** リブは梁の上に立って
        #   いて、当たっているのは**下端面**（40×3）だけ。リベットは面に
        #   垂直に打つので、掴み代がリブの高さ 24 + 梁 3 + 1 = φ3.2×28 に
        #   なり、買える上限（25）を超える（`screw_place` がそう言って
        #   4 本とも落としていた）。
        #   → `car_beam_e*` と同じ形にする: リブを **t8** にして下端面に
        #     M4 のタップを立て、梁（t3）はバカ穴にして**下から**通す。
        #     t3 では下穴 φ3.3 の壁が残らない（`L.TAP_MIN[4] = 6.0`）。
        rib = put(parts, Pos(P.RAIL_X0 + P.FORK_LEN + 37.0, sy * 260.0, P.FORK_Z + 16.0)
                  * Box(40.0, 8.0, 24.0, align=CTR),
                  f"car_rib_{sd}", to="car_beam", how="BOLT",
                  note="2-M4（リブの下端面にタップ。梁は貫通）")
        if sy == 1:
            LEDGER.add_solid("キャリッジ補強リブ A5052 t8", rib, "A5052", "装填", qty=2)
        for i_, dx_ in enumerate((-12.0, 12.0)):
            face_screws(parts, [Pos(P.RAIL_X0 + P.FORK_LEN + 37.0 + dx_,
                                    sy * 260.0, P.FORK_Z + 1.0) * Rot(180, 0, 0)],
                        "car_beam", f"car_rib_{sd}", 4, 3.0,
                        f"car_rib_{sd}_{i_}", link="carriage")
    # 上押さえのガイド軸系を受ける金具（左右の側板の内面に付く）
    # ⚠ この金具の上面が、グラバー駆動ベルトのループ（外周半径 23）の
    #   下限を決める。ベルトの上限は旋回アームの平面（862）なので、
    #   窓は「金具の上面 .. 858」。+49（上面 813.5）で 48.5mm 開ける。
    # ⚠ **ここは以前 Y 全幅の横材（car_cross）だった。上押さえプレートが
    #   降りる道を塞いでいた。** プレート（Z 776..883 を上下）とこの横材
    #   （Z 810.5..813.5）はどちらも水平な板なので、すれ違えない。
    #   press=0 と 105 では当たらず、**途中の 62.6 だけ** 57,543mm³ 重なる。
    #   4 点標本（0 / 35 / 70 / 105）では出ず、連続掃引検査で見つかった。
    #   → |Y| < 205 を落として左右 2 枚の金具にし、左右をつなぐ役目は
    #     プレートの可動域より上（Z 911.5）の press_shelf に移す。
    #     キャリッジは「側板 + 柱 + 棚板」の門型で前側を締める。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # 肉抜き（φ26・縁の残肉 22 / 13）。板は 70 × 94.9
        # ⚠ **外端の切り口で側板に留めていた。** 座面は t3 × 70 = 210mm² しか
        #   なく、そこに上押さえ一式 2.3kg が腕 300mm でぶら下がる
        #   （`joint_load.py` が 21.5N·m の曲げとして検出）。板厚 3mm の
        #   切り口にボルトを通す形なので、実機では穴が伸びて金具が垂れる。
        #   → 外端を +Z へ 20mm 立ち上げ、側板の内面に**面で**当てる。
        #     座面 70×20 = 1,400mm²（6.7 倍）。曲げは面のせん断で受ける。
        # ⚠ 立ち上げる向きは **−Z**。+Z 側はグラバー配線のケーブルベアが
        #   占めている（帯は Z 817..828、|Y| 284..311.9）。そこへ耳を
        #   出すと、12mm でも 1,785mm³ 食い込む（20mm では 2,310mm³）。
        #   下側（Z 795..810.5）は側板（Z 766..826）の内面が続いていて、
        #   上押さえのガイド軸（|Y| < 230）とも離れている。
        #   座面は t3 の切り口 210mm² → 70×15 = 1,050mm²（5 倍）。
        # ⚠ **t5 にした分は「下へ」伸ばす（2026-08-05）。** 板の中心を
        #   動かさずに t3 → t5 にしたら、上面が 813.5 → 814.5 へ**上がり**、
        #   グラバー配線のケーブルベア（Z 817..900）まで 2.50mm になって
        #   規定（3mm）を切った。ついでに柱の下フランジ（`press_post_*f0`、
        #   板の上面 813.5 に載る前提）と 400mm³ 食い込み、駆動ベルトとも
        #   0.50mm まで詰まった。**上面 813.5 は動かしてはいけない基準面**。
        #   → 水平板の中心を 1.0 下げて Z 808.5..813.5 にする。
        #     立板は上端をそれに合わせ、下端（795.5）を変えないよう 15 → 13。
        rise = 13.0
        dz_brk = -1.0          # 水平板の中心オフセット（上面を 813.5 に保つ）
        # ⚠ **水平板と立板を別部品にする**（曲げられない）。立板を側板へ
        #   ボルトし、水平板は立板の端面のタップへ留める。
        org_br = Pos(P.RAIL_X0 + 60.0, sy * (P.PRESS_BRK_Y0 + P.PRESS_BRK_W / 2),
                     P.FORK_Z + 49.0)
        # ⚠ **水平板は t3 → t5・締結は皿にする（2026-08-05）。**
        #   立板を t6 にして水平板を貫く M4 を立てたら、その**頭が
        #   グラバー配線のケーブルベア（Z 817..900）に 0.5mm 入った**
        #   （実測: ねじ Z 801.5..817.5 / ベア Z 817.0..900、距離 0.000mm）。
        #   `sweep_fine` が「行程の途中で離れる締結」として出す
        #   （引込み端では 75.9mm 離れるので、置いた姿勢だけの貫通と分かる）。
        #   → 皿もみにして頭を面一にする。t3 では M4 の皿もみ（深さ 2.3mm）で
        #     残肉 0.7mm しか残らないので、板を t5 にする。上面は 813.5 →
        #     814.5 に上がるが、ベアまで 2.5mm 空く。
        brk = put(parts, L.mat(org_br * Pos(0, 0, dz_brk) * (
            L.plate(70.0, P.PRESS_BRK_W, 5.0)
            - Pos(0, -P.PRESS_BRK_W / 4, 0) * Cylinder(13.0, 8.0, align=CTR)
            - Pos(0, P.PRESS_BRK_W / 4, 0) * Cylinder(13.0, 8.0, align=CTR)),
            "A5052"),
            f"car_brk_{sd}", to=f"car_brk_{sd}v", how="BOLT",
            note="2-M4 皿（立板の上の端面にタップ。水平板は皿もみ）")
        # ⚠ **立板は t3 → t6 → t8。** 水平板を貫く M4 は立板の**上の端面**へ
        #   立てる。t3 の小口には M4 のタップが立たず（`L.TAP_MIN[4] = 6.0`）、
        #   t6 でも下穴 φ4.5 の外に残る壁が 0.75mm しかない。
        #   **0.3d = 1.2mm の壁**を残すには t6.9 要るので t8 にする
        #   （`scripts/screw_seat_check.py`）。座面も 70×8 = 560mm²。
        #   側板の内面に当てる外面（|Y| = PRESS_BRK_W/2）は動かさない。
        put(parts, L.mat(org_br
                         * Pos(0, sy * (P.PRESS_BRK_W - 8.0) / 2,
                               dz_brk - 2.5 - rise / 2)
                         * Box(70.0, 8.0, rise, align=CTR), "A5052"),
            f"car_brk_{sd}v", to=f"car_side_{sd}", how="BOLT", note="3-M4")
        LEDGER.add_solid("キャリッジ 上押さえ受け 水平板 A5052 t5", brk, "A5052", "装填")
        # 70 × 13 × 8mm（rise を 15 → 13 に詰めた分）
        LEDGER.add("キャリッジ 上押さえ受け 立板 A5052 t8", 0.020, "体積概算", "装填")
    brackets = []
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # インナーレールの内側面（|Y|=320.9）より内側に置く
        # ⚠ 長さは**後端横梁まで届くこと**。横梁をフォークから逃がして +32 した
        #   ので、FORK_LEN のままでは側金具が横梁に届かない。
        br_len = P.FORK_LEN + 90.0
        # ⚠ **t5 のまま据え置く。** `plate_audit` は「安全率 42、t2 まで
        #   落とせる」（−129g）と言い、実際に t3 にして試した。結果は
        #   **離れ 11 件**。この板は put で**中心**を `CAR_SIDE_Y` に置いて
        #   いるので、厚みを変えると両面が 1mm ずつ動き、外面に留まる
        #   インナーレールと、内面に留まる car_beam / car_brk / 配線が
        #   まとめて離れる。
        #   ⚠ 板厚を変えられるのは、**相手が `F.face()` で面を参照している
        #     板だけ**。座標を手で書いている相手がいる板は、板厚の変更が
        #     そのまま組立の破壊になる。ここを直すなら相手の面参照への
        #     書き換えとセットでやること（86g のために 11 か所を動かす価値が
        #     あるかは、§7 のレバー（kg 単位）を先にやってから判断する）。
        car_t = 5.0
        # ⚠ **φ28 を 8 個並べるのをやめた（2026-08-07）。** 穴の上下に
        #   残る肉（16mm）と穴どうしの残肉（12mm）が食い違っていて、
        #   細いほうで強度が決まるのに太いほうが目立つ形だった。
        #   長穴にすると縁のリブ 10mm・梯子のリブ 12mm で幅が揃う。
        #   ⚠ 穴の**上下の張り出しは φ28 のときと同じ ±14** に留めること。
        #     ここを広げると、縁を通っている締結（耳板・受け金具）の座面に
        #     食い込む。長さだけ伸ばして抜く量を増やす。
        br_pl, _rep = L.plate_slots(br_len, 60.0, car_t, length=50.0,
                                    width=28.0, rib=12.0, edge=10.0,
                                    along="x", label=f"car_side_{sd}")
        br = put(parts, Pos(P.RAIL_X0 + br_len / 2, sy * P.CAR_SIDE_Y, P.FORK_Z + 33.0)
                 * Rot(90, 0, 0) * br_pl,
                 f"car_side_{sd}", to=f"rail_in_{sd}", how="BOLT", note="インナーレールへ 6-M4")
        brackets.append(br)
    LEDGER.add_solid("キャリッジ側金具 A5052 t5", brackets[0], "A5052", "装填", qty=2)
    # フォーク傾斜ヒンジ（カム駆動・モーターなし）
    for sy, y in ((1, P.CAR_SIDE_Y - 2.5 - 12.0), (-1, -(P.CAR_SIDE_Y - 2.5 - 12.0))):
        sd = "L" if sy > 0 else "R"
        # ⚠ 軸受座の穴は**軸と同心**に。-10 / +6 のずらしを入れていたので
        #   穴の中心が軸から (10, 2) ずれ、軸が穴の縁に 24mm³ ×2 当たって
        #   いた。軸は関節原点（RAIL_X0+FORK_LEN, FORK_Z+FORK_HINGE_UP）に
        #   あるので、座もそこに置く。
        put(parts, Pos(P.RAIL_X0 + P.FORK_LEN, y,
                       P.FORK_Z + P.FORK_HINGE_UP)
            * (Box(24.0, 24.0, 20.0, align=CTR)
               # ⚠ 軸は中空パイプ（外径 8・内径 4）。穴を軸と同心にしたので
               #   クリアは 0.2mm で足りる。0.5 だと接触判定のしきい値
               #   ちょうどになり、姿勢で「離れ」に反転する。
               - Rot(90, 0, 0) * Cylinder(4.2, 26.0, align=CTR)),
            f"hinge_blk_{sd}", to=f"car_side_{sd}", how="BOLT", note="2-M4")
    LEDGER.add("傾斜ヒンジ軸受 + 戻しばね + カム", 0.100, "概算", "装填")
    # 上押さえの昇降ガイド（ラック&ピニオン + ガイド軸2本）
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ ガイド軸は X=-145 にあり、後端横梁（X=+17）とは 160mm 離れている。
        #   「横梁に圧入」と宣言していたが実体は触れていない。専用の横材を渡す。
        # ⚠ 櫛歯は y = 0, ±110, ±220（幅 20）。|Y|=120 は 110 の歯（100..120）に
        #   当たる。歯の間の空き帯（20..100）に置く。
        # ⚠ 軸長はストロークで決まる。プレートは退避 FORK_Z+118 から
        #   105 下がって FORK_Z+13 まで来るので、軸下端はそれより下に要る。
        # ⚠ **軸はブッシュの行程の外でしか受けられない。** 行程は
        #   Z 764..896（press 0..105）で、その内側で受けるとブッシュが
        #   受け金具に必ず当たる（以前は Z 810.5 の横材に圧入していた）。
        #   下（Z<764）は櫛歯が傾斜で掃く帯なので受けられない。
        #   → 棚板（Z 911.5..934）から**片持ちで吊る**。
        #   φ10 鋼・張り出し 152mm・側力 20N でたわみ 0.19mm。
        # ⚠ 上押さえを 33mm 上げたので軸も伸ばす。ブッシュの行程は
        #   Z 764..929、棚板の割りブロックの上端は 967。軸は 763..968 に取る
        #   （下端 763 は櫛歯の上面。752 まで下げると机の天板 760 に当たる）。
        # ⚠ 中空（φ10 外径・φ6 内径）。機体の他の軸と同じ。無垢だと 2 本で
        #   0.25kg あり、35.0kg 制限に対して効く。断面二次は 1 − (6/10)^4 =
        #   87% 残るので、たわみは 0.19 → 0.22mm にしかならない。
        put(parts, Pos(P.RAIL_X0 + 60.0, sy * 60.0, P.FORK_Z + 102.5)
            * (Cylinder(5.0, 205.0, align=CTR)
               - Cylinder(3.0, 209.0, align=CTR)),
            f"press_guide_{sd}", to="press_shelf", how="CLAMP",
            note="上端を棚板の割りブロックでクランプ")
    # 上押さえ駆動 M2006 とそのマウント。
    # ⚠ **モーターだけ置いてマウントを描いていなかった**ので 27mm 浮いていた。
    #   質量表の「ラック&ピニオン + ガイド軸 0.22kg」には受け金具が
    #   含まれているつもりだったが、形が無ければ位置の妥当性は検証できない。
    mx = P.RAIL_X0 + 60.0
    # 棚板: ガイド軸(|Y|=120)からモーター取付面(Y=240)まで渡す
    # ⚠ 上押さえプレート（FORK_Z+111.5、下へ 80mm 動く）の**可動域の外**へ置くこと。
    #   FORK_Z+107 では上押さえパッドと 15,330mm³ 重なっていた。
    # ⚠ t3 の板が丸軸に当たる面積は 100mm² しかない。軸を抱くブロックを
    #   板と一体にして座面を出す。
    # ⚠ 棚板は**キャリッジ前側の横つなぎでもある**。上押さえプレートの
    #   可動域（Z 776..883）より上を通るので、ここでしか左右をつなげない。
    #   左右 2 本のガイド軸を吊り、両側の柱に載る。
    z_post0 = P.FORK_Z + 49.0 + 1.5      # 受け金具の上面
    z_post1 = P.FORK_Z + 183.0 - 1.5     # 棚板の下面（+33）
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        yc = sy * (P.PRESS_POST_Y0 + P.PRESS_POST_W / 2)
        # 縦板 + 上下フランジ（t3 の曲げ物）。板の端面 150mm² では座面が
        # 足りない（seat_min[BOLT]=150 ちょうどで判定が反転する）。
        # 縦板は肉抜き（φ26 を 2 つ。縁の残肉 12）
        # ⚠ **コの字（立板 + 上下の耳）を 1 部品にしない。** 曲げられないので
        #   （`export_fab.CAN_BEND`）、立板 1 枚と水平板 2 枚に分け、
        #   水平板を立板の**端面のタップ**へ留める。3 部品 + M4 4 本になるが、
        #   コの字は 1 個も作れないので、これが実物になる唯一の形。
        # ⚠ **立板は t3 → t6。** 上下のフランジを貫く M4 は立板の端面へ
        #   立てることになるが、t3 の小口には M4 のタップが立たない
        #   （`L.TAP_MIN[4] = 6.0`）。座面も 50×3 = 150mm²（判定がちょうど
        #   反転する値）→ 300mm² になる。内側の面は動かさない。
        web = (Pos(mx, sy * (P.PRESS_POST_Y0 + P.PRESS_POST_W - 3.0),
                   (z_post0 + z_post1) / 2)
               * (Box(50.0, 6.0, z_post1 - z_post0 - 6.0, align=CTR)
                  - Rot(90, 0, 0) * Cylinder(13.0, 8.0, align=CTR)
                  - Pos(0, 0, 34.0) * Rot(90, 0, 0)
                  * Cylinder(13.0, 8.0, align=CTR)
                  - Pos(0, 0, -34.0) * Rot(90, 0, 0)
                  * Cylinder(13.0, 8.0, align=CTR)))
        # ⚠ 留まるのは受け金具の**立板**（`car_brk_*v`）。2026-08-05 に
        #   受け金具を水平板と立板へ分けたので、`car_brk_{sd}`（水平板・
        #   板厚 Z）に「ボルト」と書くと 3mm の小口に留めることになる。
        # ⚠ **立板は受け金具に直接は触れない。** 実測: 立板 Y 223..226 /
        #   Z 816.5..941.5、受け金具の立板 Y 308.9..311.9 で **83mm 離れて
        #   いる**。荷重の鎖は
        #     car_brk（水平板）→ 下フランジ f0 → 立板 → 上フランジ f1 → 棚板
        #   なので、立板の固定先は**下フランジ**。`car_brk_*v` と書いたのは
        #   分割のときの取り違え（`assembly_check` が「接していない」で落とした）。
        pst = put(parts, L.mat(web, "A5052"), f"press_post_{sd}",
                  to=f"press_post_{sd}f0", how="BOLT", note="2-M4")
        LEDGER.add_solid("上押さえ 棚板受け 立板 A5052 t6", pst, "A5052", "装填")
        for j, zf in enumerate((z_post0 + 1.5, z_post1 - 1.5)):
            fl = Pos(mx, yc, zf) * Box(50.0, P.PRESS_POST_W, 3.0, align=CTR)
            # ⚠ 下フランジ(f0)は**受け金具の水平板**に載る。上フランジ(f1)は
            #   立板の端面タップへ。f0 まで「立板へ」にすると、立板と f0 が
            #   相互に留まり合って固定の循環になる。
            put(parts, L.mat(fl, "A5052"), f"press_post_{sd}f{j}",
                to=(f"car_brk_{sd}" if j == 0 else f"press_post_{sd}"),
                how="BOLT",
                note="2-M4（受け金具へ）" if j == 0 else "2-M4（立板の端面にタップ）")
            LEDGER.add("上押さえ 棚板受け 水平板 A5052 t3", 0.011, "体積概算", "装填")
    # ⚠ 幅は 56。70 だと棚板だけで 0.25kg あり、全体 34.98/35.0 に対して
    #   余裕が無くなる。軸クランプのブロック（40 幅）が載れば足りる。
    # 肉抜き。⚠ 穴は軸クランプ（|Y| 50..70）と柱の座面（|Y| 201..228）を
    #   避ける位置に置く。等間隔に割り付けると |Y|=66 に当たる。
    # ⚠ **φ24 を 3 個ずつ並べていたのをやめた（2026-08-07）。** 幅 56 の帯に
    #   丸を並べると、穴の左右に残る肉（16mm）と穴どうしの残肉（6mm）が
    #   てんでばらばらになる。帯板は三角格子の内接円が入らない（幅 56 に
    #   対して一辺 56 の三角の内接円は 16mm しかない）ので、長手に沿った
    #   長穴にする。残るリブは「両側の縁 + 穴と穴の梯子」で幅が一定になる。
    shelf_pl, _rep = L.plate_slots(
        56.0, 2 * 247.0, 3.0, length=40.0, width=28.0, rib=12.0, edge=8.0,
        along="y", offset=113.0,
        # 軸クランプのブロック（40×20、|Y| 50..70）と柱の座面（|Y| 201..228）
        guards=[(0.0, sy_ * 60.0, 22.0) for sy_ in (1, -1)]
        + [(0.0, sy_ * 214.5, 20.0) for sy_ in (1, -1)],
        label="press_shelf")
    # ガイド軸（φ10）が抜ける穴
    for sy_ in (1, -1):
        shelf_pl -= Pos(0, sy_ * 60.0, 0) * Cylinder(5.1, 8.0, align=CTR)
    shelf = put(parts, Pos(mx, 0.0, P.FORK_Z + 183.0) * shelf_pl,
                # ⚠ 棚板が載るのは柱の**上の水平板**（`…f1`）。立板の端面
                #   （t3）に「ボルト」と書くと 3mm の小口へ留めることになる。
                "press_shelf", to=("press_post_Lf1", "press_post_Rf1"),
                how="BOLT", note="各 2-M4")
    LEDGER.add_solid("上押さえ駆動 棚板 A5052 t3", shelf, "A5052", "装填")
    # ⚠ ガイド軸の**割締めブロック**は棚板と一体で描いていたが、
    #   Box(40,20,24) に φ10.2 の穴を通した「割締め」は 3D の削り出しになる。
    #   自校では作れない（`export_fab.CAN_MILL_3D = False`）ので、
    #   **市販のシャフトクランプ**（φ10 用）を棚板の上にボルトで載せる。
    for sy_ in (1, -1):
        sd_ = "L" if sy_ > 0 else "R"
        # ⚠ **クランプは棚板の上に「載せる」。** 前は中心を棚板中心 +9 に
        #   置いていたので、高さ 24 の下半分が棚板（t3）へ 3mm 食い込んで
        #   いた（40×20×3 = 2,400mm³ の外接、実測 2,155mm³）。買ってきた
        #   クランプが板に埋まることはない。棚板の上面（+1.5）＋クランプの
        #   半分（12）＝ +13.5 に置く。軸は棚板の φ10.2 穴を通って上へ抜ける
        #   ので、掴む高さが 4.5mm 上がるだけ。
        put(parts, Pos(mx, sy_ * 60.0, P.FORK_Z + 183.0 + 1.5 + 12.0)
            * L.mat(Box(40.0, 20.0, 24.0, align=CTR)
                    - Cylinder(5.1, 26.0, align=CTR), "STEEL"),
            f"press_clamp_{sd_}", to="press_shelf", how="BOLT",
            note="2-M4。φ10 シャフトクランプ（購入）")
        LEDGER.add("φ10 シャフトクランプ", 0.040, "カタログ", "装填")
    # 面板: モーターのフランジ面(Y=240)に当てる
    # 棚板は Y 117..247。面板はその端面（247）に立てる（238.5 だと板の中）
    # ⚠ M2006 は取付面から軸が -Y へ 12 出る。面板（Y247..250）には
    #   その逃げ穴が要る。無いと軸が板を 151mm³ 貫く。
    face_pl = L.plate(70.0, 60.0, 3.0) - Rot(90, 0, 0) * Cylinder(9.0, 12.0, align=CTR)
    face = put(parts, Pos(mx, 248.5, P.FORK_Z + 205.0) * Rot(90, 0, 0) * face_pl,
               "press_face", to="press_shelf", how="RIVET", note="4-φ3.2")
    LEDGER.add_solid("上押さえ駆動 面板 A5052 t3", face, "A5052", "装填")
    # 面板は Y 247..250。m2006 は Rot(90,0,0) で本体が +Y へ出るので、
    # 取付面を面板の外面 250 に置く
    # m2006 は Rot(90,0,0) で本体が -Y、軸が +Y。取付面を面板の内面 247 に置くと
    # 本体が -Y へ出て面板と重ならない
    # m2006 は Rot(90,0,0) で 取付面から本体が **+Y へ 63**、軸が -Y へ 12。
    # 面板は Y 247..250。取付面を 250（面板の外面）に置けば本体は 250..313 で
    # 面板と重ならない。
    # 取付面は面板の外面（247+3=250）ちょうど。250.5 では 0.5mm 浮く
    pm_at = Pos(mx, 250.0, P.FORK_Z + 205.0) * Rot(90, 0, 0)
    put(parts, pm_at * L.m2006(),
        "press_motor", to="press_face", how="BOLT", note="4-M3 PCD26")
    flange_screws(parts, pm_at, "press_face", "press_motor",
                  P.M2006_BOLT_PCD, 4, 3, 3.0, "press_mot", link="carriage")
    # ベルトクランプ（駆動ベルトを掴む腕）— プーリ間 X=-450..+20 の内側に収まる位置
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # ⚠ |Y|=240・幅20 だと櫛歯（y=220±10）と駆動軸（x=-60±5）の両方に触れる。
        #   クランプはキャリッジと一緒に 316mm 動くので、擦れは即故障になる。
        # ⚠ クランプは**ベルトの直線部**を掴む部品。駆動プーリ（X=-60）や
        #   アイドラ（X=-430）の位置に置くとプーリを飲み込む（5,185mm³）。
        #   プーリ間の直線部（X -400..-90）に置くこと。
        # ⚠ クランプはキャリッジ側板（|Y| 311.9..316.9）まで届いて
        #   はじめてキャリッジと一緒に動く。ベルト（|Y| 240）から
        #   側板までの 56mm を腕で渡す。
        arm_l = (P.CAR_SIDE_Y - 2.5) - (BELT_Y + 8.0)
        # ⚠ 1 本の柱でベルトの高さまで立てていたので、**ベルトの下側の
        #   直線部（Z 815..818）を柱が突き抜けて** 675mm³ ×2 重なっていた。
        #   注記は「ベルトを歯で挟む板 2 枚」なのに、挟む形になっていない。
        #   柱をベルトの下面（815）で止め、ベルトの外側（|Y| 248..256）を
        #   通る立ち上がりで上へ回し、上面（818）に蓋を載せて挟む。
        b_lo = BELT_Z - 23.0        # ベルト下側直線部の下面（外周半径 20+3）
        b_hi = BELT_Z - 20.0        # 同 上面
        put(parts, Pos(BELT_CLAMP_X, sy * (BELT_Y + 8.0),
                       (P.FORK_Z + b_lo) / 2)
            * (Box(BELT_CLAMP_W, 16.0, b_lo - P.FORK_Z, align=CTR)
               # ベルトの外側を回る立ち上がり（|Y| 248..256）
               + Pos(0, sy * 4.0, (b_lo + b_hi) / 2 - (P.FORK_Z + b_lo) / 2)
               * Box(BELT_CLAMP_W, 8.0, b_hi - b_lo, align=CTR)
               # 上の蓋（ベルトの上面に載る）
               + Pos(0, 0, b_hi + 1.5 - (P.FORK_Z + b_lo) / 2)
               * Box(BELT_CLAMP_W, 16.0, 3.0, align=CTR)
               # ⚠ 腕は側金具の内面（CAR_SIDE_Y - t/2）まで。313 と手で
               #   書いていたので 1.1mm 過走していた。
               #   高さも側金具（Z 766..826）と 20mm 重ねる。以前は
               #   1mm しか重ならず M4 の座面が 60mm² しか出ていなかった。
               + Pos(0, sy * arm_l / 2, 12.0)
               * Box(BELT_CLAMP_W, arm_l, 20.0, align=CTR)),
            f"belt_clamp_{sd}",
            to=(f"belt_grab_{sd}", f"car_side_{sd}"), how=("CLAMP", "BOLT"),
            note="ベルトを歯で挟む板 2 枚 + 2-M4")
    LEDGER.add("ベルトクランプ A5052", 0.060, "体積概算", "装填", qty=2)
    LEDGER.add("M2006 P36 (上押さえ)", P.M2006_MASS, "DJI カタログ", "装填")
    LEDGER.add("ラック&ピニオン + ガイド軸（軸は中空）", 0.190, "概算", "装填")
    auto_screws(parts, "carriage")
    return Compound(label="grabber_slide", children=parts)


def link_fork():
    """櫛歯フォーク（原点 = 傾斜ヒンジ軸、櫛歯は -X 方向へ伸びる）。

    引込み端で固定カムがヒンジを押し下げ、山をホッパーへ滑落させる。
    傾斜そのものにモーターは要らない（カム + 戻しばね）。
    """
    parts = []
    tines = []
    for i in range(P.FORK_TINES):
        y = (i - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH
        # 歯先は**下面を水平のまま延長し、上面だけ**を削ってナイフエッジにする。
        # 以前は 8°・30mm で下面ごと下げていて、先端が机の天板より 1.1mm 下にあった。
        # あれは天板の**縁に正面から当たって机を押す**形（ルール 4.1.4a の反則）。
        # 下面を水平に保てば、縁は天板 +1.0mm で越える（scripts/fork_clearance.py）。
        # ⚠ 長さは FORK_LEN + FORK_TINE_EXT。FORK_LEN 自体を増やすとヒンジ
        #   （RAIL_X0 + FORK_LEN）も動いて先端が動かない。
        tines.append(put(parts, Pos(0, y, -P.FORK_HINGE_UP) * L.fork_tine(
            P.FORK_LEN + P.FORK_TINE_EXT,
            P.FORK_TINE_W, P.FORK_T, P.FORK_TIP_LEN, P.FORK_TIP_T),
            # ⚠ **皿リベットでなければならない。** 丸頭は頭が 1.2mm 出る。
            #   頭が座るのは薄いほう＝櫛歯で、櫛歯が根元バーに触れているのは
            #   上面なので、頭は**下面**に出る。すると歯の下面 760.5 に対して
            #   実形状の最下点が 759.30 になり、机の天板 760 より下を通る
            #   ＝**縁に正面衝突して机を押す**（規定 4.1.4a）。
            #   歯の下面は「何も出てはいけない面」なので、120° の座ぐりに
            #   沈めて面一にする（t2.5 に対し頭の高さ 0.87mm。残り 1.6mm）。
            f"tine{i}", to="fork_root", how="RIVET",
            note="根元バーに 2-φ3 皿リベット（下面は面一）"))
    # 傾斜ヒンジ軸。**フォークと一緒に回る**ので fork リンク側に置く。
    # 軸受ブロックはキャリッジ側（ROTATE でリンクをまたぐ）。
    # 軸受ブロックは |Y| 288..312。軸はその内側 288 までで止め、
    # ブロックの穴で受ける（620 だと 310 までブロックを貫く）
    # ⚠ **ヒンジ軸は関節の回転原点そのものに置く。** 局所 (-10, 0, 6) に
    #   置いていたので、チルト 22° で軸が公転して（world で X -22.4 /
    #   Z 772.3 へ移動）、動かない軸受ブロックの穴の縁に 21mm³ ×2 当たって
    #   いた。軸が回転軸と一致していれば、チルトしても軸は動かない。
    #   `JOINTS["fork_tilt"]["origin"]` がこのリンクの原点なので、局所では
    #   原点 (0,0,0) がヒンジ軸。
    put(parts, Rot(90, 0, 0)
        * (Cylinder(4.0, 2 * 289.0, align=CTR) - Cylinder(2.5, 2 * 292.0, align=CTR)),
        "hinge_shaft", to=("hinge_blk_L", "hinge_blk_R"), how="ROTATE")
    # 櫛歯を束ねる根元バー + カムフォロア
    # 幅はヒンジブロック（|Y| 288..312）の内側まで。620 だと 1,318mm³ 食い込む
    root = (Pos(-14.0, 0, 1.0 - P.FORK_HINGE_UP) * Box(28.0, 560.0, 2.0, align=CTR)
            + Pos(-1.0, 0, 11.0 - P.FORK_HINGE_UP)
            * Box(2.0, 560.0, 22.0, align=CTR))
    # ⚠ 立ち上げ板（厚さ 2）が**軸を丸ごと飲み込んでいた**（3,646mm³）。
    #   「軸に割締め」と書いてあるのに、割締めの実体が無く板が軸を貫通。
    #   PRESS の許容 4,000 の陰に隠れていたが、ビューアでは干渉に見える。
    # ⚠ 板に軸の逃げ穴を開けるだけでは**板が上下に分断される**（厚さ 2 に
    #   対し軸は φ8）。櫛歯の位置に割締めブロックを置いて軸を抱かせ、
    #   板はブロックで繋ぐ（実物も 5 か所の割締めで留める）。
    # ⚠ ブロックの下端は局所 -8（world 763）までにする。-12 まで下げると
    #   櫛歯の下面（760.5）より下に出て机の天板（760）に当たる。
    root -= Rot(90, 0, 0) * Cylinder(4.0, 2 * 300.0, align=CTR)
    # ⚠ **割締めブロック 5 個は別部品**（2026-08-05）。前は根元バーと 1 個の
    #   ソリッドにしていたが、Box(12,20,20) に φ8 の穴を通した割締めは
    #   **3D の削り出し**で、自校では作れない（`export_fab.CAN_MILL_3D`）。
    #   実物も「5 か所の割締めで留める」と書いてあるとおり、市販の
    #   **φ8 シャフトクランプ**を根元バーへボルトで留めるのが素直。
    #   これで根元バーは「底板 + 立ち上げ板」の平板 2 枚になる。
    # ⚠ **底板と立ち上げ板は別部品にする（2026-08-05）。** 割締めブロックを
    #   購入品へ出した結果、軸の逃げ穴のところで root が 2 塊に分かれた
    #   （`assembly_check` の「分断」）。上のコメントが warning していた
    #   「板に軸の逃げ穴を開けるだけでは板が上下に分断される」がそのまま
    #   起きた状態。ブロックで繋がなくなった以上、**2 枚の平板**として
    #   置き、立板を底板の端面タップへ留めるのが素直（どちらも切り抜きで作れる）。
    # ⚠ **`root.solids()` で分けても平板にならない。** 軸の逃げ穴（φ8、
    #   局所 Z -4..4）が立板を上下に割るので、塊は
    #     塊1 = 底板（Z -8..-6）+ 立板の下部（-8..-4）  ← **L 字**
    #     塊2 = 立板の上部（4..14）
    #   になる。塊1 は曲げ板判定のまま（実際 `FAB.md` で「曲げ板」と出た）。
    #   → **最初から 2 枚の平板として作る**。底板は Z -8..-6 で軸（-4..4）に
    #     当たらないので逃げ穴が要らない。立板は軸より上（4..14）だけにする。
    #     2 枚はクランプ 5 個が繋ぐ（元のコメントの「板はブロックで繋ぐ」）。
    # ⚠ **底板は t3。t2 にしてはいけない。** 皿リベットの頭は
    #   `screw_place` が「薄いほうの板」に入れる（`head_a = ta <= tb`）。
    #   底板 t2 < 櫛歯 t2.5 だと頭が**底板側（上）**に入り、軸が櫛歯を
    #   貫いて**下へ 1.5mm 出る**（world 759.00。机の天板 760 より下）。
    #   ⚠ §32 のとおり櫛歯の下面は「何も出てはいけない面」。頭は必ず
    #     櫛歯側（下）に入れ、皿で面一にすること。
    #   t3 なら 3 > 2.5 で頭が櫛歯側になり、最下点は 760.50 に戻る。
    #   下端（局所 -8）は変えない（櫛歯との接触が動くので）。
    base_pl = Pos(-14.0, 0, 1.5 - P.FORK_HINGE_UP) * Box(28.0, 560.0, 3.0, align=CTR)
    # ⚠ **立板は「軸より上」ではなく「軸の脇」に置く。** 前は
    #   `Pos(-1, 0, 9 - FORK_HINGE_UP)` = 局所 X -2..0 / Z -4..6 で、
    #   ヒンジ軸（局所 X -4..4 / Z -4..4）を**560mm 全長にわたって貫いて**
    #   いた（実測 3,646mm³ の食い込み）。上のコメントの「立板は軸より上
    #   （4..14）だけにする」は `- P.FORK_HINGE_UP` を引いた時点で崩れていた
    #   （UP=8 なので実際は -4..6）。**軸を跨ぐのはクランプの仕事**なので、
    #   立板は軸を避けてクランプの -X 側の面（局所 X = -12）に突き当てる。
    #     クランプ 局所 X -12..0    → 立板 X -15..-12（突き合わせ、M3 は水平）
    #     カムフォロア X -38.8..-20.8 → 5.8mm 空く
    #     底板の上面 Z -5 に載せ、クランプと同じ Z -5..15 まで立ち上げる。
    web_pl = Pos(-13.5, 0, 5.0) * Box(3.0, 560.0, 20.0, align=CTR)
    # ⚠ 底板は軸に**直接は触れない**（軸は局所 Z -4..4、底板は -8..-5）。
    #   「軸に割締め」と書くと `assembly_check` が「宣言した固定先と
    #   接していない」で落とす。実際に軸を抱くのは**クランプ**なので、
    #   底板はクランプに留まると書くのが正しい（2026-08-05）。
    put(parts, L.mat(base_pl, "A5052"), "fork_root",
        to=tuple(f"fork_clamp{i}" for i in range(P.FORK_TINES)),
        how="BOLT", note="各 2-M3（クランプを底板へ）")
    # ⚠ **底板にボルトで留めてはいけない。** 底板は t2 で、M3 のタップが
    #   立たない（`L.TAP_MIN[3] = 4.5`）。しかも上下方向のボルトになるので
    #   頭か先端が底板の**下**に出る。実際そう宣言したら、フォーク実形状の
    #   最下点が 760.50 → **756.27** に落ちた（机の天板 760 より 3.7mm 下）。
    #   §32 でわざわざ下面を水平に保った意味が消える。
    #   → 立板は**割締めクランプ**（20mm 角、M3 のタップが立つ）へ
    #     **X 方向**に留める。ねじが水平になるので下に出ない。
    put(parts, L.mat(web_pl, "A5052"), "fork_root_v",
        to=tuple(f"fork_clamp{i}" for i in range(P.FORK_TINES)),
        how="BOLT", note="各 1-M3（クランプの側面へ水平に）")
    for i, yb in enumerate([(j - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH
                            for j in range(P.FORK_TINES)]):
        # ⚠ **クランプは底板の上に載せる。** 前は Z 中心 2.0（局所 -8..12）で
        #   底板（-8..-5）と 3mm ぶん重なっていた（実測 480mm³、PRESS の許容
        #   300mm³ 超過）。買ってきたクランプが板に埋まることはあり得ない。
        #   Z 中心 5.0（-5..15）にすると底板の上面にちょうど載り、
        #   軸（-4..4）は変わらず抱ける。
        put(parts, L.mat(Pos(-6.0, yb, 5.0) * Box(12.0, 20.0, 20.0, align=CTR)
                         - Pos(0, yb, 0) * Rot(90, 0, 0)
                         * Cylinder(4.0, 24.0, align=CTR), "STEEL"),
            # ⚠ 底板との締結は底板の側で宣言済み。ここにも書くと循環する。
            f"fork_clamp{i}", to="hinge_shaft", how="PRESS",
            note="φ8 シャフトクランプ（購入）。根元バーへ 2-M3")
        LEDGER.add("φ8 シャフトクランプ", 0.020, "カタログ", "装填")
    # ⚠ 根元バーは局所 Z 0..22。26 に置くとバーから 9mm 浮く。
    #   バーの上面に軸心を載せる。
    # ⚠ 角に対して斜め 45° に置くと、半径 9 では 12.7mm 届かない。
    #   中心の Z は相手の**高さ範囲の中**に取り、X だけで寄せる。
    # ⚠ **Z は `fork_root_v`（立板）の高さ範囲で取る。** 2026-08-05 に
    #   根元バーを底板と立板へ分けたので、`fork_root` の Z 範囲は
    #   −8..14 → **−8..−6** に縮んだ。中心を `fork_root` で取ると 3 → −7 に
    #   落ち、半径 9 のローラーが局所 −16（world 755）まで下がる。
    #   櫛歯の下面は 760.5、机の天板は 760 なので **5mm 食い込む**。
    #   実際そうなった（`validate` の「フォーク実形状の最下点 755.00」）。
    #   ⚠ **部品を分けると、その部品の面を参照している相手が動く。**
    #     `F.face()` で位置を決めている箇所は、分割のたびに見直すこと。
    # ⚠ **Z は「底板の上面 + 半径」で取る。** 立板の Z 中心に置いていたので、
    #   ローラー（半径 9）の下側が底板（world 763.58..776.85）へ 2.76mm
    #   埋まっていた（`assembly_check` の ROTATE 食い込み 169mm³）。
    #   回る部品が板に埋まっていたら回らない。カムは**上から**押し下げる
    #   固定カムなので、ローラーは底板の上面に載っているのが正しい。
    put(parts, L.mat(Pos(F.face("fork_root", "x", -1) - 1.8, 0,
                         F.face("fork_root", "z", 1) + 9.0)
                     * Rot(90, 0, 0) * Cylinder(9.0, 40.0, align=CTR),
                     # カムフォロワは**回る**部品。回り止めは要らない。
                     "PETG"), "cam_follower", to="fork_root", how="ROTATE",
        note="M6 軸 + フランジブッシュ")
    # 質量は形状から取る（すくい斜面を伸ばした分がそのまま効くので、固定値にしない）
    LEDGER.add_solid("フォーク櫛歯 A5052 t2.5", tines[0], "A5052", "装填", qty=P.FORK_TINES)
    LEDGER.add_solid("櫛歯根元バー A5052", root, "A5052", "装填")
    auto_screws(parts, "fork")
    return Compound(label="fork_tilt", children=parts)


def link_press():
    """上押さえプレート（原点 = 車体座標、下限位置）。"""
    x, y, t = P.PRESS_PLATE
    # 上押さえは雑巾を軽く押さえるだけ（数十グラム）。無垢である必要はない
    parts = []
    # ⚠ ガイド軸を X=-215 へ動かしたので、プレートも合わせる。
    #   -145 のままだと +X 端が -45 まで来て、ヨー -30° で振れた旋回アーム
    #   （X=-55 まで来る）と当たる。
    # ⚠ **丸穴のグリッドをやめた（2026-08-07）。** φ44 を 3×3 に並べていたが、
    #   残る材料が「穴と穴のすきま」でしかなく、幅が場所ごとにばらついた
    #   （設計値ではなく径とピッチの引き算の余りで決まっていた）。
    #   アイソグリッド（正三角形の格子）にすると、リブ幅は板のどこでも
    #   8mm ちょうどになり、面内のせん断にも筋交いとして効く。
    #   ここは機体でいちばん平らで大きく見える板なので、見た目の効きも大きい。
    # ⚠ 軸穴（|Y|=60）の座は `guards` で必ず残す。丸穴のころ、割り付けが
    #   変わって座が両側から食われ「分断 3 塊」「離れ 2 件」になった。
    #   格子でも同じことが起きるので、座は穴の割り付けに任せない。
    # 格子の対称軸は板の X 軸（`angle=90`）。軸穴が |Y|=60 の左右対称に
    # あるので、上下対称にしたほうが割り付けが素直に見える。
    pl, _rep = L.plate_iso(
        x, y, t, cell=77.0, rib=8.0, edge=10.0, angle=90.0,
        guards=[(0.0, sy_ * 60.0, 15.0) for sy_ in (1, -1)],
        label="press_plate")
    # ガイド軸（|Y|=60、φ10）が通る穴。無いと軸が板を 157mm³ ×2 貫く
    for sy_ in (1, -1):
        pl -= Pos(0, sy_ * 60.0, 0) * Cylinder(6.0, t + 2, align=CTR)
    plate = put(parts, Pos(P.RAIL_X0 + 60.0, 0, P.FORK_Z + P.PRESS_PLATE_Z) * pl,
        "press_plate", to=("press_bush_L", "press_bush_R"), how="BOLT",
        note="リニアブッシュ 2 個を板に締結")
    # ⚠ 「リニアブッシュ 2 個」と注記していた実体が無く、板の穴（φ12）と
    #   ガイド軸（φ10）が 1mm 空いたまま「SLIDE」と宣言していた。
    #   摺動を受け持つ部品を描かないと、板は軸に対して浮いている。
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        put(parts, L.mat(Pos(P.RAIL_X0 + 60.0, sy * 60.0,
                             P.FORK_Z + P.PRESS_PLATE_Z)
                         * (Cylinder(6.0, 26.0, align=CTR)
                            # ⚠ つばは**プレートの上面に載せる**。+13 では
                            #   板から 11mm 浮いたところにあり、ブッシュと
                            #   板の接触は穴の壁だけに頼っていた。
                            # ⚠ つばを φ20 にしていたので、**ボルトを通す肉が
                            #   無かった**（外径 φ12 のまわりに 4mm の輪しか
                            #   残らず、M3 の頭 φ5.5 がはみ出す）。「2 個を板に
                            #   締結」と宣言して実体が無かったのはそのため。
                            #   実在の**フランジ付き** LMF10UU は つば φ29・
                            #   取付穴 PCD21 なので、そちらの寸法で描く。
                            + Pos(0, 0, t / 2 + 2.0)
                            * Cylinder(14.5, 4.0, align=CTR)
                            - Cylinder(5.0, 32.0, align=CTR)), "STEEL"),
            f"press_bush_{sd}", to=f"press_guide_{sd}", how="SLIDE",
            note="リニアブッシュ LMF10UU（フランジ付き）")
        LEDGER.add("リニアブッシュ φ10 フランジ付き", 0.028, "カタログ", "装填")
        # つば → プレート（t2 なのでタップは立たない。貫通させてナット）
        for i_, sxb in enumerate((1, -1)):
            face_screws(parts, [Pos(P.RAIL_X0 + 60.0 + sxb * 10.5, sy * 60.0,
                                    P.FORK_Z + P.PRESS_PLATE_Z + t / 2 + 4.0)],
                        f"press_bush_{sd}", "press_plate", 3, 0.0,
                        f"press_bush_{sd}_{i_}", link="press", how_b="THRU",
                        length=10.0, extras=(("HEXNUT", 3), ("WASHER", 3)))
    # ガイド軸（|Y|=60、φ10）が通るのでパッドに逃げ穴を開ける
    pad = L.plate(x - 20.0, y - 20.0, P.PRESS_PAD_T)
    for sy in (1, -1):
        # ⚠ 逃げ穴は **φ32**。φ16 だと、リニアブッシュのつばを留める M3
        #   （PCD21 ＝ 半径 10.5）の先が板を抜けてパッドへ 28mm³ ×4 入る。
        #   スポンジなので大きく抜いても押さえの面積は足りる。
        pad -= Pos(0, sy * 60.0, 0) * Cylinder(16.0, 12.0, align=CTR)
    # ⚠ パッドの Z は**プレートの下面から**引く。板厚 PRESS_PLATE[2] を
    #   4→2 に変えたとき、ここが数値べた書きだったせいで 1mm 潜り、
    #   45,702mm³ 食い込んだ。板厚を変えても追随するよう式で書く。
    pad_t = P.PRESS_PAD_T
    pad_z = P.FORK_Z + P.PRESS_PLATE_Z - t / 2 - pad_t / 2
    put(parts, Pos(P.RAIL_X0 + 60.0, 0, pad_z) * L.mat(pad, "SPONGE"),
        "press_pad", to="press_plate", how="CLAMP", note="スポンジを両面テープ貼り")
    LEDGER.add_solid(f"上押さえプレート A5052 t{t:.0f}", plate, "A5052", "装填")
    LEDGER.add("押さえスポンジパッド", 0.040, "概算", "装填")
    auto_screws(parts, "press")
    return Compound(label="grabber_press", children=parts)


# ===========================================================================
# ポーズ適用（STEP 用の静的配置）
# ===========================================================================
def _loc(origin, rot=(0, 0, 0)):
    return Location(origin, rot)


# 姿勢によらない部分のキャッシュ。
#   実測: build 1回 9.2 秒のうち **base_link と車輪で 8割** を占めていた。
#   姿勢を変えても base_link（258ソリッド）と車輪は形が変わらないのに、
#   毎回1から作り直していた。4姿勢を回す validate.py で 36.7 秒、
#   21姿勢の sweep_envelope.py では 193 秒がこれに消えていた。
#   → 一度作って使い回す。質量台帳ぶんは _STATIC_LEDGER に取っておいて毎回積み直す。
_STATIC = None
_STATIC_LEDGER: list = []


_STATIC_FIX: dict | None = None


def _static_parts():
    """姿勢に依存しない部分（base_link と車輪）。初回だけ作ってキャッシュする。

    ⚠ 形をキャッシュするなら**固定宣言も一緒に**キャッシュすること。
      build() は毎回 F.reset() するので、2 回目以降は車体側の宣言が
      1 つも登録されず、固定の連鎖が切れて全 423 部品が「浮き」に
      なっていた。掃引検査（15 姿勢）の 2 姿勢目以降がまるごと
      無意味になっていて、しかも数字は出るので気づきにくい。
    """
    global _STATIC, _STATIC_LEDGER, _STATIC_FIX
    if _STATIC is not None:
        LEDGER.items.extend(_STATIC_LEDGER)
        F.restore(_STATIC_FIX)
        return list(_STATIC)
    mark = len(LEDGER.items)
    parts = [link_base()]
    for name in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        j = JOINTS[name]
        hand = 1 if name in ("wheel_fl", "wheel_rr") else -1
        sy = 1 if name.endswith("l") else -1
        parts.append(label_shape(Pos(*j["origin"]) * link_wheel(hand, name[6:], sy), name))
    LEDGER.add("メカナムホイール φ127", P.WHEEL_MASS, "カタログ", "駆動", qty=4)
    _STATIC = list(parts)
    _STATIC_LEDGER = list(LEDGER.items[mark:])
    _STATIC_FIX = F.snapshot()
    return list(parts)


def build(pose: dict | None = None):
    """指定ポーズの TR 全体（ラベル付き Compound）を返す。"""
    pose = dict(P.POSE_MATCH if pose is None else pose)
    LEDGER.items.clear()
    F.reset()
    # ⚠ 押出材の印は「作った直後の put」で消費する。組立の外で `L.ext2020()` を
    #   呼ぶ検査（`scripts/vendor_check.py`）があるので、印が立ったまま
    #   組立に入ると**最初の put が無関係な名前で例外になる**。ここで落とす。
    L.LAST_EXT = False
    _declare_touches()

    yaw = pose.get("yaw", 0.0)
    pitch = pose.get("pitch", P.PITCH_DEFAULT)
    grab = pose.get("grab", 0.0)
    press = pose.get("press", 0.0)
    tilt = pose.get("tilt", 0.0)

    # 砲塔
    jy = JOINTS["turret_yaw"]
    yaw_loc = Pos(*jy["origin"]) * Rot(0, 0, yaw)
    jp = JOINTS["shooter_pitch"]
    pitch_loc = Pos(*jp["origin"]) * Rot(0, -pitch, 0)
    jf0 = JOINTS["fork_tilt"]
    # ⚠ **リンクの変換は link_*() を呼ぶ前に記録する。** 自動配置のねじは
    #   リンクローカルで凍結してあるので、位置計算（screw_place.py）の側が
    #   「そのねじがどのリンクに属するか」を世界座標から逆算するのに要る。
    LINK_LOC.clear()
    LINK_LOC.update({
        "base": Location(), "rails": Location(),
        "singulator": Pos(*JOINTS["singulator"]["origin"]),
        "turret": yaw_loc, "pitch": yaw_loc * pitch_loc,
        "carriage": Pos(-grab, 0, 0),
        "press": Pos(-grab, 0, 0) * Pos(0, 0, -press),
        "fork": Pos(-grab, 0, 0) * Pos(*jf0["origin"]) * Rot(0, tilt, 0),
        **{f"wheel_{w}": Pos(*JOINTS[f"wheel_{w}"]["origin"])
           for w in ("fl", "fr", "rl", "rr")},
    })

    children = _static_parts()
    pitch_children = [link_pitch()]
    for name in ("roller_upper", "roller_lower"):
        jr = JOINTS[name]
        pitch_children.append(label_shape(
            Pos(*jr["origin"]) * link_roller("u" if name.endswith("upper") else "d"), name))
    pitch_group = pitch_loc * Compound(label="pitch_group", children=pitch_children)
    yaw_group = yaw_loc * Compound(label="turret", children=[link_turret_yaw(), pitch_group])
    children.append(label_shape(yaw_group, "turret"))

    # シンギュレータ
    js = JOINTS["singulator"]
    children.append(label_shape(Pos(*js["origin"]) * link_singulator(), "singulator"))

    # グラバー（可動）
    slide = Pos(-grab, 0, 0)
    carriage = link_carriage()
    press_grp = Pos(0, 0, -press) * link_press()
    jf = JOINTS["fork_tilt"]
    fork_grp = Pos(*jf["origin"]) * Rot(0, tilt, 0) * link_fork()
    grab_group = slide * Compound(label="grabber",
                                  children=[carriage, press_grp, label_shape(fork_grp, "fork")])
    children.append(label_shape(grab_group, "grabber"))
    # 可動レール（インナー/中間）の繰り出し表現
    # スライドレールは**段ごとに別部品**にする。アウターは車体に固定、
    # 中間とインナーは動く。1 つに融合すると「上桁に留めるねじが可動段にも
    # 留まっている」という宣言になり、機構として成立しない。
    STAGE = ("out", "mid", "in")
    rails_fixed, rails_moving = [], []
    for sy in (1, -1):
        sd = "L" if sy > 0 else "R"
        # レール取付プレート。上桁の内側面から下へ伸ばして締結面を作る
        # ⚠ 上端は上桁（Z 818..838）に**届かせつつ**、台座横梁とその
        #   ブラケット（X 75..112）を避ける。板を短くすると今度は桁に
        #   留められなくなるので、**長さは保ったまま該当部を切り欠く**。
        # ⚠ 上桁 topbeam0（X -430..-285.4）に留めると宣言するなら、板が
        #   そこまで届いていること。RAIL_X0（-275.4）始まりでは 10mm 足りない。
        #   端で突き合わせるだけでは座面が 0mm²。桁と**重ねて**留める。
        pl_x0 = -350.0
        pl_len = 114.6 - pl_x0
        pl_top = P.BEAM_TOP_Z
        pl_h = pl_top - (P.RAIL_Z - P.RAIL_H / 2)
        # ⚠ 切り欠きの位置は**板の原点基準**。板の始点を RAIL_X0 から
        #   pl_x0 へ動かしたとき、ここが RAIL_X0 のままだったので
        #   台座横梁（X 75..95）とブラケットに 1,332mm³ 食い込んだ。
        notch = ((104.0 - pl_x0) - pl_len / 2, pl_h / 2 - 11.0, 76.0, 24.0)
        # ⚠ **φ22 を 14 個一列に並べるのをやめた（2026-08-07）。** 465mm の
        #   帯に丸が 14 個並ぶのは、上桁の内側で真横から見える位置なのに
        #   いちばん既製品っぽく見えていた。長穴（54×24）にすると
        #   1 個 1,150mm² で、丸（380mm²）の 3 倍抜ける。
        #   切り欠き（台座横梁の逃げ）の縁にも `edge` を効かせるため、
        #   切り欠きは**あとから引かず** `cuts` として輪郭に渡す。
        pl_shape, _rep = L.plate_slots(
            pl_len, pl_h, P.RAIL_PLATE_T, length=54.0, width=24.0, rib=11.0,
            edge=9.0, along="x", cuts=[notch], label=f"rail_plate_{sd}")
        rp_ = put(rails_fixed,
                  Pos((pl_x0 + 114.6) / 2, sy * P.RAIL_PLATE_Y,
                      (P.RAIL_Z - P.RAIL_H / 2 + pl_top) / 2)
                  * Rot(90, 0, 0) * pl_shape,
                  f"rail_plate_{sd}", to=(f"topbeam0_{sd}", f"topbeam1_{sd}"),
                  how="TSLOT", note="上桁の内側面へ 6-M5")
        # ⚠ 手書きの 0.290kg をやめて実体から取る（2026-08-07）。丸穴のころ
        #   から 66g ずれていて、肉抜きを変えるたびに台帳だけ古くなる。
        LEDGER.add_solid("レール取付プレート A5052 t4（肉抜き）", rp_, "A5052", "装填")
        base = Pos(P.RAIL_X0, sy * P.RAIL_MOUNT_Y, P.RAIL_Z)
        for i, st in enumerate(STAGE):
            nm = f"rail_{st}_{sd}"
            # ⚠ 段と段は**直接は触れない**。あいだにボール保持器が入る。
            #   隣の段に SLIDE と宣言すると、6.51mm の隙間が「離れ」になる。
            #   摺動を伝えているのは保持器なので、そこへ留める。
            # ⚠ 段と保持器は**同じ留め方**で宣言する。段からは SLIDE、
            #   保持器からは ROTATE と書いていたので、同じ組に 2 つの
            #   方法が宣言されて「どちらの基準で検査するか」が決まらなく
            #   なっていた。転がり接触なので両方 ROTATE。
            tgt = ((f"rail_plate_{sd}",) if i == 0
                   else (f"rail_ball_{sd}{i - 1}u", f"rail_ball_{sd}{i - 1}d"))
            how = "BOLT" if i == 0 else "ROTATE"
            put(rails_fixed if i == 0 else rails_moving,
                base * L.rail_stage(i, grab, hand=-sy), nm, to=tgt, how=how,
                note="アウターを取付プレートへ 6-M4" if i == 0 else "ボール保持器を介して摺動")
        for k in (0, 1):
            for sz in (1, -1):
                put(rails_moving, base * L.rail_retainer(k, sz, grab, hand=-sy),
                    f"rail_ball_{sd}{k}{'u' if sz > 0 else 'd'}",
                    to=(f"rail_{STAGE[k]}_{sd}", f"rail_{STAGE[k + 1]}_{sd}"),
                    how="ROTATE", note="鋼球 + 保持器")
    auto_screws(rails_fixed, "rails")
    children.append(Compound(label="slide_rails_fixed", children=rails_fixed))
    children.append(Compound(label="slide_rails", children=rails_moving))

    # サドル・ブラケットを実体で描いたので、概算からその分を外す
    LEDGER.add("配線・コネクタ・結束", 0.700, "概算", "電装")
    # 締結具は**数えて積む**。
    # ⚠ ここは長らく「ボルト・ナット・スペーサ類 0.700kg 概算」の 1 行だった。
    #   実際に数えると、骨格の L 金具だけで M5 が 92 本＋後入れナット 92 個
    #   ある。概算の中に隠れているあいだは、誰も「多すぎる／少なすぎる」を
    #   判断できない。呼び径と長さが決まれば質量は形から出るので、
    #   **員数表と同じ数え方**（tr_lib.fastener_ledger_rows）で積む。
    #   内訳は scripts/fastener_bom.py が out/fasteners_bom.md に書き出す。
    for label, qty, unit in L.fastener_ledger_rows():
        LEDGER.add(label, unit, "規格外形からの体積 × 7.93g/cm³", "締結具",
                   qty=qty)
    # 装飾はプラダン＋カッティングシートに（規定 35kg の余裕が薄いため）
    LEDGER.add("装飾（学校・地域モチーフ）", 0.250, "概算", "その他")

    return Compound(label="tr_robot", children=children)


def gen_step():
    return build(P.POSE_MATCH)

# ---------------------------------------------------------------------------
# 組み立て結果の自己検査
# ---------------------------------------------------------------------------
_BUILD_SOLIDS: int | None = None
_build_unguarded = build


def build(pose: dict | None = None):  # noqa: F811
    """`build()` のソリッド数が、同じプロセス内で変わらないか見張る。

    ⚠ **既にあるソリッドを新しい Compound の children に渡すと親が
      付け替わり、元の木が壊れる。** `_STATIC` は車体側の部品をプロセス内で
      使い回すので、1 度でも付け替えると次の姿勢の build が**黙って**部品を
      失う。実際 STEP を 3 姿勢まとめて出したとき、2 姿勢目と 3 姿勢目が
      14〜15 ソリッド少ないファイルになっていた（463 → 449 / 448）。
      数を数えていなかったので気づけず、欠けた STEP を渡しかけた。
      姿勢を変えてもソリッド数は変わらないはず（scripts/assembly_check.py
      --pose swept の全姿勢で確認済み）なので、
      変わったら止める。ブーリアンが空を返して部品が消える類も同時に捕まる。
    """
    # ⚠ 数え方は**検査が使うのと同じ経路**にする。`.solids()` は木が
    #   壊れても同じ数を返すことがあり、それでは見張りにならない
    #   （実際 `.solids()` で見張った版は再現テストで検出できなかった）。
    #   検査は validate.solids_with_bbox() を使うので、そちらで数える。
    global _BUILD_SOLIDS
    shape = _build_unguarded(pose)
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__), "..", "scripts"))
    import validate as _V
    n = len(_V.solids_with_bbox(shape))
    if _BUILD_SOLIDS is None:
        _BUILD_SOLIDS = n
    elif n != _BUILD_SOLIDS:
        raise ValueError(
            f"ソリッド数が {_BUILD_SOLIDS} → {n} に変わった。既存のソリッドを"
            f"別の Compound に渡して木を壊していないか"
            f"（Compound(children=[...]) は親を付け替える）")
    return shape
