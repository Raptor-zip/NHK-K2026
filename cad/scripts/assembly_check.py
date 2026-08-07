"""組立の厳格チェック — **宣言された固定関係と実体を突き合わせる**.

    python scripts/assembly_check.py [--md] [--pose match|stowed|loading|all]

判定は距離のしきい値ではなく、`tr_fix.py` の**固定宣言**を基準にする。

    A. 固定先が無い / 基準からたどり着けない部品   → 浮き
    B. 宣言した固定先と実体が接していない          → 図と宣言の食い違い
    C. **宣言していない部品どうしが接触している**   → 意図しない干渉
    D. 宣言はあるが実体が食い込んでいる            → 締結方法に許されない重なり

C が従来の方式との決定的な違い。以前は「離れているべき組」を人が列挙し、
書き忘れた組は既定値 FREE（3mm 離れていれば OK）で素通りしていた。
**離れていれば通る基準は、空中に浮いた部品ほど通りやすい。**
ここでは接触しているほうを宣言させるので、書いていない接触はすべて NG になる。
"""

from __future__ import annotations

import argparse
import math
import re
import itertools
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from build123d import SkipClean  # noqa: E402
import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402

POSES = {"match": lambda: P.POSE_MATCH,
         "stowed": lambda: P.POSE_STOWED,
         "loading": lambda: P.POSE_LOADING}

# 可動範囲を振った姿勢。**1 姿勢だけ見ても足りない。**
# 砲塔は ±30° 旋回して仰角 20..60° を取り、グラバーは 425mm 走る。
# 描いた姿勢で離れていても、途中で当たるなら実機では壊れる。
def _swept():
    """可動範囲を振った姿勢。

    ⚠ **端の 3 点では足りない。** ヨー ±30 / 仰角 20・45・70 の 9 点だけを
      見ていたとき、台座横梁と仰角側板が「ヨーごとに動く窓」で当たる
      ことを見逃していた。実際には y+10 p50 で 173mm³、y+20 p25 で
      265mm³ 当たっていて、どちらも 9 点の格子から外れている。
      干渉が (ヨー, 仰角) の**領域**として現れる以上、格子を細かく
      刻まないと領域の内側を通り抜けてしまう。
    """
    out = {}
    yaws = [-P.YAW_LIMIT, -P.YAW_LIMIT * 2 / 3, -P.YAW_LIMIT / 3, 0.0,
            P.YAW_LIMIT / 3, P.YAW_LIMIT * 2 / 3, P.YAW_LIMIT]
    n_p = 5
    pitches = [P.PITCH_MIN + (P.PITCH_MAX - P.PITCH_MIN) * k / (n_p - 1)
               for k in range(n_p)]
    for yaw in yaws:
        for pitch in pitches:
            out[f"y{yaw:+.0f}p{pitch:.0f}"] = dict(P.POSE_MATCH, yaw=yaw, pitch=pitch)
    for grab in (0.0, P.GRAB_STROKE / 3, P.GRAB_STROKE * 2 / 3, P.GRAB_STROKE):
        for press in (0.0, P.PRESS_STROKE):
            out[f"g{grab:.0f}s{press:.0f}"] = dict(P.POSE_MATCH, grab=grab, press=press)
    return out


SWEPT = _swept()
ALL_POSES = {**POSES, **{k: (lambda v=v: v) for k, v in SWEPT.items()}}

# リンクをまたいで許される固定方法。これ以外は**関節が無いのに
# 別リンクの部品へ留めている**ことになり、機構として成立しない。
CROSS_LINK_OK = {"SLIDE", "ROTATE", "MESH", "ROUTE", "CONNECT", "BUNDLE",
                 "SIT", "CONTAIN", "CLAMP", "SHAFT", "THRU", "TNUT"}


# 部品名から所属リンクを決め打ちする例外。
# スライドレールは 3 段が別々の速度で動くので、描画上のグループ
# （slide_rails）とリンクは一致しない。アウターは車体、インナーは
# キャリッジと同じ速度、中間はその半分。
NAME_LINK = {"rail_out_": "base_link", "rail_in_": "grabber_slide",
             "rail_mid_": "rail_mid", "rail_ball_": "rail_mid"}


def link_of(path: str) -> str:
    """ソリッドのパスから所属リンクを取り出す。"""
    leaf = path.split("/")[-1].split("#")[0]
    for pre, lk in NAME_LINK.items():
        if leaf.startswith(pre):
            return lk
    parts = path.split("/")
    for key in ("shooter_pitch", "roller_upper", "roller_lower", "turret_yaw",
                "grabber_press", "grabber_slide", "fork_tilt", "fork", "singulator",
                "slide_rails", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        if key in parts:
            return key
    return "base_link"

# **置いてあるだけ**でよい部品と、その理由。
# ⚠ ここに書いていない部品が SIT / CONTAIN / TOUCH_OK だけで留まっていたら
#   不良として出す。理由を書かせるのは「気付かないうちに増えた」を防ぐため。
REST_OK = {
    # ⚠ **`bucket` はここから外した（2026-08-07）。** 「競技で審判が置く」と
    #   書いて SIT（載せるだけ）に逃がしていたが、置くのは審判ではなく
    #   チームで（規定 4.1.1「移動バケツはロボットを運び込む前までに設置を
    #   完了させること」）、規定 3.2.3a は「競技中は常に水平を維持できる
    #   ようにしっかりと固定すること」を求めている。いまは底面 4-M5 で
    #   受け板に留まっているので、この逃げ道は要らない。
    "mascot_suit": "規定 3.1.3 のマスコット（計量前に工具無しで外す）",
    "bucket_2l_datum": "規定 3.2.3c の 2L 目盛り位置（実体ではない）",
    "chair": "規定で載せる椅子（固定はベルトで、形は代表寸法）",
}

# 宣言の陰に隠れる「許容内の重なり」の合計の上限 [mm³]。
# ⚠ 2026-07-30 時点の内訳: 99 組 9,717mm³ のうち 92 組 8,735mm³ が
#   SCREW_IN / THRU / TNUT ＝ **ねじがタップ穴に噛む体積そのもの**
#   （1 組 ~95mm³。ねじを呼び径の円柱・穴を下穴径で描けば必ずこうなる）。
#   残りは BUNDLE（束ねた配線）と CONNECT（コネクタ）。いずれも実体として
#   正しいので、これ以上は減らせない。
#   ここを超えたら「ベルトを平板で描いた」「プーリの内径が軸径と違う」
#   といった**描き間違いが新しく入った**と見なす。
# ⚠ 2026-07-31: ねじ 1,544 個の実体化と機上計算機・表示器の追加で
#   9,717 → 12,056mm³ に増えた。増分はねじがタップ穴に噛む体積と
#   コネクタの差し込みで、いずれも実体として正しい。14,000 に上げる。
ALLOWED_BUDGET = 14_000.0
# ねじ 1 本あたりの上乗せ [mm³]。⚠ ねじが下穴に噛む体積は**本数に比例する**
#   ので、固定値だけで見ていると実体化した瞬間に超えて、そのまま赤が
#   居座る（2026-08-02 時点で 791 組 57,732mm³ / ねじ 485 本 ＝ 1 本
#   あたり 119mm³）。M5×12 が下穴に 6mm 噛めば 118mm³ で、これが標準。
ALLOWED_PER_SCREW = 130.0

# 接触とみなす最大すきま [mm]。組立公差と 3D プリントの寸法ばらつきを見込む
CONTACT_TOL = 0.5
# ⚠ しきい値ちょうどで接している設計は**姿勢や板厚のわずかな変更で
#   判定が反転する**。実際、側板を t4→t3 にして内面が 0.5mm 動いたとき、
#   ヨー 0 では通り ±30° では「離れ」5 件になる、という再現性の悪い
#   症状が出た。余裕がこれ未満の接触は別枠で警告する。
MARGIN_WARN = 0.25
# ブーリアンを走らせる bbox 重なりの下限 [mm³]（bbox の重なりは実体の重なりの上限）
BOOL_MIN = 20.0

# **意図して複数片にしている**部品と、その片数。
# ここに書いたものだけ分断検査から外れる。数まで書かせるのは、
# 「気付かないうちに更に割れた」を見逃さないため。
SPLIT_OK = {
    # 台座リングプレートは横梁の真上だけを残した 2 枚の円弧。
    # 給送（600mm 幅の布）が -X 側を通るので、閉じたリングにはできない。
}


def bbox_overlap(ba, bb):
    v = 1.0
    for lo_a, hi_a, lo_b, hi_b in (
            (ba.min.X, ba.max.X, bb.min.X, bb.max.X),
            (ba.min.Y, ba.max.Y, bb.min.Y, bb.max.Y),
            (ba.min.Z, ba.max.Z, bb.min.Z, bb.max.Z)):
        v *= max(0.0, min(hi_a, hi_b) - max(lo_a, lo_b))
    return v


def bbox_gap(ba, bb):
    dx = max(0.0, ba.min.X - bb.max.X, bb.min.X - ba.max.X)
    dy = max(0.0, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y)
    dz = max(0.0, ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def screw_allow(na: str, nb: str) -> float:
    """組の一方がねじなら、その軸が材の中に入りうる体積 [mm³]。

    ねじ 1 本まるごとぶんを上限にする。これを超えるのは「ねじが 2 つの材に
    完全に埋まっている」ことで、そのときは長さか位置が間違っている。
    """
    import tr_lib as _L
    out = 0.0
    for nm in (na, nb):
        f = F.FASTENER.get(nm)
        if not f:
            continue
        kind, size, length, ex = f
        # ⚠ **頭と付属品も勘定に入れる。** 軸だけで見ていたので、溝ナット
        #   付き（φ11 × 3.4）のねじが正しく通っていても 515mm³ > 406 で
        #   食い込みになった。員数表と同じ関数から体積を出す。
        g = _L.fastener_mass(kind, size, length) * 1000.0
        for ek, es in ex:
            g += _L.fastener_mass(ek, es, 0.0) * 1000.0
        out = max(out, g / _L.RHO_FASTENER * 1.15)
    return out


def overlap_volume(sa, sb):
    """実体の重なり体積 [mm³]。`&` は交差が空なら None を返す。

    ⚠ **`SkipClean` で囲む。** build123d はブーリアンのたびに
      `ShapeUpgrade_UnifySameDomain`（同一平面の面を融合して形を整える）を
      走らせる。作った形をそのまま STEP に出すなら要るが、**ここは体積を
      測って捨てるだけ**なので、整えた分はまるごと無駄になる。
      実測 1.59 倍（51.21 → 32.14 秒 / 16 回）、体積は 1e-9 も動かない。
      この関数は総当たりの内側（組立検査・連続掃引・回転全角度・
      ビューア内訳・衝突被覆）から呼ばれるので、そのまま効く。
    """
    with SkipClean():
        r = sa & sb
        return 0.0 if r is None else r.volume


def _screw_ab() -> dict[str, tuple[str, str]]:
    """自動配置したねじ名 → (留める部品 a, 留める部品 b)。"""
    import json as _json
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "out", "screws.json")
    if not os.path.exists(p):
        return {}
    with open(p, encoding="utf-8") as fp:
        d = _json.load(fp)
    items = d if isinstance(d, list) else d.get("screws", [])
    return {s["name"]: (s.get("a", ""), s.get("b", "")) for s in items
            if s.get("name")}


_SCREW_AB: dict[str, tuple[str, str]] | None = None


def _screw_neighbour(na: str, nb: str) -> bool:
    """「ねじが、留めている相手の**すぐ隣の部品**に当たっている」か。

    ⚠ 自動配置のねじは座面の中心に立つので、**皿の頭（M4 で φ8）が
      隣の板の面に接する**ことがある。実測 `scr_a0053`（`car_brk_L` ↔
      `car_brk_Lv` を留める M4 皿、Y 307.9）は頭が Y 311.9 まで届き、
      側板の内面（311.9）にちょうど接した。
    ⚠ **これは不良ではない。** `car_brk_Lv` は側板へ 3-M4 で留まっている
      ので、ねじと側板は同じ剛体の中で隣り合っているだけ。宣言を書く
      相手でもない（ねじ名は配置のたびに変わるので書けない）。
    ⚠ **1 ホップに限る。** 固定の推移閉包まで許すと機体のほぼ全部が
      「隣」になり、遠くの部品を突いているねじを見逃す。
    """
    global _SCREW_AB
    if _SCREW_AB is None:
        _SCREW_AB = _screw_ab()
    for scr, other in ((na, nb), (nb, na)):
        ab = _SCREW_AB.get(scr)
        if ab is None:
            continue
        if other in ab:
            return True
        for side in ab:
            if not side:
                continue
            if other in F.targets_of(side) or side in F.targets_of(other):
                return True
    return False


def part_name(path: str) -> str:
    """ソリッドのパスから部品名（固定宣言のキー）を取り出す。"""
    return path.split("/")[-1].split("#")[0]


def analyze(pose_name):
    shape = A.build(ALL_POSES[pose_name]())
    sol = V.solids_with_bbox(shape)
    # 1 部品が複数ソリッドになる場合（C 形材など）はまとめて扱う
    names = [part_name(n) for n, _s, _b in sol]
    known = set(names)

    bad = {"unresolved": F.unresolved(known), "orphan": [], "detached": [],
           "undeclared": [], "dug": [], "crosslink": [], "split": [], "seat": [],
           "margin": [], "single": [], "deep": [], "tap": [], "tool": [],
           "bend": [], "conflict": [], "cycle": [], "rest": [], "brk": [],
           "allowed_over": [],
           "cable_end": [], "cable_slack": [], "cable_loop": [], "selflap": []}

    # --- A1a. **置いてあるだけ**の部品 ---------------------------------
    # ⚠ SIT / CONTAIN / TOUCH_OK しか宣言が無い部品は、位置を決めるものが
    #   何も無い。実機では振動で動く。ベルト（MESH）・軸（ROTATE）・
    #   配線（ROUTE/CONNECT/BUNDLE）は対偶や経路が位置を決めているので
    #   これには当たらない。**意図して置いてあるものだけ**を許す。
    for nm_ in sorted(known):
        lst_ = F.FIXINGS.get(nm_, [])
        if not lst_ or nm_ in REST_OK:
            continue
        if all(h_ in ("SIT", "CONTAIN", "TOUCH_OK") for _t, h_, _q, _n in lst_):
            bad["rest"].append((nm_, [h_ for _t, h_, _q, _n in lst_]))

    # --- A1b. 宣言そのものの矛盾 ---------------------------------------
    # ⚠ 同じ組に**別の留め方**を 2 つ書いていたら、どちらが正しいのか
    #   決まらない。A→B を BOLT、B→A を SLIDE と書けば、検査は片方の
    #   基準（許容 0 と接触必須）でしか見ないので、もう片方の意図は
    #   黙って捨てられる。
    # ⚠ **循環**（A→B→C→A）も弾く。組立順序が決まらないので、実機では
    #   どこかを最後にこじ入れることになる。
    seen_pair: dict[tuple, str] = {}
    for name, lst in F.FIXINGS.items():
        for t, how, _q, _n in lst:
            key = tuple(sorted((name, t)))
            prev = seen_pair.get(key)
            if prev is not None and prev != how:
                bad["conflict"].append((name, t, prev, how))
            else:
                seen_pair[key] = how
    # 有向グラフ（部品 → 固定先）で循環を探す
    adj_d = {n: [t for t, _h, _q, _nn in l] for n, l in F.FIXINGS.items()}
    state: dict[str, int] = {}

    def _walk(n, path):
        if state.get(n) == 2:
            return
        if state.get(n) == 1:
            i = path.index(n)
            cyc = tuple(path[i:] + [n])
            # ⚠ **往復（A→B→A）は循環ではない。** 相互に留まることを
            #   表しているだけで、組立順序の問題にはならない（同時に組む）。
            #   3 部品以上を回る循環だけが「どこかを最後にこじ入れる」
            #   ことになる。
            if len(cyc) > 3:
                bad["cycle"].append(cyc)
            return
        state[n] = 1
        for nx in adj_d.get(n, ()):
            _walk(nx, path + [n])
        state[n] = 2

    for n0 in list(adj_d):
        if state.get(n0) is None:
            _walk(n0, [])

    # --- A2. 締結の段数（公差の積み上がり）----------------------------
    # ⚠ 段が深い部品は、1 段ごとに位置決め誤差が足されるので実機で
    #   位置が出ない。0.1mm の公差が 12 段積めば 1.2mm になる。
    #   締結を宣言できていても、段数を見ないと「組めるが精度が出ない」
    #   設計になる。
    #   ただし多段スライド（3 段引きレール）とヒンジを経る機構では
    #   段数が深くなるのは必然。実機では治具や基準面で直接位置を出す。
    #   ここで見たいのは「想定より深い経路が混ざっていないか」なので、
    #   必然の 14 段（射出ローラー・上押さえパッド）は通す。
    DEPTH_WARN = 15
    dep = F.depth_of(known)
    link_by_part: dict[str, str] = {}
    for n_, _s, _b in sol:
        link_by_part.setdefault(part_name(n_), link_of(n_))
    for nm_, d_ in dep.items():
        if d_ >= DEPTH_WARN:
            bad["deep"].append((d_, nm_))
    if dep:
        vs = sorted(dep.values())
        bad["depth_stat"] = (vs[-1], vs[len(vs) // 2], len(vs))

    # --- 1 つの部品が分断されていないか -------------------------------
    # ⚠ 穴やテーパのカットが深すぎると、**1 部品が離ればなれのソリッドになる**。
    #   台座リングプレートが肉抜き穴で 8 個の円弧に切れ、櫛歯の歯先が
    #   傾斜カットで 3.1mm 離れた別体になっていた。どちらも図では
    #   「そこにある」ように見えるので、体積や外形の検査では出てこない。
    by_part: dict[str, list] = {}
    for path, so, bx in sol:
        by_part.setdefault(part_name(path), []).append((so, bx))
    for nm, lst in by_part.items():
        if len(lst) < 2:
            continue
        n_ = len(lst)
        par = list(range(n_))

        def _find(a, par=par):
            while par[a] != a:
                par[a] = par[par[a]]
                a = par[a]
            return a

        for i, j in itertools.combinations(range(n_), 2):
            (si, bi), (sj, bj) = lst[i], lst[j]
            if bbox_gap(bi, bj) > CONTACT_TOL:
                continue
            try:
                if si.distance_to(sj) <= CONTACT_TOL:
                    ri, rj = _find(i), _find(j)
                    if ri != rj:
                        par[rj] = ri
            except Exception:
                pass
        groups = len({_find(i) for i in range(n_)})
        if groups > 1 and groups != SPLIT_OK.get(nm):
            bad["split"].append((nm, n_, groups))

    # --- 分断の裏返し: 同じ部品名の中でソリッドが重なっていないか ------
    # ⚠ **これまでの盲点。** 干渉検査は「部品 A のソリッド」と「部品 B の
    #   ソリッド」の組しか見ておらず、**同じ部品名に属するソリッドどうし**は
    #   一度も比べていなかった。463 ソリッド / 423 部品なので 40 ソリッドが
    #   複数ソリッドの部品に属する。C 形材や L 字金具のように意図して
    #   複数塊で作った部品の**内部**で重なっていても、図では 1 部品なので
    #   気づけない。分断（離ればなれ）は見ていたのに、その逆を見ていなかった。
    for nm_, lst_ in by_part.items():
        if len(lst_) < 2:
            continue
        for i_ in range(len(lst_)):
            for j_ in range(i_ + 1, len(lst_)):
                sa_, ba_ = lst_[i_]
                sb_, bb_ = lst_[j_]
                if bbox_overlap(ba_, bb_) < BOOL_MIN:
                    continue
                v_ = overlap_volume(sa_, sb_)
                if v_ > BOOL_MIN:
                    bad["selflap"].append((v_, nm_, i_, j_))

    # --- リンクをまたぐ固定の検査 ---
    link = {}
    for path, _s, _b in sol:
        link[part_name(path)] = link_of(path)
    for name, lst in F.FIXINGS.items():
        if name not in link:
            continue
        for t, how, _q, _n in lst:
            if t not in link or how in CROSS_LINK_OK:
                continue
            if link[name] != link[t]:
                bad["crosslink"].append((name, link[name], t, link[t], how))

    # --- A. 基準からたどり着けない部品 ---
    reach = F.reachable(known)
    bad["orphan"] = sorted(known - reach)

    # 許容の内側で重なっている組の [件数, 合計体積] と、大きい順の明細
    allowed = [0, 0.0]
    allowed_top: list = []

    # --- B/C/D. 実体どうしの関係 ---
    contact = set()          # 実際に接触している組
    for i, j in itertools.combinations(range(len(sol)), 2):
        na, nb = names[i], names[j]
        if na == nb:
            continue         # 同じ部品の別ソリッド
        _pa, sa, ba = sol[i]
        _pb, sb, bb = sol[j]
        dec = F.declared(na, nb)
        if bbox_overlap(ba, bb) > BOOL_MIN:
            v = overlap_volume(sa, sb)
            if v > 1.0:
                contact.add((na, nb))
                ok = dec[1] if dec else 0.0
                # ⚠ **ねじが穴を通るぶんの重なりは、呼び径と長さで決まる。**
                #   固定の許容（THRU 400mm³）で見ていたので、M6×25 のような
                #   太くて長いねじは、正しく通っていても必ず超えた
                #   （椅子の取付金具で 485mm³ × 6 本）。ねじ山も下穴も
                #   描かない方針なので、軸が材の中にあるぶんは幾何そのもの。
                #   **宣言のある組にだけ**適用する（宣言なしで貫くのは異常）。
                if dec:
                    ok = max(ok, screw_allow(na, nb))
                if v > ok:
                    bad["dug"].append((v, na, nb, dec[0] if dec else "宣言なし", ok))
                elif ok > 0.0 and v > 20.0:
                    # ⚠ **許容の内側の重なりも数えて出す。**「食い込み 0」は
                    #   「どこも重なっていない」ではない。PRESS 4,000 /
                    #   SHAFT 4,000 / MESH 3,200 を許しているので、宣言の
                    #   陰で数万 mm³ が重なっていても検査は 0 と言う。
                    #   ビューアで見れば全部干渉に見えるので、隠さず出す。
                    #
                    # ■ ここを 53,448 → 9,718mm³ まで落とした経緯（2026-07-30）
                    #   出していなかったあいだに溜まっていたのは、ほとんどが
                    #   **形を描き間違えていた**もので、物理ではなかった:
                    #     ・ベルトを平板や角枠で描き、プーリの中心を貫通
                    #       （斜路 8,581 / 昇降 4,056 / 駆動 1,958 / 減速 1,958）
                    #     ・胴 12mm のプーリに幅 15mm のベルト（鍔が 2,984）
                    #     ・プーリの内径 8 に軸 φ10（1,808）
                    #     ・「軸に割締め」と書いて割締めの実体が無く、板が軸を
                    #       貫通（3,646）
                    #     ・「ベルトを挟む板 2 枚」が 1 本の柱で、ベルトを貫通
                    #       （1,350）
                    #   直すたびに**隠れていた設計の誤り**が出てきた（受け柱が
                    #   ベルトを跨げていない・押さえベルトの持ち上げ量が足りず
                    #   下側の直線部がガイド板の中に入る・側板が端部軸に届かない）。
                    #   形を実物どおりに描くこと自体が検査になっている。
                    #
                    # ■ 残る 9,718mm³ の性質
                    #   92 組 8,735mm³ が SCREW_IN / THRU / TNUT で、
                    #   **ねじがタップ穴に噛む体積そのもの**（1 組 ~95mm³）。
                    #   ねじを呼び径の円柱、穴を下穴径で描けば必ずこうなる。
                    #   残りは BUNDLE（束ねた配線どうし）と CONNECT
                    #   （コネクタが本体に挿さる）で、いずれも実体として正しい。
                    #   → **ここから先を 0 に近づけるにはねじ山を描くしかなく、
                    #     設計の妥当性には寄与しない。**
                    allowed[0] += 1
                    allowed[1] += v
                    allowed_top.append((v, na, nb, dec[0]))
                continue
            # bbox は重なるが実体は重ならない → 接触判定へ落とす
        elif bbox_gap(ba, bb) > CONTACT_TOL:
            continue
        try:
            d = sa.distance_to(sb)
        except Exception:
            continue
        if d <= CONTACT_TOL:
            contact.add((na, nb))
            if dec is None and not _screw_neighbour(na, nb):
                bad["undeclared"].append((d, na, nb))
            # ⚠ ここへは `dec is None` でも落ちてくる（宣言が無くても
            #   ねじの隣どうしなら上の if を素通りする）。`dec[0]` を
            #   そのまま引くと TypeError で検査ごと止まる。実際、計測輪を
            #   作り直して新しいねじの隣接が増えた回に落ちた。
            #   宣言が無い組に「余裕が公差以下」を言う意味は無いので外す。
            elif (dec is not None and d > MARGIN_WARN
                  and dec[0] != "TOUCH_OK" and F.HOW[dec[0]][0]):
                # 宣言どおり接してはいるが、余裕が公差以下。板厚や姿勢を
                # 少し変えるだけで「離れ」に反転する箇所。
                # TOUCH_OK は「接してもよい」だけで接触必須ではないので除く。
                bad["margin"].append((d, na, nb, dec[0]))

    allowed_top.sort(reverse=True)
    bad["allowed_stat"] = tuple(allowed)
    bad["allowed_top"] = allowed_top[:6]
    # ⚠ **合計に上限を置く。** 「食い込み 0」の陰で数万 mm³ が重なっていても
    #   検査は通ってしまう（実際 53,448mm³ 溜まっていた）。中身を直して
    #   9,717mm³ まで落としたので、そこから増えたら**新しい描き間違いが
    #   入った**と見なして落とす。上げるときは中身を確かめてから上げること。
    # ⚠ **ねじの本数に比例する枠にする。** 重なりの中身はほとんどが
    #   「ねじが下穴に噛む体積」で、これは本数に比例して増える。固定値の
    #   ままだと、ねじを実体にした時点（§37 で 345 本）で必ず超え、
    #   以後ずっと赤いままになって**本当に増えたときに気づけない**。
    #   1 本あたりの平均（実測 75mm³）が跳ね上がったら出るようにする。
    n_scr = sum(1 for nm in known if nm.startswith("scr"))
    budget = ALLOWED_BUDGET + ALLOWED_PER_SCREW * n_scr
    if allowed[1] > budget:
        bad["allowed_over"].append((allowed[1], budget))

    # --- B. 宣言したのに接していない ---
    for name, lst in F.FIXINGS.items():
        if name not in known:
            continue
        for t, how, _q, _n in lst:
            if t not in known:
                continue          # unresolved で別途報告
            if not F.HOW[how][0]:
                continue
            if (name, t) not in contact and (t, name) not in contact:
                bad["detached"].append((name, t, how))

    # --- D2. 1 点留めになっていないか ---------------------------------
    # ⚠ 締結先が 1 つしか無い部品は、実機では**その 1 点を軸に回る**。
    #   ボルト 1 本で留めた板は必ず回るので、2 点以上で拘束すること。
    #   軸への固定（PRESS / SHAFT / ROTATE / SLIDE）は軸自体が向きを
    #   決めるので 1 つでよい。小物（体積 3,000mm³ 未満）も除く。
    # ⚠ SEW（縫い付け・接着）は**面で**留まる。マスコットの頭や三角巾は
    #   縫い代の全周が付いているので、1 か所の宣言でも回りようがない。
    SPIN_FREE = {"PRESS", "SHAFT", "ROTATE", "SLIDE", "MESH", "CONTAIN",
                 "ROUTE", "CONNECT", "BUNDLE", "CLAMP", "SIT", "SEW"}
    #   本数は note の「4-M3」「2-M5」といった書き方からも読む
    #   （qty 引数を省いた呼び出しが多いため）。
    def _n_fast(note: str, qty: int) -> int:
        # 「4-M3」「2-φ3.2」「8-φ4」いずれの書き方も拾う
        m = re.findall(r"(\d+)\s*-\s*[MmφΦ]", note or "")
        return max([qty] + [int(x) for x in m]) if m else qty

    for name, lst in F.FIXINGS.items():
        if name not in known or len(lst) != 1:
            continue
        t, how, qty, note = lst[0]
        if how in SPIN_FREE or _n_fast(note, qty) >= 2:
            continue
        vol = sum(sa_.volume for sa_, _b in by_part.get(name, ()))
        if vol >= 3000.0:
            bad["single"].append((vol, name, t, how))

    # --- D3. ねじが相手に刺さるなら、そこにねじ山が立つ厚みがあるか ----
    # ⚠ SCREW_IN は「ねじ本体が相手に刺さる」宣言＝**相手にタップを切る**。
    #   ねじ山が 1.5d ぶん立たないと、締めた瞬間に舐める。
    #   M5 なら 7.5mm、M4 なら 6mm、M3 なら 4.5mm。
    #   板が薄いなら、貫通させてナットで留める（BOLT）か、
    #   ボスを立てるか、溝ナット（TSLOT）にすること。
    #   相手の厚みは bbox の最小辺で見る（板なら板厚、押出材なら断面）。
    TAP_MIN = {3: 4.5, 4: 6.0, 5: 7.5, 6: 9.0}
    for name, lst in F.FIXINGS.items():
        if name not in known or not name.startswith("scr"):
            continue
        m = re.search(r"M(\d)", "".join(str(x) for x in lst))
        size = int(m.group(1)) if m else None
        if size is None:
            # 名前から拾えないときはねじ自身の bbox の細い辺を径とみなす
            bs = [b for _s, b in by_part.get(name, ())]
            if not bs:
                continue
            e = sorted((bs[0].max.X - bs[0].min.X, bs[0].max.Y - bs[0].min.Y,
                        bs[0].max.Z - bs[0].min.Z))
            size = max(3, min(6, round(e[0] / 2)))
        need = TAP_MIN.get(size)
        if need is None:
            continue
        for t, how, _q, _n in lst:
            if how != "SCREW_IN" or t not in known:
                continue
            for _s, bb_ in by_part.get(t, ()):
                thick = min(bb_.max.X - bb_.min.X, bb_.max.Y - bb_.min.Y,
                            bb_.max.Z - bb_.min.Z)
                if thick < need:
                    bad["tap"].append((thick, need, name, t, size))
                break

    # --- D3b. 配線の曲げ半径が足りるか ---------------------------------
    # ⚠ ケーブルには**最小曲げ半径**がある（外径の 4 倍が目安）。折れ線で
    #   描いていると、節点で 90° 以上曲げても図の上では成立してしまう。
    #   実機では被覆が潰れて芯線が切れるか、反発して可動部へ押し出される。
    #   節点を半径 R で丸めるには、両隣の直線が R·tan(θ/2) 以上必要
    #   （θ = 折れ角）。足りなければ「曲げきれない」。
    for nm_, (pts, dia_) in F.CABLE.items():
        if nm_ not in known or len(pts) < 3:
            continue
        r_min = 4.0 * dia_
        for i in range(1, len(pts) - 1):
            a_, b_, c_ = pts[i - 1], pts[i], pts[i + 1]
            u = [b_[k] - a_[k] for k in range(3)]
            v = [c_[k] - b_[k] for k in range(3)]
            lu = sum(x * x for x in u) ** 0.5
            lv = sum(x * x for x in v) ** 0.5
            if lu < 1e-6 or lv < 1e-6:
                continue
            cosang = sum(u[k] * v[k] for k in range(3)) / (lu * lv)
            cosang = max(-1.0, min(1.0, cosang))
            theta = math.acos(cosang)          # 折れ角（0 = 直進）
            if theta < math.radians(8.0):
                continue
            need = r_min * math.tan(theta / 2)
            if min(lu, lv) < need:
                bad["bend"].append((math.degrees(theta), need, min(lu, lv),
                                    nm_, i))

    # --- D3c. 配線は両端が何かに繋がっているか --------------------------
    # ⚠ 配線は**両端に行き先がある**。片側だけ宣言していると、もう一方の
    #   端が空中で終わっていることに気づけない。実機では必ず機器か
    #   コネクタか結束点に繋がるので、留め先が 1 つしかない配線は
    #   宣言漏れ（あるいは経路の描き忘れ）。
    # ⚠ 可動部をまたぐ配線には**余長**が要る。両端のリンクが違えば、
    #   関節が動くたびに端点間距離が変わるので、U ループやケーブルベアで
    #   吸収しなければ引き抜かれる。note にその旨が書かれていなければ
    #   設計意図が残っていない。
    SLACK_WORDS = ("ループ", "余長", "ベア", "たるみ", "遊び")
    for nm_ in F.CABLE:
        if nm_ not in known:
            continue
        tg = F.targets_of(nm_)
        note_all = " ".join(n for _t, _h, _q, n in F.FIXINGS.get(nm_, ()))
        slack = any(w in note_all for w in SLACK_WORDS)
        if len(tg) < 2:
            # ⚠ ここは**検査を弱めている**ので、弱めた根拠と数を必ず出す。
            #   U ループ／ケーブルベアの可動側の端点は姿勢ごとに 425mm 動く
            #   ので、1 本の静的な折れ線では表せない（g0 と g425 で端点が
            #   425mm 離れる）。固定側だけ描くのは妥当だが、「ループと
            #   書けば片端で通る」ことにすると、本当に片端が空中で
            #   終わっている配線も一緒に通ってしまう。
            #   本来は可動側の端点を姿勢から算出して描くべきで、それが
            #   済むまでの免除としてカウントを表に出しておく。
            # ⚠ 2026-07-30: 可動側が通れる帯を実測で探した。
            #   **|Y| 300..320 / Z 851..872（21mm）が唯一の空き帯**で、
            #   ほかは駆動系（モーター Z 878..914 / 駆動軸 Z 833..843 /
            #   軸受ブラケット |Y| 333..336 / ベルト |Y| 232.5..247.5）と
            #   レール取付プレート（|Y| 336..340）で埋まっている。
            #   モーター取付板の腕がこの帯を 1,800mm³ 横切っていたので
            #   Z 872.5..875.5 へ逃がした（板は Z 861..931 なので、
            #   下げると板から離れて「分断」になる。上へ逃がすしかない）。
            #   φ12 の束に必要な 20mm は確保できているので、次はこの帯に
            #   U ループの可動側を描いて免除をなくす。
            (bad["cable_loop"] if slack else bad["cable_end"]).append(
                (nm_, len(tg)))
            continue
        lks = {link_by_part.get(t) for t in tg if t in link_by_part}
        if len(lks) > 1 and not slack:
            bad["cable_slack"].append((nm_, sorted(x for x in lks if x)))

    # --- D4. 工具が入るか ----------------------------------------------
    # ⚠ 締結を宣言できていても、六角レンチが入らなければ実機では組めない。
    #   「設計上は留まっている」と「組み立てられる」は別の話。
    #   頭の座面から工具を差す向きへ φ16 × 45mm の空間を要求する
    #   （M4/M5 の六角レンチのボール側 + 手の逃げ）。締結相手そのものと、
    #   同じ列の隣のねじは通す。
    from build123d import (Align as _Al, Cylinder as _Cyl, Pos as _Pos,
                           Rot as _Rot)
    TOOL_D, TOOL_L = 16.0, 45.0
    for nm_, tl in F.TOOL.items():
        if nm_ not in known:
            continue
        tx, ty, tz, dx, dy, dz = tl
        # 向きベクトルから Z 軸を回す角度を出す（軸に平行な場合だけ扱う）
        if abs(dz) > 0.5:
            rot = _Rot(0, 0, 0) if dz > 0 else _Rot(180, 0, 0)
        elif abs(dy) > 0.5:
            rot = _Rot(-90, 0, 0) if dy > 0 else _Rot(90, 0, 0)
        else:
            rot = _Rot(0, 90, 0) if dx > 0 else _Rot(0, -90, 0)
        space = (_Pos(tx, ty, tz) * rot
                 * _Cyl(TOOL_D / 2, TOOL_L,
                        align=(_Al.CENTER, _Al.CENTER, _Al.MIN)))
        sb = space.bounding_box()
        skip = set(F.targets_of(nm_)) | {nm_}
        # ⚠ 塞いでいる相手が「そのねじより**後に組む**部品」なら、締める
        #   時点にはまだ存在しないので工具は入る。段数（締結の連鎖の深さ）で
        #   近似する。可動部（別リンク）も動かせば避けられる。
        #   これを見ないと、車輪ハブ 16 本・歯付きリング 2 本が
        #   「回せない」と誤検出される（実際はどちらも後付け）。
        my_d = dep.get(nm_, 0)
        my_link = link_by_part.get(nm_)
        worst = (0.0, None)
        for other, lst2 in by_part.items():
            # ⚠ **配線は工具アクセスを塞がない。** ハーネスは骨格を組んで
            #   ねじを締めた**後**に通すもので、締める時点にはまだ無い。
            #   段数（締結の連鎖の深さ）では配線は浅く出る（骨格に直接
            #   結束するため）ので、後付けの近似では拾えなかった。
            if other in skip or other.startswith(("scr", "cab_")):
                continue
            if dep.get(other, 0) > my_d:
                continue                      # 後から組む部品
            if link_by_part.get(other) != my_link:
                continue                      # 可動部は動かせば避けられる
            for so_, bo_ in lst2:
                if bbox_overlap(sb, bo_) < 200.0:
                    continue
                v = overlap_volume(space, so_)
                if v > worst[0]:
                    worst = (v, other)
        # 工具空間の 15% を超えて塞がれていたら回せない
        if worst[0] > 0.15 * 3.1416 * (TOOL_D / 2) ** 2 * TOOL_L:
            bad["tool"].append((worst[0], nm_, worst[1]))

    # --- E. 締結面が「面」になっているか ------------------------------
    # ⚠ 接触しているだけでは締結できない。角が 1 点触れているのと、
    #   座面が重なっているのは別物。ボルト・リベット・溶接は
    #   **座面がある**ことが前提なので、接触部の広がりを見る。
    #   bbox の重なりのうち大きい 2 辺の積を接触面積の目安にする
    #   （点接触なら 0 に近く、面で当たっていれば座面ぶんの値が出る）。
    seat_min = {"BOLT": 150.0, "TSLOT": 150.0, "RIVET": 80.0,
                "BRACKET": 150.0, "WELD": 60.0, "RIVET_": 80.0}
    for name, lst in F.FIXINGS.items():
        if name not in known:
            continue
        # ⚠ ねじ自体に「座面」は要らない。溝ナットやタップに刺さるので
        #   接触面積はねじの断面ぶんしか出ない（M5 なら 20mm²）。
        #   ねじの頭が当たる面の評価は別の話（ワッシャの有無で見る）。
        if name.startswith("scr"):
            continue
        for t, how, _q, _n in lst:
            need = seat_min.get(how)
            if need is None or t not in known:
                continue
            if (name, t) not in contact and (t, name) not in contact:
                continue          # 「離れ」で既に出ている
            best = 0.0
            for sa_, ba_ in by_part.get(name, ()):
                for sb_, bb_ in by_part.get(t, ()):
                    e = sorted(
                        max(0.0, min(hi_a, hi_b) - max(lo_a, lo_b))
                        for lo_a, hi_a, lo_b, hi_b in (
                            (ba_.min.X, ba_.max.X, bb_.min.X, bb_.max.X),
                            (ba_.min.Y, ba_.max.Y, bb_.min.Y, bb_.max.Y),
                            (ba_.min.Z, ba_.max.Z, bb_.min.Z, bb_.max.Z)))
                    best = max(best, e[1] * e[2])
            if best < need:
                bad["seat"].append((name, t, how, best, need))

    # --- F. BRACKET と宣言した継手に、金具とねじの実体があるか ----------
    # ⚠ **BRACKET はどの検査にも掛かっていなかった。** 員数表
    #   （`fastener_bom`）は BRACKET を「金具自身の TSLOT 宣言と二重になる」
    #   という理由で数から外している。正しいのだが、**金具そのものを
    #   描き忘れた継手は、二重どころか 1 つも無いのに素通りする**。
    #   実際 表示器の横材（`disp_beam`）は「BRACKET・各 2-M5」と書いてある
    #   だけで、金具もねじも 1 つも無く、柱の前に浮いていた。
    #   ここでは「a と b の両方に留まる第三の部品（＝金具）があり、その
    #   金具が a 側・b 側それぞれに実体のねじを持つ」ことまで見る。
    #   金具を介さず直接ねじで留まっている継手（貫通ボルト等）も許す。
    served: dict[frozenset, list[str]] = {}
    for nm_ in F.FASTENER:
        tg = frozenset(t_ for t_ in F.targets_of(nm_) if not L.is_screw_part(t_))
        served.setdefault(tg, []).append(nm_)

    def _screws_between(a_, b_):
        return [s_ for tg, names in served.items()
                if a_ in tg and b_ in tg for s_ in names]

    for name, lst in F.FIXINGS.items():
        if name not in known:
            continue
        for t, how, _q, _n in lst:
            if how != "BRACKET" or t not in known:
                continue
            mids = [m for m in F.FIXINGS
                    if m not in (name, t) and m in known
                    and not L.is_screw_part(m)
                    and {name, t} <= set(F.targets_of(m))]
            if not mids:
                if not _screws_between(name, t):
                    bad["brk"].append((name, t, "金具もねじも実体が無い"))
                continue
            for m in mids:
                na_ = len(_screws_between(m, name))
                nb_ = len(_screws_between(m, t))
                if na_ == 0 or nb_ == 0:
                    bad["brk"].append(
                        (name, t, f"金具 {m} のねじが {name} 側 {na_} 本 /"
                                  f" {t} 側 {nb_} 本"))
    return len(sol), len(known), bad


def _fmt(bad):
    # cable_loop は「検査を弱めた箇所」の記録。合否には数えないが
    # 見えるところに出し続ける（黙って免除すると免除が既定になる）。
    return sum(len(v) for k, v in bad.items()
               if k not in ("depth_stat", "cable_loop", "allowed_stat",
                            "allowed_top"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="match",
                    choices=list(ALL_POSES) + ["all", "swept"])
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--limit", type=int, default=30)
    args = ap.parse_args()

    names = (list(POSES) if args.pose == "all"
             else list(SWEPT) if args.pose == "swept" else [args.pose])
    rows = []
    for nm in names:
        t0 = time.time()
        rows.append((nm, *analyze(nm), time.time() - t0))

    if args.md:
        print("組立の厳格チェック。判定は距離のしきい値ではなく"
              "`tr_fix.py` の**固定宣言**を基準にする。\n")
        print("| 姿勢 | ソリッド | 部品 | 未解決 | **分断** | **リンク跨ぎ** | **浮き** | **離れ** "
              "| **未宣言接触** | **食い込み** | 時間 |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
        for nm, ns, npart, bad, dt in rows:
            print(f"| {nm} | {ns} | {npart} | {len(bad['unresolved'])} | "
                  f"**{len(bad['split'])}** | **{len(bad['crosslink'])}** | "
                  f"**{len(bad['orphan'])}** | **{len(bad['detached'])}** | "
                  f"**{len(bad['undeclared'])}** | **{len(bad['dug'])}** | {dt:.1f}s |")
        print()
        if all(_fmt(b) == 0 for _n, _s, _p, b, _t in rows):
            print("**問題なし** ✅\n")
        for nm, _ns, _np, bad, _dt in rows:
            if bad["unresolved"]:
                print(f"\n### {nm}: 存在しない部品を固定先にしている {len(bad['unresolved'])} 件\n")
                for a, t in bad["unresolved"][:args.limit]:
                    print(f"- `{a}` → `{t}`（そんな部品は無い）")
            if bad["split"]:
                print(f"\n### {nm}: 分断された部品 {len(bad['split'])} 件\n")
                print("> 1 部品のはずが離ればなれのソリッドになっている。"
                      "肉抜きやテーパのカットが深すぎる。\n")
                print("| 部品 | ソリッド数 | 塊 |")
                print("|---|---|---|")
                for nm_, n_, g_ in bad["split"][:args.limit]:
                    print(f"| `{nm_}` | {n_} | {g_} |")
            if bad["crosslink"]:
                print(f"\n### {nm}: リンクをまたぐ固定 {len(bad['crosslink'])} 件\n")
                print("> 関節が無いのに別リンクの部品へ留めている。"
                      "実機では動いた瞬間に壊れる。\n")
                print("| 部品 | リンク | 固定先 | リンク | 方法 |")
                print("|---|---|---|---|---|")
                for a_, la, t_, lt, how in bad["crosslink"][:args.limit]:
                    print(f"| `{a_}` | {la} | `{t_}` | {lt} | {how} |")
            if bad["orphan"]:
                print(f"\n### {nm}: 基準からたどり着けない {len(bad['orphan'])} 部品\n")
                print("> 固定の連鎖が切れている＝実機では手で支えないと落ちる。\n")
                for n in bad["orphan"][:args.limit]:
                    tg = "、".join(F.targets_of(n)) or "（固定先の宣言なし）"
                    print(f"- `{n}` → {tg}")
            if bad["detached"]:
                print(f"\n### {nm}: 宣言した固定先と接していない {len(bad['detached'])} 件\n")
                print("| 部品 | 固定先 | 方法 |")
                print("|---|---|---|")
                for a, t, how in bad["detached"][:args.limit]:
                    print(f"| `{a}` | `{t}` | {how} |")
                if len(bad["detached"]) > args.limit:
                    print(f"\n（他 {len(bad['detached']) - args.limit} 件）")
            if bad["undeclared"]:
                print(f"\n### {nm}: 宣言のない接触 {len(bad['undeclared'])} 件\n")
                print("> 触れているのに固定関係が書かれていない＝意図しない干渉か、書き漏れ。\n")
                print("| すきま | 部品 A | 部品 B |")
                print("|---|---|---|")
                for d, a, b in sorted(bad["undeclared"])[:args.limit]:
                    print(f"| {d:.2f} mm | `{a}` | `{b}` |")
                if len(bad["undeclared"]) > args.limit:
                    print(f"\n（他 {len(bad['undeclared']) - args.limit} 件）")
            if bad["seat"]:
                print(f"\n### {nm}: 座面が足りない締結 {len(bad['seat'])} 件\n")
                print("> 接してはいるが、当たっているのが角や線で座面になっていない。\n")
                print("| 部品 | 固定先 | 方法 | 接触 | 要 |")
                print("|---|---|---|---|---|")
                for a_, t_, how, got, need in bad["seat"][:args.limit]:
                    print(f"| `{a_}` | `{t_}` | {how} | {got:.0f} mm² | {need:.0f} mm² |")
            if bad["brk"]:
                print(f"\n### {nm}: BRACKET と書いたのに金具が無い"
                      f" {len(bad['brk'])} 件\n")
                print("> 「金具で留まっている」という**要約**の宣言。金具そのものと、"
                      "金具を留めるねじが実体で入っていなければ、実機では"
                      "ただ突き合わせてあるだけになる。\n")
                print("| 部品 A | 部品 B | 中身 |")
                print("|---|---|---|")
                for a_, t_, why_ in bad["brk"][:args.limit]:
                    print(f"| `{a_}` | `{t_}` | {why_} |")
            if bad["dug"]:
                print(f"\n### {nm}: 食い込み {len(bad['dug'])} 件\n")
                print("| 重なり | 部品 A | 部品 B | 方法 | 許容 |")
                print("|---|---|---|---|---|")
                for v, a, b, how, ok in sorted(bad["dug"], reverse=True)[:args.limit]:
                    print(f"| {v:,.0f} mm³ | `{a}` | `{b}` | {how} | {ok:,.0f} mm³ |")
                if len(bad["dug"]) > args.limit:
                    print(f"\n（他 {len(bad['dug']) - args.limit} 件）")
    else:
        for nm, ns, npart, bad, dt in rows:
            print(f"[{nm}] ソリッド{ns} 部品{npart}  未解決{len(bad['unresolved'])} "
                  f"分断{len(bad['split'])} 自己重なり{len(bad['selflap'])} "
                  f"リンク跨ぎ{len(bad['crosslink'])} "
                  f"浮き{len(bad['orphan'])} 離れ{len(bad['detached'])} "
                  f"未宣言接触{len(bad['undeclared'])} 食い込み{len(bad['dug'])} "
                  f"座面不足{len(bad['seat'])} 公差ぎりぎり{len(bad['margin'])} "
                  f"1点留め{len(bad['single'])} 段数超過{len(bad['deep'])} "
                  f"ねじ山不足{len(bad['tap'])} 工具入らず{len(bad['tool'])} "
                  f"曲げ不足{len(bad['bend'])} 宣言矛盾{len(bad['conflict'])} "
                  f"循環{len(bad['cycle'])} 置いただけ{len(bad['rest'])} "
                  f"金具不在{len(bad['brk'])} "
                  f"配線片端{len(bad['cable_end'])} "
                  f"余長未宣言{len(bad['cable_slack'])} "
                  f"[ループ免除{len(bad['cable_loop'])}] ({dt:.1f}s)")
            for nm_, n_, g_ in bad["split"][:args.limit]:
                print(f"    分断    {nm_}: {n_} ソリッドが {g_} 塊に離れている")
            for a_, t_, how, got, need in bad["seat"][:args.limit]:
                print(f"    座面    {a_} ↔ {t_} [{how}] 接触 {got:.0f}mm² < 要 {need:.0f}")
            for a_, t_, why_ in bad["brk"][:args.limit]:
                print(f"    金具    {a_} ↔ {t_}: {why_}")
            for a_, la, t_, lt, how in bad["crosslink"][:args.limit]:
                print(f"    跨ぎ    {a_}({la}) ↮ {t_}({lt}) [{how}]")
            for a, t in bad["unresolved"][:args.limit]:
                print(f"    未解決  {a} → {t}")
            for n in bad["orphan"][:args.limit]:
                print(f"    浮き    {n}")
            for a, t, how in bad["detached"][:args.limit]:
                print(f"    離れ    {a} ↮ {t} ({how})")
            for d, a, b in sorted(bad["undeclared"])[:args.limit]:
                print(f"    未宣言  {d:5.2f}mm {a} ↔ {b}")
            for v, a, b, how, ok in sorted(bad["dug"], reverse=True)[:args.limit]:
                print(f"    食込み  {v:8,.0f}mm³ {a} ↔ {b} ({how} 許容{ok:,.0f})")
            for v_, nm_, i_, j_ in sorted(
                    bad["selflap"], reverse=True)[:args.limit]:
                print(f"    自己重なり {v_:,.0f}mm³ {nm_} の"
                      f"ソリッド #{i_} と #{j_}")
            for nm_, k_ in bad["cable_end"][:args.limit]:
                print(f"    配線端  {nm_} は留め先が {k_} 個しかない"
                      f" ← 片端が空中で終わっている")
            for nm_, lks_ in bad["cable_slack"][:args.limit]:
                print(f"    余長    {nm_} は {'/'.join(lks_)} をまたぐのに"
                      f" 余長の宣言が無い")
            for nm_, hows_ in bad["rest"][:args.limit]:
                print(f"    置いただけ {nm_} は {hows_} しか宣言が無い"
                      " ← 位置を決めるものが無い")
            for a_, b_, h1_, h2_ in bad["conflict"][:args.limit]:
                print(f"    矛盾    {a_} ↔ {b_} に {h1_} と {h2_} の"
                      f"両方が宣言されている")
            for cyc in bad["cycle"][:args.limit]:
                print(f"    循環    {' → '.join(cyc)}")
            for th_, nd_, got_, nm_, i_ in sorted(
                    bad["bend"], reverse=True)[:args.limit]:
                print(f"    曲げ    {nm_} の第{i_}節点は {th_:.0f}° 折れ、"
                      f"直線 {nd_:.0f}mm 要るのに {got_:.0f}mm しかない")
            for v_, nm_, ob_ in sorted(bad["tool"], reverse=True)[:args.limit]:
                print(f"    工具    {nm_} は {ob_} に塞がれて回せない"
                      f"（{v_:,.0f}mm³）")
            for th_, nd_, nm_, t_, sz_ in sorted(bad["tap"])[:args.limit]:
                print(f"    ねじ山  {nm_} → {t_} は厚み {th_:.1f}mm、"
                      f"M{sz_} のタップには {nd_:.1f}mm 要る")
            for d_, nm_ in sorted(bad["deep"], reverse=True)[:args.limit]:
                print(f"    段数    {nm_} は基準面から {d_} 段目"
                      f" ← 公差が積み上がる")
            if bad.get("allowed_stat") and bad["allowed_stat"][0]:
                k_, v_ = bad["allowed_stat"]
                print(f"    許容内の重なり {k_} 組・合計 {v_:,.0f}mm³"
                      f"（上限 {ALLOWED_BUDGET:,.0f}+{ALLOWED_PER_SCREW:.0f}/ねじ"
                      f"・圧入/噛み合いとして"
                      f"宣言済み・ビューアでは干渉に見える）")
                # ⚠ 合計だけ出していると、中身が入れ替わっても気づけない。
                #   大きい順に出しておけば、ベルトを平板で描き直したような
                #   退行が一目で分かる。
                for v2, a2, b2, h2 in bad.get("allowed_top", []):
                    print(f"      {v2:8,.0f}mm³ {a2} ↔ {b2} [{h2}]")
            for tot_, lim_ in bad.get("allowed_over", []):
                print(f"    許容内合計 {tot_:,.0f}mm³ が上限 {lim_:,.0f} を超えた"
                      f" ← 新しい描き間違いが入った疑い")
            if bad.get("depth_stat"):
                mx_, md_, n_ = bad["depth_stat"]
                print(f"    締結の段数 最大{mx_}段（中央値{md_}段・部品{n_}個）")
            for v, a, t_, how in sorted(bad["single"], reverse=True)[:args.limit]:
                print(f"    1点留め  {v:8,.0f}mm³ {a} → {t_} [{how}] のみ"
                      f" ← この 1 点を軸に回る")
            for d, a, b, how in sorted(bad["margin"], reverse=True)[:args.limit]:
                print(f"    公差    {d:.2f}mm {a} ↔ {b} [{how}]"
                      f" ← 板厚や姿勢が少し変わると離れる")
            if bad["dug"]:
                tot = sum(v for v, *_ in bad["dug"])
                print(f"    食い込み合計 {tot:,.0f}mm³"
                      f"（最大 {max(v for v, *_ in bad['dug']):,.0f}mm³）")
    return 1 if any(_fmt(b) for _n, _s, _p, b, _t in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
