"""板材の余肉・過大寸法の検査 — 「その板、本当に全面が要るのか」を機械的に見る.

    python scripts/plate_audit.py [--pose match] [--md] [--limit N] [--all]

板は「大きく作っておけば穴の位置に困らない」ので、設計中に必ず太る。
太った板は figure の上では何も壊さないので、どの検査にも引っかからずに残る。
ここでは板 1 枚ごとに**使っている面積**と**全面積**を突き合わせて、
使っていない領域を質量に換算して出す。

出す不良は 4 種類:

  A. 使用率が低い（idle）… 締結・接触が届いている領域が板の実肉のごく一部
  B. 縁の余り（trim）   … 一番外側の締結・接触から板の端までが縁代より遠い。
                           そこには荷重が通らないので、切り落としても剛性は変わらない
  C. 肉抜き無し（solid）… 肉抜きを通していない大面積の板
  D. 板厚過大（thick）  … 支点間を渡る帯とみなして曲げ応力を見積もり、
                           安全率が要求の 2 倍以上あるもの

既存の検査との切り分け
  * `scripts/fea_frame.py` は**押出材の骨格**だけを梁で解く。板は等価梁 2 本に
    潰してあるので、板 1 枚の余肉は原理的に見えない。
  * `scripts/assembly_check.py` の「座面不足」は**接触面積が足りない**方向の検査。
    ここは逆に**接触に対して板が大きすぎる**方向を見るので、重複しない。
  * `scripts/fasteners.py` は締結 1 か所ごとのボルト本数の話で、板の面積は見ない。

⚠ **判定は「宣言された固定関係」を基準にする。** 距離で接触を拾うと、
  たまたま近くを通っている部品まで「使っている」ことになり、余肉が消える。
  荷重が通るのは宣言した相手（と、そこに刺さるねじ）だけ。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402

# --- しきい値 ---------------------------------------------------------------
PLATE_T_MAX = 12.0        # これより厚いものは「板」と呼ばない（ブロック・押出材）
PLATE_MIN_SIDE = 30.0     # 短辺がこれ未満は「棒」。曲げの効き方が違う
PLATE_AREA_MIN = 5_000.0  # mm² 板の投影面積。これ未満は削っても効かない
PLATE_FILL_MIN = 0.25     # bbox に対する実肉の比。これ未満は L 字・曲げ板

GROW = 1.0                # mm 接触とみなす隙間（assembly_check の CONTACT_TOL 相当）
# ⚠ ねじ 1 本の座面は頭の直径しかないが、**荷重は板の中で 45° に広がる**。
#   頭の縁から板厚ぶん外側までが座として効く。ねじの bbox だけで数えると、
#   ボルト留めの板が軒並み「9 割が余肉」になって使い物にならない。
SCREW_SPREAD = 10.0       # mm ねじ 1 本が板に効かせる半径の増分
EDGE_MARGIN = 12.0        # mm 縁代。M5 の通し穴（φ5.5）+ 縁 2d が目安
TRIM_MIN = 10.0           # mm これ以下の余りは切っても意味が無い（曲げ代・面取り）
WASTE_G_MIN = 25.0        # g これ未満の余肉は報告しない（部品点数の割に効かない）

LIGHTEN_FILL = 0.85       # 実肉率がこれ以上なら「肉抜きされていない」
LIGHTEN_AREA_MIN = 25_000.0   # mm² 肉抜きを要求する下限面積

IDLE_RATIO = 0.60         # 使用率がこれ未満なら「使っていない領域が広い板」
IDLE_AREA_MIN = 15_000.0  # mm² 使用率を問う下限面積

LEDGER_GAP_KG = 0.030     # 台帳と実体の差がこれ以上なら出す
LEDGER_GAP_REL = 0.30     # かつ相対でこれ以上ずれていること

# A5052-H32 の耐力 195MPa に安全率 2.2。fea_frame.py の押出材と同じ考え方。
SIGMA_ALLOW = 90.0        # MPa
DYN = 3.0                 # 動的倍率（走行の突き上げ・射出反力）
SF_TARGET = 2.0           # 要求安全率
SF_MAX = 4.0              # これを超えたら板厚が過大
STOCK_T = (0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)   # 手に入る板厚

# ⚠ **曲げ応力だけで板厚を決めてはいけない。** ぶら下がる質量から出した
#   曲げ応力は、どの板でも「t0.8 で足りる」と言ってくる（実際に最初の版は
#   ガセットまで t0.8 を勧めてきた）。板厚を決めているのは曲げではなく
#     ・ボルト穴の支圧（薄い板は穴が伸びて締結が緩む）
#     ・座屈と取り回し（大きい薄板は手で持つだけで曲がる）
#     ・切削の実用下限（t2 未満のアルミを CNC で削ると反る）
#   の 3 つ。ここを入れないと、検査が「作れない板」を勧める道具になる。
T_FLOOR_MIN = 2.0         # mm 切削アルミ板の実用下限
T_FLOOR_BOLT = 0.5        # ボルト呼び径に対する最小板厚（支圧）
T_FLOOR_SPAN = 1 / 200.0  # 支点間に対する最小板厚（たわみ・取り回し）
# 薄板でよい材料（曲げ加工品・薄いポリカ）は上の下限を当てない
T_FLOOR_FREE = ("PC", "SUS304")
# ⚠ 樹脂中空板は**板厚が規格で決まっている**（プラダン t2.5/3/4/5、
#   テクセル t5/10）。曲げ応力だけで「t0.8 へ」と言っても買えない。
#   しかも中空板は板厚を落とすと面剛性が一気に落ちる（断面二次が t³）。
STOCK_T_BOARD = {"PP_DANPLA": (2.5, 3.0, 4.0, 5.0), "TEKCELL": (5.0, 10.0)}
SCREW_D = {5.5: 3.0, 7.0: 4.0, 8.5: 5.0, 10.0: 6.0}   # 頭部径 → 呼び径

# 位置決めをしない宣言。これで留まっている相手は荷重を渡さないので、
# 「板を使っている」ことにはしない（tr_fix.depth_of の NO_STACK と同じ理由）。
NO_LOAD = ("ROUTE", "BUNDLE", "CONNECT", "CONTAIN")
# 荷重を骨格へ渡す締結。板厚の検査で「支点」として数える。
# ⚠ ADJUST（レベリング座）も入れる。板どうしは離れているが、**荷重は
#   調整ねじ 4 本を通って確かに渡る**。外すと可動板が「支点 0」になり、
#   板厚の検査が「どこにも留まっていない板」として扱う。
STRUCT = ("BOLT", "TSLOT", "BRACKET", "RIVET", "WELD", "PRESS", "SHAFT",
          "ADJUST")

# 検査から外す部品と、その理由。**理由を書かせる**のは、面倒になって
# ここに足すのを防ぐため（assembly_check.REST_OK と同じ運用）。
SKIP = {
    "mascot_envelope": "規定 3.1.3 の外形（実体ではない）",
    "mascot_rag": "マスコットが持っている雑巾（フェルト）",
    "mascot_badge": "マスコットのゼッケン（フェルト）",
    "bucket": "競技で審判が置く購入品",
    "bucket_2l_datum": "規定 3.2.3c の目盛り位置（実体ではない）",
    "chair_5go": "規定で載せる椅子（形は代表寸法）",
    "press_pad": "スポンジ。面で布を押さえるのが仕事なので全面が要る",
    "retard_pad": "リタードパッド。門の幅いっぱいが摩擦面",
}
# 前方一致で外す部品。理由つき。
SKIP_PREFIX = {
    "rail_": "MISUMI SRX3616 の段。購入品なので寸法を変えられない",
    "skirt_": "カバー。覆う面積そのものが仕事なので、接触では測れない",
    "hop_": "ホッパーの壁。囲う面積が仕事（プラダン 0.2 mg/mm³ で軽い）",
    "cab_": "配線束",
    "scr": "ねじ",
    "wire_": "配線",
}

# ⚠ 色を失った部品の材質。`L.mat()` が付けた色は、そのあとブーリアンを
#   掛けると消える（`L.mat(...) - Cylinder(...)` の形）。材質が分からないと
#   「削って何 g 減るか」が出せないので、落ちたものだけここで補う。
#   ここに載る部品が増えたら、その部品は STEP でも無色になっている。
# 購入品エンベロープの**実効密度** [g/mm³] = カタログ質量 ÷ 外形の体積。
# ⚠ **形はあるのに密度が無い部品**が残っていた。LiDAR・カメラ・モーター・
#   電池・基板・ベルトは外形だけを箱や円柱で描いてあり、その材質は
#   `P.DENSITY`（製作品の材料）に無い。すると体積×密度が **0** になり、
#   「この板は何も支えていない」と判定される。板厚の検査からも
#   トポロジー最適化の対象からも**静かに落ちる**（実際それで、LiDAR
#   ブラケット 3 枚とバケツ座と上押さえ板が「荷重を渡す相手が宣言に無い」
#   として最適化されないまま残っていた）。
# ⚠ `P.DENSITY` のほうへは足さないこと。あちらは `LEDGER.add_solid` が
#   製作品の質量を出すのに使う表で、購入品はカタログ値で別に計上している。
#   混ぜると同じ物が二重に載る。
ENVELOPE_RHO = {
    "SENSOR": 0.65e-3,      # 2D LiDAR 130g / φ60×70 ≒ 198,000mm³
    "MOTOR": 2.70e-3,       # M3508 P19 365g / φ42×98 ≒ 136,000mm³
    "MOTOR_SHAFT": 7.85e-3,  # 出力軸（鋼）
    "PCB": 1.80e-3,         # ガラエポ + 部品
    "SCREEN": 2.50e-3,      # 表示器の前面ガラス（280g はベゼル側に計上済み）
    "BATTERY": 3.40e-3,     # 6S LiPo 500g / 105×35×40 = 147,000mm³
    "ESTOP": 1.00e-3,       # きのこ形スイッチ（樹脂 + 接点）
    "RUBBER": 1.40e-3,      # 歯付きベルト（ゴム + 芯線）
    "CABLE": 1.40e-3,       # 配線束（銅 + 被覆）
    # マスコットは EPP の芯 + フェルト。**規定 3.1.3 で重量制限外**なので
    # 台帳には載せないが、椅子マウントの荷重には効く（19.6L で約 1.2kg）。
    "MASCOT": 0.061e-3, "MASCOT_SUIT": 0.061e-3, "MASCOT_TRIM": 0.061e-3,
    "MASCOT_RAG": 0.061e-3, "MASCOT_DARK": 0.061e-3,
}

MAT_FALLBACK = {
    "pedestal_ring0": "A5052", "pedestal_ring1": "A5052", "pitch_side_L": "A5052", "pitch_side_R": "A5052",
    "press_plate": "A5052", "press_shelf": "A5052", "press_guide_L": "A5052",
    "press_guide_R": "A5052", "car_beam": "A5052", "car_rib_L": "A5052",
    "car_rib_R": "A5052", "car_side_L": "A5052", "car_side_R": "A5052",
    "rail_plate_L": "A5052", "rail_plate_R": "A5052", "yaw_arm_f": "A5052",
    "yaw_arm_r": "A5052", "yaw_motor_deck": "A5052", "yaw_motor_post_m": "A5052",
    "yaw_motor_post_p": "A5052", "yaw_side_L": "A5052", "yaw_side_R": "A5052",
    "belt_clamp_L": "A5052", "belt_clamp_R": "A5052", "lift_guide": "A5052",
    "hinge_blk_L": "A5052", "hinge_blk_R": "A5052", "fork_root": "A5052",
    "hub_fl": "A5052", "hub_fr": "A5052", "hub_rl": "A5052", "hub_rr": "A5052",
    "mount_brk_fl": "A5052", "mount_brk_fr": "A5052", "mount_brk_rl": "A5052",
    "mount_brk_rr": "A5052", "sing_mot_brk": "PETG", "press_face": "A5052",
    # ⚠ ローラー軸は**アルミ A2017 の中空軸**（台帳のラベルどおり）。
    #   STEEL と置くと 1 本 297g になり、台帳の 105g と 3 倍ずれて
    #   「台帳が間違っている」と誤報する。材質を推測で埋めるときは、
    #   台帳のラベルに書いてある材質と突き合わせること。
    "hinge_shaft": "STEEL", "roller_shaft_u": "A5052", "roller_shaft_d": "A5052",
    "beltshaft_drvL": "STEEL", "beltshaft_drvR": "STEEL",
    "beltshaft_idlL": "STEEL", "beltshaft_idlR": "STEEL",
    "brk_mast_arm_L_m": "ADC12", "brk_mast_arm_R_m": "ADC12",
    "aim_camera": "SENSOR", "power_led": "PCB", "cam_follower": "PETG",
    "hop_floor": "PP_DANPLA", "hop_side_L": "PP_DANPLA", "hop_side_R": "PP_DANPLA",
    "press_bush_L": "POM", "press_bush_R": "POM",
    # ⚠ 計測輪のキャリッジ金具と取付板は **PETG**（2026-08-07 に確定）。
    #   どちらも曲げフランジと丸ボスを持つ 3D 形状で、自校の設備では
    #   アルミ板から作れない（`export_fab.CAN_BEND` / `CAN_MILL_3D` = False）。
    "odo_arm0": "PETG", "odo_arm1": "PETG", "odo_arm2": "PETG",
    "odo_brk0": "PETG", "odo_brk1": "PETG", "odo_brk2": "PETG",
    "beltbrk_drv_L": "A5052", "beltbrk_drv_R": "A5052",
    "beltbrk_idl_L": "A5052", "beltbrk_idl_R": "A5052",
    "car_brk_L": "A5052", "car_brk_R": "A5052",
    "press_post_L": "A5052", "press_post_R": "A5052",
    "yaw_pulley_big": "A5052",
    "tine0": "SUS304", "tine1": "SUS304", "tine2": "SUS304",
    "tine3": "SUS304", "tine4": "SUS304",
}
# 質量を持たない表示物（配線の束・エンベロープ）。材質不明として数えない。
MAT_IGNORE_PREFIX = ("cab_", "wire_", "rag_", "scr")

# ⚠ 板の外形は **姿勢がゼロの状態**で測る。可動部の板は姿勢のぶんだけ
#   回っているので、世界座標の bbox が実寸より大きく出る。実際 仰角 45°の
#   まま測ると 230×338 の側板が 343×343 の正方形に見え、
#   「四方に 40〜90mm の余肉がある」という嘘の指摘が出ていた。
#   板の寸法は姿勢に依らないので、回転を全部 0 にした姿勢で測る。
POSE_FLAT = dict(P.POSE_MATCH, yaw=0.0, pitch=0.0, tilt=0.0, grab=0.0, press=0.0)
POSES = {"flat": POSE_FLAT, "match": P.POSE_MATCH, "stowed": P.POSE_STOWED,
         "loading": P.POSE_LOADING}


# ---------------------------------------------------------------------------
# 組立を作り、部品ごとに (ソリッド, bbox, 材質) を集める
# ---------------------------------------------------------------------------
def build_parts(pose_name: str = "match"):
    """(部品名 -> 情報) と固定宣言モジュールを返す。

    ⚠ 材質は **`put()` の時点の色**から取る。`L.mat()` が材質ごとに色を
      付けているので、色は事実上の材質タグになっている。組立が終わった
      あとの Solid では色が落ちている（`_solids()` が畳むときに消える）ので、
      put を包んで名前と一緒に控える。
    """
    import tr_assembly as A
    import tr_fix as F
    import validate as V
    import assembly_check as AC

    rgb2mat = {tuple(round(c, 4) for c in v): k for k, v in L.MAT_COLOR.items()}
    mat_by_name: dict[str, str] = {}
    orig_put = A.put
    # ⚠ **手書きの質量は必ず腐る。** 台帳には `LEDGER.add(ラベル, kg, "体積概算")`
    #   で手入力した行があり、形を直したあとも数字はそのまま残る。実際
    #   「旋回アーム A5052 t6 60×640」は 0.062kg と書いてあったが、
    #   実体は 0.617kg（10 倍）。35kg 規定に対して 1kg 近い誤差が
    #   どの検査にも出ていなかった。
    #   そこで put の直後に来る LEDGER.add を、その部品の申告とみなして
    #   実体の体積 × 密度と突き合わせる（`add_solid` は実体から計算して
    #   いるので対象外）。
    #   ⚠ 対応づけは「**前の台帳行から今の台帳行までに置いた部品**」。
    #     put と台帳行が 1:1 に並んでいない箇所（ループでまとめて置いて
    #     最後に qty で計上する等）では、関係ない部品が混じる。
    #     だからこの検査は**合否ではなく、人が見る候補**として出す。
    pending: list[str] = []
    claims: list[tuple] = []
    orig_add = L.MassLedger.add
    orig_add_solid = L.MassLedger.add_solid

    def rec_add(self, label, unit_kg, source, group, qty=1):
        if pending:
            claims.append((tuple(pending), label, unit_kg, source, qty))
        pending.clear()
        return orig_add(self, label, unit_kg, source, group, qty)

    def rec_add_solid(self, label, shape, material, group, qty=1):
        pending.clear()        # 実体から計算した行。ここで区切る
        return orig_add_solid(self, label, shape, material, group, qty)

    def rec_put(parts, shape, name, to=None, how=None, note="", tool=None):
        pending.append(name)
        col = getattr(shape, "color", None)
        if col is None:
            for ch in getattr(shape, "children", []) or []:
                col = getattr(ch, "color", None)
                if col is not None:
                    break
        if col is not None:
            key = tuple(round(float(x), 4) for x in tuple(col)[:3])
            m = rgb2mat.get(key)
            if m is not None:
                mat_by_name[name] = m
        return orig_put(parts, shape, name, to, how, note, tool)

    A.put = rec_put
    L.MassLedger.add = rec_add
    L.MassLedger.add_solid = rec_add_solid
    try:
        shape = A.build(POSES[pose_name])
    finally:
        A.put = orig_put
        L.MassLedger.add = orig_add
        L.MassLedger.add_solid = orig_add_solid

    info: dict[str, dict] = {}
    for path, sol, box in V.solids_with_bbox(shape):
        nm = AC.part_name(path)
        d = info.setdefault(nm, {"vol": 0.0, "box": [1e18, -1e18, 1e18, -1e18, 1e18, -1e18],
                                 "n": 0, "solids": []})
        # ⚠ 実体も持たせる。ここは**部品名 → 材質**を決められる唯一の場所
        #   （材質は put() のときの色にしか無い）なので、実体が要る側
        #   （`export_fab.py` の製作データ書き出し）が同じことを書き写すと、
        #   材質の対応表が 2 つに増えて必ず片方が腐る。
        #   `shape` が生きているあいだ参照を持つだけなので、増える分は無い。
        d["solids"].append(sol)
        d["vol"] += sol.volume
        b = d["box"]
        b[0] = min(b[0], box.min.X); b[1] = max(b[1], box.max.X)
        b[2] = min(b[2], box.min.Y); b[3] = max(b[3], box.max.Y)
        b[4] = min(b[4], box.min.Z); b[5] = max(b[5], box.max.Z)
        d["n"] += 1
    unknown = []
    for nm, d in info.items():
        m = mat_by_name.get(nm) or MAT_FALLBACK.get(nm)
        if m is None and not nm.startswith(MAT_IGNORE_PREFIX):
            unknown.append(nm)
        d["mat"] = m or "A5052"
        d["rho"] = P.DENSITY.get(d["mat"]) or ENVELOPE_RHO.get(d["mat"], 0.0)
        d["mass"] = d["vol"] * d["rho"] / 1000.0        # kg
    return info, F, sorted(unknown), claims


def ledger_gaps(info, claims):
    """台帳の手書き質量と実体の食い違い。(差[kg], 部品, ラベル, 申告, 実体) の並び。

    ⚠ 「体積概算」と書いてある行だけを見る。カタログ値（購入品）や
      「概算」（配線・ねじ類のまとめ計上）は実体と 1:1 で対応しないので、
      比べても意味が無い。
    """
    out = []
    for names, label, unit_kg, source, qty in claims:
        if "体積" not in source:
            continue
        grp = [n for n in names if n in info]
        # ⚠ **1 個の put と 1 行の台帳が並んでいるときだけ比べる。**
        #   まとめて置いてから qty で計上している箇所は、購入品（レールの段・
        #   プーリ）まで同じ塊に入るので、体積 × 密度が別物になる
        #   （レール取付板 290g の行に、スチールのレール 3 段が混ざって
        #     「実体 1251g」と出た）。狼少年にしないために黙る。
        if len(grp) != 1:
            continue
        got = sum(info[n]["mass"] for n in grp)
        claim = unit_kg * qty
        if got <= 0:
            continue
        diff = got - claim
        if abs(diff) >= LEDGER_GAP_KG and abs(diff) / got >= LEDGER_GAP_REL:
            out.append((diff, grp, label, claim, got, qty))
    out.sort(key=lambda r: -abs(r[0]))
    return out


def subtree_mass(info, F) -> dict[str, float]:
    """固定の連鎖を根から張った木で、その部品にぶら下がる質量 [kg]。

    板厚が過大かどうかは「その板が何を支えているか」で決まる。
    宣言をたどれば、どの部品がどの板の上に乗っているかは分かる。
    """
    adj: dict[str, set[str]] = {}
    for name, lst in F.FIXINGS.items():
        for t, h, _q, _n in lst:
            if h in NO_LOAD:
                continue
            adj.setdefault(name, set()).add(t)
            adj.setdefault(t, set()).add(name)
    parent: dict[str, str | None] = {}
    order: list[str] = []
    frontier = [r for r in F.ROOTS]
    for r in frontier:
        parent[r] = None
    while frontier:
        nxt = []
        for cur in frontier:
            for n in sorted(adj.get(cur, ())):
                if n not in parent:
                    parent[n] = cur
                    order.append(n)
                    nxt.append(n)
        frontier = nxt
    out = {nm: info.get(nm, {}).get("mass", 0.0) for nm in parent}
    for nm in reversed(order):                    # 葉から根へ足し上げる
        p = parent[nm]
        if p is not None:
            out[p] = out.get(p, 0.0) + out[nm]
    return out


# ---------------------------------------------------------------------------
# 板ごとの幾何
# ---------------------------------------------------------------------------
def plan_of(box, k: int):
    """厚み軸 k を除いた 2 軸の (lo, hi) を返す。"""
    ax = [i for i in range(3) if i != k]
    return [(box[2 * i], box[2 * i + 1]) for i in ax], ax


def rect_inter(r1, r2):
    out = []
    for (a0, a1), (b0, b1) in zip(r1, r2):
        lo, hi = max(a0, b0), min(a1, b1)
        if hi <= lo:
            return None
        out.append((lo, hi))
    return out


def union_area(rects) -> float:
    """軸並行矩形の和の面積（座標圧縮）。"""
    if not rects:
        return 0.0
    xs = sorted({v for r in rects for v in r[0]})
    ys = sorted({v for r in rects for v in r[1]})
    a = 0.0
    for i in range(len(xs) - 1):
        cx = (xs[i] + xs[i + 1]) / 2
        for j in range(len(ys) - 1):
            cy = (ys[j] + ys[j + 1]) / 2
            for r in rects:
                if r[0][0] <= cx <= r[0][1] and r[1][0] <= cy <= r[1][1]:
                    a += (xs[i + 1] - xs[i]) * (ys[j + 1] - ys[j])
                    break
    return a


def partners_of(F, nm: str):
    """(相手, 方法) の一覧。自分が宣言した先と、自分を宣言した相手の両方。"""
    out = []
    for t, h, _q, _n in F.FIXINGS.get(nm, ()):
        out.append((t, h))
    for other, lst in F.FIXINGS.items():
        for t, h, _q, _n in lst:
            if t == nm:
                out.append((other, h))
    return out


def audit(info, F):
    """(idle, trim, solid, thick) の 4 つの不良リストを返す。"""
    sub = subtree_mass(info, F)
    idle, trim, solid, thick = [], [], [], []
    for nm, d in sorted(info.items()):
        if nm in SKIP or nm.startswith(tuple(SKIP_PREFIX)):
            continue
        b = d["box"]
        ext = [b[1] - b[0], b[3] - b[2], b[5] - b[4]]
        k = min(range(3), key=lambda i: ext[i])
        t = ext[k]
        plan, ax = plan_of(b, k)
        side = [plan[0][1] - plan[0][0], plan[1][1] - plan[1][0]]
        area = side[0] * side[1]
        if t > PLATE_T_MAX or min(side) < PLATE_MIN_SIDE or area < PLATE_AREA_MIN:
            continue
        fill_box = d["vol"] / (area * t)
        if fill_box < PLATE_FILL_MIN:
            continue                       # L 字・曲げ板。平板の話ではない
        mat_area = d["vol"] / t            # 実肉の面積（肉抜き後）
        rho = d["rho"]

        # --- 接触している矩形を集める ---------------------------------
        rects, sup, bolt_d = [], [], 0.0
        for tgt, how in partners_of(F, nm):
            if how in NO_LOAD or tgt not in info:
                continue
            ob = info[tgt]["box"]
            grow = SCREW_SPREAD if tgt.startswith("scr") else GROW
            if tgt.startswith("scr"):
                # ねじの bbox は (頭部径, 頭部径, 全長)。**中央値が頭部径**。
                # 最大を取ると全長を拾って M6 と読み違える。
                head = sorted((ob[1] - ob[0], ob[3] - ob[2], ob[5] - ob[4]))[1]
                for hd, dd in SCREW_D.items():          # 頭部径から呼び径を引く
                    if abs(head - hd) < 1.2 or abs(head - hd - 2.0) < 1.2:
                        bolt_d = max(bolt_d, dd)
            gb = [ob[0] - grow, ob[1] + grow, ob[2] - grow, ob[3] + grow,
                  ob[4] - grow, ob[5] + grow]
            # 厚み方向で届いていなければ接触ではない（宣言だけの相手）
            if gb[2 * k] > b[2 * k + 1] + GROW or gb[2 * k + 1] < b[2 * k] - GROW:
                continue
            gplan, _ = plan_of(gb, k)
            r = rect_inter(plan, gplan)
            if r is None:
                continue
            rects.append(r)
            if how in STRUCT:
                sup.append((r, tgt, how))
        if not rects:
            continue                       # 接触が拾えない板は判定しない

        # A. 縁の余り: 使っている範囲 + 縁代 で板を切り直したら何 g 減るか
        env = [(min(r[i][0] for r in rects) - EDGE_MARGIN,
                max(r[i][1] for r in rects) + EDGE_MARGIN) for i in (0, 1)]
        keep = rect_inter(plan, env) or plan
        keep_area = (keep[0][1] - keep[0][0]) * (keep[1][1] - keep[1][0])
        cut = [(plan[i][0] - keep[i][0], plan[i][1] - keep[i][1]) for i in (0, 1)]
        waste_g = (area - keep_area) * t * fill_box * rho
        if waste_g >= WASTE_G_MIN and max(abs(c) for cc in cut for c in cc) >= TRIM_MIN:
            trim.append((waste_g, nm, d["mat"], t, side, cut, ax, keep_area / area))

        # A'. 使用率 = 「締結・接触が届いている面積」÷「実肉の面積」
        used = union_area([[(max(plan[i][0], r[i][0] - EDGE_MARGIN),
                             min(plan[i][1], r[i][1] + EDGE_MARGIN)) for i in (0, 1)]
                           for r in rects])
        ratio = min(1.0, used / mat_area) if mat_area > 0 else 1.0
        idle_g = (mat_area - used) * t * rho
        if ratio < IDLE_RATIO and mat_area >= IDLE_AREA_MIN and idle_g >= WASTE_G_MIN:
            idle.append((idle_g, nm, d["mat"], t, side, ratio, len(rects)))

        # B. 肉抜きが入っていない大面積の板
        if (fill_box >= LIGHTEN_FILL and mat_area >= LIGHTEN_AREA_MIN
                and d["mat"] in ("A5052", "SUS304", "PETG")):
            # 既定の φ40 / ピッチ 70 / 余白 30 で入る穴の数から削減量を見積もる
            nx = max(0, int((side[0] - 60.0) // 70.0))
            ny = max(0, int((side[1] - 60.0) // 70.0))
            gain = nx * ny * math.pi * 20.0 ** 2 * t * rho
            if gain >= WASTE_G_MIN:
                solid.append((gain, nm, d["mat"], t, side, nx * ny, ratio))

        # C. 板厚が荷重に対して過大か
        if len(sup) >= 2:
            def ctr(r):
                return ((r[0][0] + r[0][1]) / 2, (r[1][0] + r[1][1]) / 2)
            span, pair = 0.0, None
            for i in range(len(sup)):
                for j in range(i + 1, len(sup)):
                    ci, cj = ctr(sup[i][0]), ctr(sup[j][0])
                    dsp = math.hypot(ci[0] - cj[0], ci[1] - cj[1])
                    if dsp > span:
                        span, pair = dsp, (sup[i][1], sup[j][1])
            load_kg = max(sub.get(nm, d["mass"]), d["mass"])
            w_eff = mat_area / max(span, 1.0)          # 実肉から出した等価幅
            if span > 20.0 and load_kg > 0 and w_eff > 1.0:
                mom = load_kg * 9.81 * DYN * span / 4.0        # N·mm
                sigma = mom / (w_eff * t * t / 6.0)
                sf = SIGMA_ALLOW / max(sigma, 1e-9)
                if sf > SF_MAX:
                    t_need = t * math.sqrt(SF_TARGET / sf)
                    stock = STOCK_T_BOARD.get(d["mat"], STOCK_T)
                    if d["mat"] not in T_FLOOR_FREE and d["mat"] not in STOCK_T_BOARD:
                        t_need = max(t_need, T_FLOOR_MIN,
                                     T_FLOOR_BOLT * (bolt_d or 4.0),
                                     T_FLOOR_SPAN * span)
                    t_new = min([s for s in stock if s >= t_need] or [t])
                    if t_new < t - 1e-6:
                        gain = mat_area * (t - t_new) * rho
                        if gain >= WASTE_G_MIN:
                            thick.append((gain, nm, d["mat"], t, t_new, sf, span,
                                          load_kg, pair))
    for lst in (idle, trim, solid, thick):
        lst.sort(reverse=True)
    return idle, trim, solid, thick


AXN = "XYZ"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="flat", choices=list(POSES))
    ap.add_argument("--limit", type=int, default=15)
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--all", action="store_true", help="板と判定した部品を全部出す")
    args = ap.parse_args()

    info, F, unknown, claims = build_parts(args.pose)
    idle, trim, solid, thick = audit(info, F)
    gaps = ledger_gaps(info, claims)

    if unknown:
        print(f"⚠ 材質が取れなかった部品 {len(unknown)} 個: {', '.join(unknown[:12])}"
              + (" …" if len(unknown) > 12 else ""))

    if args.all:
        print("\n--- 板と判定した部品 ---")
        for g, nm, mt, t, side, cut, ax, keep in trim:
            print(f"  {nm:22s} {mt:9s} t{t:4.1f} {side[0]:6.0f}×{side[1]:6.0f} "
                  f"余肉 {g:6.0f}g")

    tot = sum(x[0] for x in trim) + sum(x[0] for x in solid) + sum(x[0] for x in thick)
    if args.md:
        print(f"\n### 板材の余肉 — 合計 {tot / 1000:.2f} kg 分の削減余地\n")
        print("| 板 | 材質 | 板厚 | 外形 [mm] | 不良 | 削減 [g] | 改善案 |")
        print("|---|---|---|---|---|---|---|")
    for g, nm, mt, t, side, ratio, nr in idle[:args.limit]:
        why = f"締結・接触 {nr} か所が届く面積は実肉の {ratio * 100:.0f}% しかない"
        if args.md:
            print(f"| {nm} | {mt} | t{t:.1f} | {side[0]:.0f}×{side[1]:.0f} | 使用率 "
                  f"{ratio * 100:.0f}% | ({g:.0f}) | {why}。荷重経路で抜くか外形を切る |")
        else:
            print(f"[遊休] {nm:22s} {mt:9s} t{t:4.1f} {side[0]:5.0f}×{side[1]:5.0f} "
                  f"→ 使用率 {ratio * 100:3.0f}% ({g:5.0f}g 分が遊んでいる): {why}")
    for g, nm, mt, t, side, cut, ax, keep in trim[:args.limit]:
        detail = " / ".join(
            f"{AXN[ax[i]]}{'-' if s == 0 else '+'}側 {abs(c):.0f}mm"
            for i in (0, 1) for s, c in enumerate(cut[i]) if abs(c) >= TRIM_MIN)
        if args.md:
            print(f"| {nm} | {mt} | t{t:.1f} | {side[0]:.0f}×{side[1]:.0f} | 縁の余り | "
                  f"{g:.0f} | {detail} を切る（残 {keep * 100:.0f}%） |")
        else:
            print(f"[余肉] {nm:22s} {mt:9s} t{t:4.1f} {side[0]:5.0f}×{side[1]:5.0f} "
                  f"→ {g:5.0f}g 削減: {detail}")
    for g, nm, mt, t, side, nh, ratio in solid[:args.limit]:
        if args.md:
            print(f"| {nm} | {mt} | t{t:.1f} | {side[0]:.0f}×{side[1]:.0f} | 肉抜き無し | "
                  f"{g:.0f} | φ40 を {nh} 個入れる（使用率 {ratio * 100:.0f}%） |")
        else:
            print(f"[肉抜] {nm:22s} {mt:9s} t{t:4.1f} {side[0]:5.0f}×{side[1]:5.0f} "
                  f"→ {g:5.0f}g 削減: φ40 肉抜き {nh} 個（使用率 {ratio * 100:.0f}%）")
    for g, nm, mt, t, t_new, sf, span, load, pair in thick[:args.limit]:
        why = f"支点間 {span:.0f}mm・荷重 {load:.2f}kg×{DYN:.0f} で安全率 {sf:.0f}"
        if args.md:
            print(f"| {nm} | {mt} | t{t:.1f} | 支点間 {span:.0f} | 板厚過大 | {g:.0f} | "
                  f"t{t_new:.1f} へ（{why}） |")
        else:
            print(f"[板厚] {nm:22s} {mt:9s} t{t:4.1f} → t{t_new:4.1f} "
                  f"→ {g:5.0f}g 削減: {why}")

    if gaps:
        tot_gap = sum(g[0] for g in gaps)
        if args.md:
            print(f"\n### 台帳の手書き質量と実体の食い違い（合計 {tot_gap:+.2f} kg）\n")
            print("| 部品 | 台帳の行 | 申告 [g] | 実体 [g] | 差 [g] |")
            print("|---|---|---|---|---|")
            for diff, grp, label, claim, got, qty in gaps[:args.limit]:
                print(f"| {','.join(grp)} | {label} | {claim * 1000:.0f} | "
                      f"{got * 1000:.0f} | **{diff * 1000:+.0f}** |")
        else:
            print(f"\n--- 台帳の手書き質量と実体の食い違い（合計 {tot_gap:+.2f} kg）---")
            for diff, grp, label, claim, got, qty in gaps[:args.limit]:
                print(f"[台帳] 申告 {claim * 1000:6.0f}g → 実体 {got * 1000:6.0f}g  差 "
                      f"{diff * 1000:+7.0f}g  「{label}」← {','.join(grp)}")

    n = len(trim) + len(solid) + len(thick)
    print(f"\n使用率不足 {len(idle)} 件 / 縁の余り {len(trim)} 件 / 肉抜き無し "
          f"{len(solid)} 件 / 板厚過大 {len(thick)} 件"
          f" — 切る・抜く・薄くするで {tot:.0f}g ({tot / 1000:.2f}kg) の削減余地")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
