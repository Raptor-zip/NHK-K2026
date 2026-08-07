"""締結部の成立性チェック（ねじ・Tナット・重力下の静的つり合い）.

    python scripts/fasteners.py [--md]

「図面は描けたが、そのねじで本当に持つのか」を数値で確かめる。
各締結部について 荷重 → ボルトのせん断/引張 → 安全率、および
アルミフレーム用ポストナットの**滑り**（実務上の律速）を見る。

前提と出典
  * ボルトは A2-70 ステンレス（引張強さ 700MPa、降伏 450MPa）を標準とする。
    有効断面積: M3=5.03 / M4=8.78 / M5=14.2 / M6=20.1 mm²
  * 締付軸力 F = T /(K·d)、K=0.20（潤滑なし）
  * アルミフレーム用ポストナットの摩擦保持力 = μ·F、μ=0.15（アルマイト面）
  * MISUMI 5シリーズ ブラケット(HBLFSN5等)の許容荷重は 1個あたり約 500N とする
    （カタログの目安値。実機では二重使いを基本とする）
  * 安全率は「静荷重 2.0 以上」「衝撃・繰返しがかかる箇所は 3.0 以上」を要求する
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

MM = 0.001
G = 9.81
AS = {"M3": 5.03, "M4": 8.78, "M5": 14.2, "M6": 20.1}     # 有効断面積 mm²
DIA = {"M3": 3.0, "M4": 4.0, "M5": 5.0, "M6": 6.0}
TORQUE = {"M3": 1.2, "M4": 2.8, "M5": 5.0, "M6": 8.5}      # N·m 推奨締付
SIGMA_Y = 450.0          # MPa A2-70 の降伏
TAU_ALLOW = 0.6 * SIGMA_Y / 2.0     # せん断許容（せん断降伏0.6σy / 安全率2）
K_TIGHTEN = 0.20
MU_SLIP = 0.15
BRACKET_ALLOW_N = 500.0

ACCEL = 2.0              # m/s² 設計加速度（急制動・被弾回避）


def preload(size: str) -> float:
    return TORQUE[size] / (K_TIGHTEN * DIA[size] * MM)


def shear_capacity(size: str, n: int) -> float:
    return n * AS[size] * TAU_ALLOW


def tensile_capacity(size: str, n: int) -> float:
    return n * AS[size] * SIGMA_Y / 2.0


def slip_capacity(size: str, n: int) -> float:
    return n * MU_SLIP * preload(size)


def joints():
    """(名称, 荷重の説明, 荷重N, ボルト, 本数, 判定方式, 要求安全率)"""
    m_turret = 4.94
    m_bucket = 1.50
    m_grab = 2.80 + 0.35 * 10 * P.RAG_MASS      # キャリッジ + 山
    out = []

    # J1 モーター取付（M3508 ↔ マウント板）: 定格トルクを PCD35 で受ける
    f_j1 = P.M3508_RATED_TORQUE / (P.M3508_BOLT_PCD / 2 * MM)
    out.append(("J1 M3508 ↔ マウント板", "定格3.0N·m を PCD35 の4本で受ける",
                f_j1, "M4", 4, "shear", 3.0))

    # J2 マウント板 ↔ ベースフレーム: 車輪1個の接地反力 + 加速反力
    f_j2 = (35.0 / 4) * G + (35.0 / 4) * ACCEL
    out.append(("J2 マウント板 ↔ フレーム", "1輪分の接地荷重＋加速反力",
                f_j2, "M5", 4, "slip", 2.0))

    # J3 ハブアダプタ ↔ 車輪
    f_j3 = P.M3508_RATED_TORQUE / (34.0 / 2 * MM)
    out.append(("J3 ハブアダプタ ↔ 車輪", "定格3.0N·m を PCD34 の4本で受ける",
                f_j3, "M4", 4, "shear", 3.0))

    # J4 側面トラス柱脚: 上部質量の転倒モーメントを柱脚のブラケットで受ける
    m_upper = m_turret + m_bucket + 1.9 + 0.9      # 砲塔+バケツ+ホッパー+斜路
    h_cg = 0.95
    moment = m_upper * ACCEL * h_cg                 # N·m
    f_j4 = moment / (2 * P.SIDE_FRAME_Y * MM)       # 左右柱で偶力として受ける
    out.append(("J4 側面トラス柱脚 ↔ ベース桁", "上部質量の転倒モーメントを偶力で受ける",
                f_j4, "M5", 4, "bracket", 2.0))

    # J5 砲塔台座リング ↔ 横梁
    # 射出反力 F = m·v²/L（加速時間は雑巾がニップを通る時間 L/v で決まる）。
    # 仰角最大 70° のとき鉛直成分が最大になる。scripts/fea_frame.py の LC5 と同じ値。
    shot_n = P.RAG_MASS * 6.5 ** 2 / (P.RAG_X * MM)
    f_j5 = m_turret * G + shot_n * math.sin(math.radians(P.PITCH_MAX))
    out.append(("J5 砲塔リング ↔ 台座横梁", "砲塔自重＋射出反力",
                f_j5, "M5", 8, "shear", 3.0))

    # J6 スライドレール ↔ 側面上桁（MISUMI 指定は M4 トラス小ねじ）
    # 本数は**公式 CAD の実測**（P.RAIL_HOLES = 5箇所）。以前は 6 と概算していた。
    f_j6 = m_grab * G * 2.0            # 全開時の片持ちで先端側レールに集中
    out.append(("J6 スライドレール ↔ 上桁", "グラバー全開時の片持ち荷重（2倍見込み）",
                f_j6, "M4", len(P.RAIL_HOLES), "slip", 2.0))

    # J7 櫛歯 ↔ 根元バー
    f_j7 = (10 * P.RAG_MASS + 0.3) * G / P.FORK_TINES * 3.0   # 1本あたり・動的3倍
    out.append(("J7 櫛歯 ↔ 根元バー", "山の荷重を櫛歯5本で分担（動的3倍）",
                f_j7, "M4", 2, "shear", 3.0))

    # J8 マスト頭部 ↔ 主柱
    f_j8 = m_bucket * G + m_bucket * ACCEL
    out.append(("J8 マスト頭部 ↔ 主柱", "バケツ自重＋加速反力",
                f_j8, "M5", 4, "bracket", 2.0))

    # J9 移動バケツ ↔ 受け板（底面 4-M5 皿・2L目盛りより下＝規定 3.2.3c）
    # ⚠ 2026-08-07 まで「バンド固定」と書いてあったが、そのバンドは図に
    #   無かった（質量台帳に 150g が載っているだけ）。いまは M5 4 本で
    #   バケツの底を貫いて留める。
    f_j9 = (m_bucket + 0.5) * math.hypot(G, ACCEL)
    out.append(("J9 移動バケツ ↔ 受け板", "バケツ＋雑巾5枚を合成加速度で保持",
                f_j9, "M5", 4, "tensile", 3.0))

    # J10 仰角ウォームホイール ↔ 仰角軸
    # 受けるトルクは「モーター側」と「自重側」の大きい方。
    #   モーター側: M2006 定格 1.0 N·m × 40:1 × 効率0.7 = 28.0 N·m  ← 支配的
    #   自重側    : 仰角ユニット 4.94kg、重心は**仰角軸から 36.7mm** しか離れていない
    #               （局所座標 X=-34.0 Z=+13.9）ので 1.78 N·m にとどまる
    # 自重が効かないのは「仰角軸とニップ中心を一致させる」設計が効いているため。
    # ローラー駆動を上下にオフセットしても重心は軸の近くに留まる（±85mm が相殺）。
    # ⚠ 効率 0.7 は**楽観的**。進み角 3.576°・摩擦角 4.868° のセルフロック
    #   ウォームの正転効率は η = tanγ/tan(γ+ρ') = 0.42 しかない
    #   （tr_params の「仰角ウォーム減速機」の節）。ここは 0.7 のまま
    #   ＝ 28N·m で評価する。実際は 16.8N·m なので安全側。
    # ⚠ ボルトが受ける力の腕は**歯車の基準円ではなくボルト円**。ここは
    #   以前 PITCH_WHEEL_DIA（当時 80）の半分で割っていたが、ボルトは
    #   PCD28 に 6 本並ぶ（tr_assembly の pitch_worm_wheel の note）。
    #   腕を 40mm と見ていたので、実際の 14mm に対して分担荷重を
    #   **1/2.9 に小さく見積もっていた**。
    j10_pcd = 28.0
    tq_worm_motor = P.M2006_RATED_TORQUE * P.PITCH_WORM_RATIO * 0.7
    f_j10 = tq_worm_motor / (j10_pcd / 2 * MM)
    out.append(("J10 ウォームホイール ↔ 仰角軸",
                f"減速後トルク28N·m を PCD{j10_pcd:.0f} のボルト円で受ける",
                f_j10, "M4", 6, "shear", 3.0))

    # --- ローラー駆動をベルトにしたぶんの締結（J11〜J13） ---
    # モーターを軸の延長線上に置けず HTD5M でオフセットしたので、
    # トルクの伝達経路が「軸直結」から「プーリ2個＋ベルト」に変わった。
    # スピンアップ時の全トルクがプーリの締結を通る。
    #   スピンアップ電流 10A/軸 × Kt(GBレス) = トルク
    kt_raw = 0.30 / (3591 / 187)          # N·m/A（ギアボックスレス）
    tq_spin = 10.0 * kt_raw               # 1軸あたりのスピンアップトルク
    r_pulley = 38.2 / 2 * MM              # HTD5M 24T の PCD/2 [m]

    # J11 プーリ ↔ ローラー軸（φ12 中空軸にキーレスで留める）
    f_j11 = tq_spin / r_pulley
    out.append(("J11 プーリ ↔ ローラー軸", "スピンアップトルクを φ38.2 プーリで受ける",
                f_j11, "M4", 2, "shear", 3.0))

    # J12 プーリ ↔ M3508 出力軸（φ10）
    out.append(("J12 プーリ ↔ M3508 出力軸", "同トルク。軸径 φ10 のいもねじ2本",
                f_j11, "M4", 2, "shear", 3.0))

    # J13 モーターブラケット ↔ 仰角側板
    # ベルト張力（トルク/半径）＋モーター自重を、仰角の全域で保持する
    f_belt = tq_spin / r_pulley
    f_j13 = math.hypot(f_belt * 2.0, 0.290 * G)   # 張り側+緩み側 ≒ 2倍
    out.append(("J13 モーターブラケット ↔ 仰角側板", "ベルト張力＋モーター自重",
                f_j13, "M4", 4, "shear", 3.0))

    return out


def stability():
    """重力下の静的つり合い: 重心位置と転倒余裕。"""
    import fea_frame as F   # 集中質量表を共用する

    # 押出材は「ベース(z=125)」と「トラス・マスト(z=620)」に分けて計上する
    items = list(F.POINT_MASSES) + [
        ("押出フレーム(ベース)", 5.32 * P.EXT_MASS_PER_M, (0.0, 0.0, 125.0)),
        ("押出フレーム(トラス・マスト)", 10.77 * P.EXT_MASS_PER_M, (-30.0, 0.0, 620.0)),
    ]
    # 質量台帳の総質量に合わせて、差分を電装デッキ上に足す（集計の取りこぼし補正）
    import tr_assembly as A_
    import tr_lib as L_
    A_.build(P.POSE_MATCH)
    ledger_total = L_.LEDGER.total_kg
    listed = sum(x[1] for x in items) + F.GRABBER_MASS + 10 * P.RAG_MASS
    if ledger_total - listed > 0:
        items.append(("集計差分", ledger_total - listed, (0.0, 0.0, 200.0)))

    def cg(grab_x: float):
        pts = items + [("グラバー+山", F.GRABBER_MASS + 10 * P.RAG_MASS, (grab_x, 0.0, 770.0))]
        m = sum(x[1] for x in pts)
        cx = sum(x[1] * x[2][0] for x in pts) / m
        cy = sum(x[1] * x[2][1] for x in pts) / m
        cz = sum(x[1] * x[2][2] for x in pts) / m
        return m, cx, cy, cz

    out = []
    for label, gx in (("グラバー収納", -145.0), ("グラバー全開", -570.0)):
        m, cx, cy, cz = cg(gx)
        edge_r = -P.WHEELBASE_X / 2      # 後方の転倒稜線
        edge_f = P.WHEELBASE_X / 2
        edge_y = P.TRACK_Y / 2
        # 転倒に至る加速度: m*a*h = m*g*d  → a = g*d/h
        a_back = G * abs(cx - edge_r) / cz
        a_fwd = G * abs(edge_f - cx) / cz
        a_side = G * abs(edge_y - abs(cy)) / cz
        out.append((label, m, cx, cy, cz, a_back, a_fwd, a_side))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    rows = []
    for name, why, load, size, n, mode, need in joints():
        if mode == "shear":
            cap = shear_capacity(size, n)
            capname = "せん断許容"
        elif mode == "tensile":
            cap = tensile_capacity(size, n)
            capname = "引張許容"
        elif mode == "slip":
            cap = slip_capacity(size, n)
            capname = "摩擦保持"
        else:
            cap = n * BRACKET_ALLOW_N
            capname = "ブラケット許容"
        sf = cap / load if load > 0 else 999
        rows.append((name, why, load, f"{size}×{n}", capname, cap, sf, need,
                     "OK" if sf >= need else "NG"))

    if args.md:
        print("| 締結部 | 荷重の内容 | 荷重 [N] | ボルト | 判定基準 | 許容 [N] | 安全率 | 要求 | 判定 |")
        print("|---|---|---|---|---|---|---|---|---|")
        for r in rows:
            print(f"| {r[0]} | {r[1]} | {r[2]:.0f} | {r[3]} | {r[4]} | {r[5]:.0f} | "
                  f"**{r[6]:.1f}** | {r[7]:.1f} | {r[8]} |")
        print()
        print("### 締付トルク")
        print("| ねじ | 推奨トルク [N·m] | 得られる軸力 [N] | 摩擦保持（1本, μ=0.15） |")
        print("|---|---|---|---|")
        for sz in ("M3", "M4", "M5", "M6"):
            print(f"| {sz} | {TORQUE[sz]:.1f} | {preload(sz):.0f} | {MU_SLIP * preload(sz):.0f} |")
        print()
        print("> アルミフレームの締結は**ボルトが折れる前にポストナットが溝を滑る**。"
              "上表の「摩擦保持」が実質の許容値であり、規定トルクでの締付管理が安全の前提になる。"
              "組立時にトルクレンチで管理し、試合前点検で全数を再確認すること。")
    else:
        for r in rows:
            print(f"[{r[8]:2s}] {r[0]:<28} 荷重{r[2]:7.0f}N  {r[3]:<6} {r[4]}{r[5]:7.0f}N  "
                  f"安全率 {r[6]:5.1f} (要求{r[7]:.1f})")
        print()
        for sz in ("M3", "M4", "M5", "M6"):
            print(f"  {sz}: {TORQUE[sz]:.1f}N·m → 軸力{preload(sz):.0f}N → 摩擦保持"
                  f"{MU_SLIP * preload(sz):.0f}N/本")
    # --- 重力下の静的つり合い ---
    st = stability()
    if args.md:
        print()
        print("### 重力下の静的つり合い（転倒余裕）")
        print()
        print("| 状態 | 総質量 [kg] | 重心 X [mm] | 重心 Z [mm] | 後方転倒 [m/s²] | "
              "前方転倒 [m/s²] | 横転倒 [m/s²] |")
        print("|---|---|---|---|---|---|---|")
        for label, m, cx, cy, cz, ab, af, asd in st:
            print(f"| {label} | {m:.1f} | {cx:+.0f} | {cz:.0f} | {ab:.1f} | {af:.1f} | {asd:.1f} |")
        print()
        print(f"- 設計加速度は **{ACCEL:.1f} m/s²**。最小の転倒加速度に対する余裕を確認する。")
        print("- 支持多角形は車輪接地点 600×600mm。重心が中心から後ろへ寄るほど後方転倒に近づく。")
        print("- **グラバー全開のまま急制動しない**こと（ソフトで相互ロック）。"
              "全開は机の前で停止しているときだけ許可する。")
    else:
        print()
        for label, m, cx, cy, cz, ab, af, asd in st:
            print(f"  {label:<12} 質量{m:5.1f}kg 重心({cx:+6.0f},{cy:+5.0f},{cz:5.0f}) "
                  f"転倒加速度 後{ab:5.1f} 前{af:5.1f} 横{asd:5.1f} m/s²")

    ng = [r for r in rows if r[8] == "NG"]
    return 1 if ng else 0


if __name__ == "__main__":
    raise SystemExit(main())
