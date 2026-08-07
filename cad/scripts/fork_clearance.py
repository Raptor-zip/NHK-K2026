"""フォーク先端の高さ誤差の積み上げ — 1.0mm の隙間に本当に入るのか.

    python scripts/fork_clearance.py [--md]

雑巾の山をすくう櫛歯は、**机の天板（760）と雑巾1枚目の下面の間**に差し込む。
歯は t2.0 で、上面 FORK_Z=763 / 下面 761。つまり

    机の天板から歯の下面まで **1.0 mm しかない**。

ここを外すと
  * 下に外す → 歯の先端が天板に当たる。押せば机が動いてルール 4.1.4a の反則
  * 上に外す → 1枚目の上に乗る。山を崩すか、押して机を動かす

一方でフレームは片持ちで、グラバーを伸ばすとたわむ（張り出しは P.GRAB_STROKE）。
そのたわみが 1.0mm を食い潰さないかを積み上げで確かめる。

積み上げるもの
  1. 骨格のたわみ（scripts/fea_frame.py の LC4 を、レール取付点で読む）
  2. 骨格の傾き × 張り出し長（取付点の回転がそのまま先端の上下になる）
  3. スライドレール自身のたわみ＋ガタ（**実測が必要**。ここでは想定値を置く）
  4. 加工・組立の公差（板の平面度、レール取付面の平行度、歯の曲がり）
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import fea_frame as F  # noqa: E402
import tr_params as P  # noqa: E402

GAP = P.FORK_Z - P.FORK_T - P.DESK_H     # 1.0 mm 天板から歯の下面まで
# ⚠ 先端は RAIL_X0 ではなく **RAIL_X0 − FORK_TINE_EXT**（歯だけ延ばした）。
TIP_X = P.RAIL_X0 - P.FORK_TINE_EXT - P.GRAB_STROKE   # 全開時の歯の先端

# --- 実測が必要な項目（カタログにたわみ量の記載が無いので想定値を置く） ---
RAIL_SAG_ASSUMED = 0.60      # mm 3段引きレールのフルストローク時の下がり（ガタ込み）
RAIL_SAG_NOTE = "実機で要実測（ダイヤルゲージで先端を測る）"
GRAB_N = F.GRABBER_MASS * 9.81   # N 先端にぶら下がる荷重（剛性の逆算に使う）

# --- 加工・組立の公差（RSS で合成する） ---
TOL = [
    ("レール取付面の平行度（CNC 削り出し t6 板）", 0.10),
    ("レール取付ボルトのクリアランス（φ5.5 穴 / M5）", 0.15),
    ("キャリッジ板の平面度", 0.10),
    ("櫛歯の曲がり（SUS304 t2.0 レーザー切り）", 0.20),
    ("歯の付け根の組立（ヒンジピン嵌合 H7/g6）", 0.05),
]


def frame_terms():
    """骨格のたわみと傾きを、レール取付点で読む。"""
    nodes, members = F.build_model()
    out = {}
    for label, kw in (("収納（グラバー全閉）", dict(extra_forces=[], accel=(0, 0, 0))),
                      (f"全開（グラバー {P.GRAB_STROKE:.1f}mm 伸長）",
                       dict(extra_forces=[], accel=(0, 0, 0), grabber_x=TIP_X))):
        u, disp, worst, key = F.solve(nodes, members, **kw)
        nd = int(np.argmin(np.linalg.norm(
            nodes - np.array([P.RAIL_X0, P.RAIL_Y, F.Z_TOP]), axis=1)))
        dz = u[6 * nd + 2]              # 上下変位 [mm]（下向きが負）
        th = u[6 * nd + 4]              # Y 軸まわりの回転 [rad]
        arm = abs(TIP_X - P.RAIL_X0)    # 取付点から先端までの張り出し
        out[label] = (dz, th, th * arm)
    return out


# --- 机の縁に入る瞬間の伸長量（ここが本当の勝負どころ） ---
DESK_X = -1200.0                       # sim/tr_sim.py と同じ机中心
DESK_EDGE_X = DESK_X + P.DESK_D / 2    # -975.0 手前側の縁
APPROACH_X = -500.0                    # 車体の停止位置（sim/tr_sim.py の APPROACH_X）


def sag_at(ext_mm: float, d_frame: float, d_tilt: float, rail: float, tol: float):
    """伸長量 ext での下がり。

    たわみ由来（骨格・レール）は張り出しの**2乗**で効く
    （曲げモーメントが腕に比例し、先端の下がりがさらに腕に比例するため）。
    公差は伸長量に依らない。
    """
    k = (ext_mm / P.GRAB_STROKE) ** 2
    return (d_frame + d_tilt + rail) * k + tol


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    fr = frame_terms()
    dz_c, th_c, arm_c = fr["収納（グラバー全閉）"]
    dz_o, th_o, arm_o = fr[f"全開（グラバー {P.GRAB_STROKE:.1f}mm 伸長）"]

    # 全閉の分は「歯の高さを組立時に合わせ込む」ことで消せる（＝基準に取り込む）。
    # 残るのは全閉→全開で**変化する分**。
    d_frame = abs(dz_o - dz_c)
    d_tilt = abs(arm_o - arm_c)
    tol_rss = float(np.sqrt(sum(v * v for _, v in TOL)))

    items = [
        ("① 骨格のたわみ（レール取付点・全閉→全開の差）", d_frame, "解析"),
        (f"② 骨格の傾き × 張り出し {abs(TIP_X - P.RAIL_X0):.1f}mm", d_tilt, "解析"),
        ("③ スライドレールのたわみ＋ガタ", RAIL_SAG_ASSUMED, RAIL_SAG_NOTE),
        ("④ 加工・組立公差（RSS 合成）", tol_rss, "見積"),
    ]
    worst_sum = sum(v for _, v, _ in items)                       # 最悪の重ね合わせ
    rss_sum = float(np.sqrt(sum(v * v for _, v, _ in items)))     # 統計合成

    allow_rail = GAP - (d_frame + d_tilt + tol_rss)               # レールに許せる下がり

    # 先端の上下剛性（骨格とレールの直列）。天板に着いたときの押付力の見積りに使う。
    global K_FRAME, K_RAIL, K_TIP
    K_FRAME = GRAB_N / (d_frame + d_tilt)
    K_RAIL = GRAB_N / RAIL_SAG_ASSUMED
    K_TIP = 1.0 / (1.0 / K_FRAME + 1.0 / K_RAIL)

    if args.md:
        print(f"隙間は **{GAP:.1f} mm**（机の天板 {P.DESK_H:.0f} → 歯の下面 "
              f"{P.FORK_Z - P.FORK_T:.0f}）。張り出しは {abs(TIP_X - P.RAIL_X0):.1f} mm。\n")
        print("| 誤差源 | 下がり [mm] | 出どころ |")
        print("|---|---|---|")
        for name, v, src in items:
            print(f"| {name} | {v:.2f} | {src} |")
        print(f"| **最悪の重ね合わせ（全部同じ向き）** | **{worst_sum:.2f}** | |")
        print(f"| 統計合成（RSS） | {rss_sum:.2f} | |")
        print()
        verdict_w = "✅ 入る" if worst_sum < GAP else "❌ 入らない"
        verdict_r = "✅ 入る" if rss_sum < GAP else "❌ 入らない"
        print(f"- 最悪ケース {worst_sum:.2f} mm vs 隙間 {GAP:.1f} mm → **{verdict_w}**")
        print(f"- 統計合成 {rss_sum:.2f} mm vs 隙間 {GAP:.1f} mm → **{verdict_r}**")
        print()
        print("### 内訳の読み方")
        print()
        print(f"- 骨格由来（①+②）は **{d_frame + d_tilt:.2f} mm** しかない。"
              "アルミフレームは十分に硬く、ここは問題にならない。")
        print(f"- **効いているのはスライドレール（{RAIL_SAG_ASSUMED:.2f} mm 想定）と公差"
              f"（{tol_rss:.2f} mm）**。つまりこの機構の成否は骨格ではなく"
              "**レールのガタと組立精度**で決まる。")
        print(f"- レールに許せる下がりは **{allow_rail:.2f} mm** まで。"
              "カタログにたわみ量の記載が無いので、**実機で必ず実測すること**"
              "（キャリッジを全開にして先端をダイヤルゲージで測る）。")
        print()
        print("### ただし「全開で 1.36mm」は失敗を意味しない")
        print()
        print("下がりは張り出しの2乗で効くので、**机の縁をくぐる瞬間はまだ伸ばしていない**。"
              "縁さえ越えれば、あとは平らな天板の上なので、下がって歯が天板に触れても"
              "山の下に入っていることは変わらない。危ないのは**縁に当たること**だけ。\n")
        print("| 停止位置の誤差 | 縁での伸長量 | 縁での下がり | 判定 |")
        print("|---|---|---|---|")
        for err in (-40, -20, 0, 20, 40, 60):
            ext = abs(DESK_EDGE_X - (APPROACH_X + err + P.RAIL_X0))
            s = sag_at(ext, d_frame, d_tilt, RAIL_SAG_ASSUMED, tol_rss)
            ok = "✅" if s < GAP and ext <= P.GRAB_STROKE else (
                "❌ ストローク不足" if ext > P.GRAB_STROKE else "❌ 縁に当たる")
            print(f"| {err:+d} mm | {ext:.0f} mm | {s:.2f} mm | {ok} |")
        print()
        ext0 = abs(DESK_EDGE_X - (APPROACH_X + P.RAIL_X0))
        print(f"- 公称の停止位置では縁での伸長は **{ext0:.0f} mm**（ストロークの "
              f"{ext0 / P.GRAB_STROKE * 100:.0f}%）しかなく、下がりは "
              f"**{sag_at(ext0, d_frame, d_tilt, RAIL_SAG_ASSUMED, tol_rss):.2f} mm**。"
              f"隙間 {GAP:.1f} mm に対して余裕がある。")
        print(f"- **歯先のテーパは上面だけに付ける**（先端 {P.FORK_TIP_LEN:.0f}mm を "
              f"t{P.FORK_T} → t{P.FORK_TIP_T} に研ぐ ＝ {P.FORK_TIP_ANGLE:.1f}°）。"
              "フォークリフトのツメと同じ考え方で、下面は水平のまま。"
              "下面を下げると先端が天板より低くなり、**縁に正面衝突して机を押す**。")
        print("- この形にすると、**「縁は高く越え、奥へ行くほど天板に近づく」が"
              "片持ちのたわみからタダで手に入る**。ばねも余分な部品も要らない。")
        print()
        print("### 進みながら歯先が降りていく（これが「すくい」の実体）")
        print()
        print("| 伸長量 | 位置 | 歯先の下面高さ | 状態 |")
        print("|---|---|---|---|")
        base_z = P.FORK_Z - P.FORK_T
        for ext, where in ((0.0, "収納"), (200.0, "机の縁"), (325.0, "山の手前"),
                           (P.GRAB_STROKE, "山の奥（全開）")):
            z = base_z - sag_at(ext, d_frame, d_tilt, RAIL_SAG_ASSUMED, tol_rss)
            if z > P.DESK_H + P.RAG_T:
                st = "❌ 1枚目より上"
            elif z > P.DESK_H:
                st = f"天板 +{z - P.DESK_H:.2f}mm ＝ 1枚目の下"
            else:
                st = f"天板に着地（押付 {(P.DESK_H - z) * K_TIP:.0f}N）"
            print(f"| {ext:.0f} mm | {where} | {z:.2f} | {st} |")
        print()
        print(f"雑巾1枚は t{P.RAG_T:.0f}mm なので、歯先が天板 +0〜{P.RAG_T:.0f}mm にあれば"
              "1枚目の下に入る。上の表のとおり、縁では +0.47mm、山の手前では +0.09mm と、"
              "**進むほど確実に「下」へ入る側に寄っていく**。")
        print()
        print("### 天板に着いたときに机を押さないか")
        print()
        print(f"- 先端の剛性は、骨格 {K_FRAME:.0f} N/mm（{GRAB_N:.1f}N で "
              f"{d_frame + d_tilt:.2f}mm たわむ）と レール {K_RAIL:.0f} N/mm の直列で "
              f"**{K_TIP:.0f} N/mm**。")
        print(f"- 全開で {worst_sum - GAP:.2f}mm ぶん食い込むので、天板が受ける力は "
              f"**{(worst_sum - GAP) * K_TIP:.0f} N**、摩擦 0.2 で水平力 "
              f"**{(worst_sum - GAP) * K_TIP * 0.2:.1f} N**。")
        print("- 机（約7kg・床の摩擦 0.3）が滑り出すのは約 21 N。"
              f"{(worst_sum - GAP) * K_TIP * 0.2:.1f} N はその "
              f"{(worst_sum - GAP) * K_TIP * 0.2 / 21 * 100:.0f}% で、"
              "ルール 4.1.4a の「机を動かした」には至らない。")
        print("- なおレールの下がりのうち**ガタの分は力を出さない**（隙間が詰まるだけ）ので、"
              "この値は上限側の見積り。")
    else:
        print(f"隙間 {GAP:.2f} mm / 張り出し {abs(TIP_X - P.RAIL_X0):.1f} mm")
        for name, v, src in items:
            print(f"  {name:<44} {v:5.2f} mm  ({src})")
        print(f"  {'最悪の重ね合わせ':<44} {worst_sum:5.2f} mm  "
              f"→ {'OK' if worst_sum < GAP else 'NG'}")
        print(f"  {'統計合成(RSS)':<44} {rss_sum:5.2f} mm  "
              f"→ {'OK' if rss_sum < GAP else 'NG'}")
        print(f"  レールに許せる下がり {allow_rail:.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
