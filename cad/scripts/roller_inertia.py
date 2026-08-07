"""射出ローラーの慣性設計 — 軽くしてよいのか、径は変えてよいのか.

    python scripts/roller_inertia.py [--md]

布の揚力補正で必要初速が 9 → 6.5 m/s に下がった（scripts/target_ev.py）。
「では慣性に余裕ができたのでローラーを軽くできるか？」を確かめる。

効いてくる3つのトレードオフ
  1. スピンアップ時間   — 軽いほど速い
  2. **1射あたりの回転数降下** — 軽いほど大きい。
     降下は射出の**最中**に起きるので、雑巾の先端と後端で射出速度が変わる。
     これは初速のばらつきそのもので、命中率に直結する（accuracy_budget.py）
  3. 初速の実測精度   — 降下が大きいほど測りやすい（nip_calibration.py）

重要な性質（式を解くと出る）
    Δω/ω = ½ · m_雑巾 · r² / J = ½ · m_雑巾 / m_リング
つまり降下率を決めるのは **リングの実効質量だけ**で、**径には依存しない**。
「径を大きくして軽くする」は成立しない（J = m r² なので、同じ J でも r を大きくすると
降下率は r² で悪化する）。逆に **径を小さくすると同じ質量・同じ降下率のままスピンアップが速くなる**。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

M_RAG = P.RAG_MASS
V_MUZZLE = 6.5                # m/s 常用初速の上限
I_SPINUP = 10.0               # A/軸（電流バジェットの割当）
KT_RAW = 0.30 / (3591 / 187)  # N·m/A ギアボックスレス
RPM_RES = 1.0                 # C620 の回転数分解能
TARGET_DROP = 0.035           # 許容する 1射あたりの速度降下（＝射出中の初速変動）


def evaluate(dia_mm: float, m_ring_kg: float):
    """外径と「リング部の実効質量（2軸ぶん）」から各指標を出す。"""
    r = dia_mm / 2 * 1e-3
    j = m_ring_kg * r * r                 # 薄肉リング近似 J = m r²（2軸ぶん）
    omega = V_MUZZLE / r
    drop_rel = 0.5 * M_RAG / m_ring_kg    # Δω/ω ≈ ½·(m_rag/m_ring)
    d_omega = drop_rel * omega
    spinup = omega / (2 * I_SPINUP * KT_RAW / j)
    res_rad = RPM_RES * 2 * math.pi / 60
    meas_rel = (res_rad / d_omega) / 2 if d_omega > 0 else 9.9
    return dict(r=r, j=j, omega=omega, drop=drop_rel, spinup=spinup, meas=meas_rel)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    # 現行: φ90 / ローラー 78g×10 + 中空軸 105g×2 のうち、外周リングの実効質量
    cur_ring = 0.68            # kg（J=1.38e-3 / r²=0.002025 から逆算した等価質量）
    cases = [
        ("現行 φ90 / リング 0.68kg", 90.0, cur_ring),
        ("φ90・リング半分に軽量化", 90.0, cur_ring / 2),
        ("φ90・リング 2/3 に軽量化", 90.0, cur_ring * 2 / 3),
        ("φ110・リング質量そのまま", 110.0, cur_ring),
        ("φ70・リング質量そのまま", 70.0, cur_ring),
        ("φ70・リング 0.55kg（軽量化 -130g）", 70.0, 0.55),
    ]
    rows = []
    for name, dia, m in cases:
        e = evaluate(dia, m)
        dm = (m - cur_ring) * 1.0
        rows.append((name, dia, m, e, dm))

    if args.md:
        print(f"常用初速 {V_MUZZLE} m/s、雑巾 {M_RAG * 1000:.0f}g、"
              f"スピンアップ電流 {I_SPINUP:.0f}A/軸。\n")
        print("| 案 | 外径 | 実効質量(2軸) | 回転数 | **1射の速度降下** | スピンアップ | 初速測定精度 | 質量差 |")
        print("|---|---|---|---|---|---|---|---|")
        for name, dia, m, e, dm in rows:
            flag = "" if e["drop"] <= TARGET_DROP * 1.05 else " ⚠"
            print(f"| {name} | φ{dia:.0f} | {m:.2f} kg | {e['omega']:.0f} rad/s | "
                  f"**{e['drop'] * 100:.1f}%**{flag} | {e['spinup']:.2f} s | "
                  f"{e['meas'] * 100:.2f}% | {dm * 1000:+.0f} g |")
        print()
        print(f"- 許容する 1射の速度降下は **{TARGET_DROP * 100:.1f}%**。"
              "降下は射出の最中に起きるので、雑巾の先端と後端で初速が変わる＝初速ばらつきそのもの。")
        print("- **軽量化は命中率を直接落とす。** リングを半分に軽くすると降下が 3.5%→7.1% に倍増し、"
              "せっかく 0.62% まで詰めた初速ばらつきが台無しになる。**ローラーは軽量化してはいけない。**")
        print("- 径を変えても降下率は変わらない（リング質量だけで決まる）。"
              "**径を小さくするとスピンアップだけ速くなる** が、ニップの接触弧が短くなって"
              "把持が不安定になるので φ90 を維持する。")
    else:
        for name, dia, m, e, dm in rows:
            print(f"  {name:30s} φ{dia:.0f} m={m:.2f}kg ω={e['omega']:5.0f} "
                  f"降下{e['drop'] * 100:4.1f}% spinup{e['spinup']:.2f}s "
                  f"測定{e['meas'] * 100:.2f}% Δm{dm * 1000:+5.0f}g")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
