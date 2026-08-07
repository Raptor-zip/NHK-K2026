"""電流バジェットの検証（規定 3.2.5：駆動系の遮断素子合計 30A 以下）.

    python scripts/power_budget.py [--md]

ミッション（後退接近→装填→前進→3射）の時間軸で各モーターの要求電流を積み上げ、
30A を超えるかを見る。超える場合は優先度つきの配分器
（射出ローラー定速維持 > 砲塔 > 駆動 > 装填）でクランプし、
「クランプによってどれだけ遅くなるか」を数値で出す。

電流換算
  M3508 P19    : Kt = 0.3 N·m/A（出力軸換算・データシート）
  M3508 GBレス : Kt = 0.3 / 19.2 = 0.0156 N·m/A（素モーター）
  M2006 P36    : 定格 1.0 N·m / 3A → Kt = 0.333 N·m/A
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

MM = 0.001
LIMIT_A = P.RULE_BREAKER_TOTAL_MAX      # 30 A
KT_M3508 = 0.30                          # N·m/A（P19 出力軸）
KT_M3508_RAW = 0.30 / P.M3508_RATIO      # N·m/A（ギアボックスレス）
KT_M2006 = 1.0 / 3.0                     # N·m/A
I_QUIESCENT = 1.2                        # A 制御系・センサー（絶縁DCDC＝規定上は対象外だが計上）

MASS = 35.0                              # kg
WHEEL_R = P.WHEEL_DIA / 2 * MM
MU_ROLL = 0.03                           # 転がり抵抗係数（メカナム＋ロンリウム）

# 射出ローラー
J_ROLLER = 6.9e-4                        # kg·m²（5個＋中空軸）
OMEGA_TARGET = 6.5 / (P.ROLLER_DIA / 2 * MM)     # rad/s（周速6.5m/s）
                                                # 布の揚力補正で必要初速が 9→6.5 に下がった
                                                # （scripts/target_ev.py）
I_SPINUP_CAP = 10.0                      # A/軸 スピンアップ時の上限（配分器で決める）
TAU_DRAG = 0.02                          # N·m 軸受＋風損

# 優先度（小さいほど優先）
PRIORITY = {"射出ローラー": 0, "砲塔ヨー": 1, "仰角": 1, "駆動": 2, "装填": 3}


def drive_current(accel_mps2: float, speed: float) -> float:
    """4輪ぶんの合計電流。"""
    f_acc = MASS * accel_mps2
    f_roll = MU_ROLL * MASS * 9.81
    tau_per_wheel = (f_acc + f_roll) * WHEEL_R / 4.0
    return 4.0 * abs(tau_per_wheel) / KT_M3508


def spinup_profile(cap_a: float):
    """上限電流 cap_a でローラーを目標周速まで上げるのに要る時間。"""
    tau = cap_a * KT_M3508_RAW
    alpha = tau / J_ROLLER
    return OMEGA_TARGET / alpha, tau


def timeline():
    """(時刻, フェーズ, {系統: 電流A}) のリスト。"""
    t_spin, _ = spinup_profile(I_SPINUP_CAP)
    rows = [
        ("0.0-1.5 待機（砲塔照準のみ）", dict(砲塔ヨー=1.0, 仰角=0.6, 駆動=0.0, 装填=0.0,
                                            射出ローラー=0.0)),
        ("1.5-4.5 補充スポットへ後退（0.8m/s・1.5m/s²）",
         dict(駆動=drive_current(1.5, 0.8), 砲塔ヨー=0.5, 仰角=0.3, 装填=0.0, 射出ローラー=0.0)),
        ("4.5-7.5 フォーク挿入＋上押さえ",
         dict(駆動=0.0, 砲塔ヨー=0.5, 仰角=0.3, 装填=4.0, 射出ローラー=0.0)),
        ("7.5-11.6 引込＋カム傾斜（機体は停止）",
         dict(駆動=0.0, 砲塔ヨー=0.5, 仰角=0.3, 装填=5.0, 射出ローラー=0.0)),
        (f"[NG運用] 前進しながらスピンアップ（{t_spin:.1f}s）",
         dict(駆動=drive_current(1.5, 0.8), 砲塔ヨー=1.5, 仰角=0.8, 装填=2.0,
              射出ローラー=2 * I_SPINUP_CAP)),
        (f"[推奨運用] 引込中（停止）にスピンアップ（{t_spin:.1f}s）",
         dict(駆動=0.0, 砲塔ヨー=0.5, 仰角=0.3, 装填=5.0,
              射出ローラー=2 * I_SPINUP_CAP)),
        ("射出中（定速維持＋1発ごとの回復）",
         dict(駆動=drive_current(0.3, 0.5), 砲塔ヨー=1.5, 仰角=0.8, 装填=2.0,
              射出ローラー=2 * (TAU_DRAG + 0.029) / KT_M3508_RAW)),
    ]
    return rows, t_spin


def allocate(demand: dict[str, float], limit: float) -> dict[str, float]:
    """優先度順に上限まで割り当てる。"""
    out = {k: 0.0 for k in demand}
    remain = limit - I_QUIESCENT
    for key in sorted(demand, key=lambda k: PRIORITY[k]):
        take = min(demand[key], max(remain, 0.0))
        out[key] = take
        remain -= take
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    rows, t_spin = timeline()

    printed = []
    for name, demand in rows:
        total = sum(demand.values()) + I_QUIESCENT
        alloc = allocate(demand, LIMIT_A)
        atotal = sum(alloc.values()) + I_QUIESCENT
        cut = {k: demand[k] - alloc[k] for k in demand if demand[k] - alloc[k] > 0.05}
        printed.append((name, demand, total, atotal, cut))

    if args.md:
        print("| 局面 | 射出 | 砲塔 | 駆動 | 装填 | 制御系 | 要求合計 | 配分後 | 判定 |")
        print("|---|---|---|---|---|---|---|---|---|")
        for name, d, total, atotal, cut in printed:
            print(f"| {name} | {d['射出ローラー']:.1f} | {d['砲塔ヨー'] + d['仰角']:.1f} | "
                  f"{d['駆動']:.1f} | {d['装填']:.1f} | {I_QUIESCENT:.1f} | "
                  f"**{total:.1f}** | {atotal:.1f} | "
                  f"{'そのまま' if not cut else '要クランプ: ' + ', '.join(cut)} |")
        print()
        print(f"- 上限 **{LIMIT_A:.0f} A**（規定 3.2.5）。制御系は絶縁DCDCで別系統だが安全側で計上。")
        print(f"- ローラーのスピンアップは 1軸 {I_SPINUP_CAP:.0f}A に制限して **{t_spin:.2f} 秒**。")
        print("  無制限（1軸20A）なら 0.74秒だが合計40Aで規定違反になる。")
        print("- **運用上の結論**: スピンアップを「前進しながら」やると 36A で規定違反になり、"
              "配分器が駆動を 10.5A→6.5A に絞る（加速 1.5→0.82 m/s²）。\n"
              "  一方 **引込中（機体停止）にスピンアップすれば 27A で収まり、クランプ不要**。\n"
              "  → ミッションシーケンスを「引込と同時にローラー始動」に変更した（sim/tr_sim.py 反映済み）。")
    else:
        for name, d, total, atotal, cut in printed:
            flag = "OK " if total <= LIMIT_A else "CLAMP"
            print(f"[{flag}] {name}")
            print(f"        射出{d['射出ローラー']:5.1f} 砲塔{d['砲塔ヨー'] + d['仰角']:4.1f} "
                  f"駆動{d['駆動']:5.1f} 装填{d['装填']:4.1f} 制御{I_QUIESCENT:.1f} "
                  f"→ 要求{total:5.1f}A / 配分後{atotal:5.1f}A"
                  + (f"  絞る: {list(cut)}" if cut else ""))
        print(f"\n  スピンアップ {t_spin:.2f} s（1軸{I_SPINUP_CAP:.0f}A制限）  "
              f"無制限なら {spinup_profile(20.0)[0]:.2f} s だが合計40Aで違反")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
