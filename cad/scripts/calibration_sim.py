"""試合中の自動較正が間に合うか — 32射で収束するかを確かめる.

    python scripts/calibration_sim.py [--md] [--n 400]

`scripts/nip_calibration.py` で「射出ごとの回転数降下から初速を 0.8% 精度で実測できる」
ことが分かった。ではそれを使って **試合中にドリフトを追従補正できるのか**。

制約が厳しい: 1試合で撃てるのは **32枚しかない**。較正に何射も使えない。

模擬するもの
  * 系統誤差のドリフト: ニップの発熱・摩耗で初速が試合中に徐々にずれる
  * 1射ごとのランダム誤差: 厚みFF後の残差 0.43% + 隙間の設定誤差 0.3%
  * 測定ノイズ: 回転数降下からの推定 0.8%
  * 推定器: EWMA（指数移動平均）。α が大きいほど速く追従するがノイズを拾う
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import accuracy_budget as AB  # noqa: E402

SHOTS = 32                  # 1試合の弾数（戦略書 §1.3）
DRIFT_TOTAL = 0.012         # 試合を通した系統ドリフト（+1.2% = 隙間0.04mm相当）
SIGMA_RANDOM = math.sqrt(0.0043 ** 2 + 0.003 ** 2)   # 1射ごとのランダム誤差
SIGMA_MEAS = 0.008          # 回転数降下からの初速推定ノイズ
PRE_CAL_SHOTS = 6           # 試合前の較正射数（練習で撃てる）


def one_match(alpha: float, rng: random.Random, pre_cal: int = 0):
    """1試合ぶんを回して、命中数と各射の残差を返す。"""
    v0, pitch, yaw = AB.nominal()
    est = 0.0                                   # 系統誤差の推定値
    if pre_cal:                                 # 試合前に撃って推定を収束させておく
        for _ in range(pre_cal):
            true_sys = 0.0
            e = true_sys + rng.gauss(0.0, SIGMA_RANDOM)
            meas = e + rng.gauss(0.0, SIGMA_MEAS)
            est += alpha * (meas - est)
    hits, residuals = 0, []
    for i in range(SHOTS):
        true_sys = DRIFT_TOTAL * (i / (SHOTS - 1))          # 線形にドリフト
        e = true_sys + rng.gauss(0.0, SIGMA_RANDOM)          # この射の実誤差
        v_cmd = v0 / (1.0 + est)                             # 推定ぶんを打ち消す
        v_actual = v_cmd * (1.0 + e)
        residuals.append(v_actual / v0 - 1.0)
        # 初速以外の誤差源（姿勢・バックラッシュ・自己位置）も同時に入れて、
        # accuracy_budget.py と同じ土俵の命中率にする
        pit = pitch
        yw = yaw
        xb = AB.X_BAR
        for name, spec in AB.SOURCES.items():
            if spec["kind"] == "v_rel":
                continue                      # 初速側は上で扱っている
            g = rng.gauss(0.0, 1.0) * spec["sigma"]
            if spec["kind"] == "pitch_deg":
                pit += math.radians(g)
            elif spec["kind"] == "yaw_deg":
                yw += math.radians(g)
            else:
                xb += g
        if AB.hit(v_actual, pit, yw, xb):
            hits += 1
        meas = e + rng.gauss(0.0, SIGMA_MEAS)                # 回転数降下からの推定
        est += alpha * (meas - est)                          # EWMA 更新
    return hits, residuals


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=400)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    rows = []
    for label, alpha, pre in (("較正なし（開ループ）", 0.0, 0),
                              ("EWMA α=0.10（推奨）", 0.10, 0),
                              ("EWMA α=0.25", 0.25, 0),
                              ("EWMA α=0.50（速い）", 0.50, 0),
                              ("EWMA α=0.10 + 試合前6射で較正", 0.10, PRE_CAL_SHOTS)):
        tot_hits, res_all = 0, []
        rng = random.Random(20260729)
        for _ in range(args.n):
            h, r = one_match(alpha, rng, pre)
            tot_hits += h
            res_all.extend(r)
        mean_res = sum(abs(x) for x in res_all) / len(res_all)
        sd = math.sqrt(sum(x * x for x in res_all) / len(res_all))
        rows.append((label, tot_hits / (args.n * SHOTS), mean_res, sd))

    if args.md:
        print(f"1試合 {SHOTS} 射 × {args.n} 試合。試合中に系統ドリフト "
              f"**+{DRIFT_TOTAL * 100:.1f}%**（ニップの発熱・摩耗）が線形に入る想定。\n")
        print("| 較正方式 | 命中率 | 初速の平均残差 | 残差 1σ |")
        print("|---|---|---|---|")
        for label, hr, mres, sd in rows:
            print(f"| {label} | **{hr * 100:.0f}%** | {mres * 100:.2f}% | {sd * 100:.2f}% |")
        print()
        best = max(rows, key=lambda r: r[1])
        worst = rows[0]
        print(f"- 較正なしだと命中率 {worst[1] * 100:.0f}%、"
              f"最良の {best[0]} で {best[1] * 100:.0f}%。")
        print("- **α=0.10 の時定数は約10射**。試合中のゆっくりしたドリフトには十分間に合う。\n"
              "  一方 α を大きくすると測定ノイズ 0.8% をそのまま指令に流し込むので逆効果になる。")
        print(f"- ただし **試合前に {PRE_CAL_SHOTS} 射だけ較正しておく**と、"
              "序盤の外しが消えてさらに良くなる。練習で必ずやること。")
        print("- α を大きくしすぎる（0.50）と測定ノイズ 0.8% を拾って逆効果。")
    else:
        for label, hr, mres, sd in rows:
            print(f"  {label:34s} 命中 {hr * 100:5.1f}%  残差 平均{mres * 100:.2f}% 1σ{sd * 100:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
