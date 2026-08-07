"""目標ごとの命中率と期待値 — 戦略書 §2.1 の期待値表を物理ベースで作り直す.

    python scripts/target_ev.py [--md] [--n 20000]

`accuracy_budget.py` と同じ誤差モデル（対策込み: 厚みFF + テーパくさび + 自動較正）を、
旗以外の目標にも回す。**目標ごとに捕捉窓の広さが全く違う**ので、
点数×命中率の順位は戦略書の想定とずれる可能性がある。

捕捉窓の置き方（ルール 4.2 の得点条件から）
  旗       : 横棒の 0〜25cm 上を降下中に通過（布が垂れて巻き付く）／横 ±0.30m
  バケツ   : 開口 φ273。ふちに触れていても得点になるので、雑巾300mm幅を考慮して
             実効半径 0.20m とする（4.2 a「ふちに直接触れており」）
  机の棚   : 棚開口（W650×H約400）に水平に入れる。上下 ±0.20m ／横 ±0.30m
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import accuracy_budget as AB  # noqa: E402

# 対策込みの誤差（scripts/calibration_sim.py の到達点）
SIGMA_V = 0.0062          # 初速 1σ（EWMA α=0.10 適用後の残差）
SIGMA_YAW_DEG = 0.20
SIGMA_RANGE_M = 0.010

# 姿勢ばらつきは accuracy_budget.py と同じ値を使う（バックラッシュ + ニップ投入姿勢）
SIGMA_PITCH_DEG = math.sqrt(0.15 ** 2 + 0.60 ** 2)

# 注意（この解析の限界）
#   弾道は真空近似で、布の空力・はためき・捕捉過程を持たない。そのため
#   **絶対値は過大評価**になる。gpusim（布シム N=300）は同じ旗に対して 37% を出しており、
#   そちらが実測に近い。本表は「目標間の相対的な狙いやすさ」と
#   「そもそも射程に入るか」を見るために使うこと。

# 目標: (名前, 得点, 水平距離[m], 目標高さ[m], 上側許容, 下側許容, 横許容, 降下要求)
TARGETS = [
    ("旗（横棒 3.0m）", 100, 3.92, 3.00, 0.25, 0.00, 0.30, True),
    ("移動バケツ（相手静止時・上面1.4m）", 100, 3.00, 1.41, 0.20, 0.20, 0.20, True),
    ("机① 棚（20点）", 20, 4.50, 0.40, 0.20, 0.20, 0.30, False),
    ("固定バケツ②（台H600）", 5, 1.60, 0.86, 0.20, 0.20, 0.20, True),
    ("固定バケツ③（台H300）", 5, 1.60, 0.56, 0.20, 0.20, 0.20, True),
    ("固定バケツ①（床置き）", 1, 1.20, 0.26, 0.20, 0.20, 0.20, True),
]


def solve_v(dist: float, rise: float, pitch: float):
    """指定の仰角で、目標点を通る初速を返す（無ければ None）。"""
    c, t = math.cos(pitch), math.tan(pitch)
    denom = 2 * c * c * (dist * t - rise)
    if denom <= 0:
        return None
    v = math.sqrt(AB.G * dist * dist / denom)
    # 布の揚力補正（gpusim 較正。近距離では掛けない）
    v *= AB.C.ShotSolver()._pitch_calib(pitch, dist)
    return v if AB.C.MUZZLE_MIN <= v <= AB.C.MUZZLE_MAX else None


def best_aim(n, dist, z_t, up, down, lat, need_desc, rng):
    """仰角を掃引して、命中率が最大になる (初速, 仰角, 命中率) を返す。"""
    best = (None, None, 0.0)
    for deg in range(20, 71, 2):
        pitch = math.radians(deg)
        # 窓の中心を通す狙い
        rise_center = z_t + (up - down) / 2 - AB.C.NIP[2]
        v = solve_v(dist, rise_center, pitch)
        if v is None:
            continue
        p = evaluate(n // 4, v, pitch, dist, z_t, up, down, lat, need_desc, rng)
        if p > best[2]:
            best = (v, pitch, p)
    if best[0] is None:
        return None, None, None
    # 最良仰角で本試行
    p = evaluate(n, best[0], best[1], dist, z_t, up, down, lat, need_desc, rng)
    return best[0], best[1], p


def evaluate(n, v0, p0, dist, z_t, up, down, lat, need_desc, rng):
    """命中率を評価する。

    v0 は **指令初速**（布の揚力補正込み）。軌道の計算は真空式なので、
    等価な真空初速 v_eff = v_cmd / 補正係数 に直してから飛ばす。
    """
    calib = AB.C.ShotSolver()._pitch_calib(p0, dist)
    rise_lo = z_t - down - AB.C.NIP[2]
    rise_hi = z_t + up - AB.C.NIP[2]
    ok = 0
    for _ in range(n):
        v = v0 / calib * (1 + rng.gauss(0, SIGMA_V))
        pitch = p0 + math.radians(rng.gauss(0, SIGMA_PITCH_DEG))
        yaw = math.radians(rng.gauss(0, SIGMA_YAW_DEG))
        x = dist + rng.gauss(0, SIGMA_RANGE_M)
        z, dz = AB.height_at(x, v, pitch)
        if not (rise_lo <= z <= rise_hi):
            continue
        if need_desc and dz >= 0:
            continue
        if abs(x * math.tan(yaw)) > lat:
            continue
        ok += 1
    return ok / n


def hit_rate(n, dist, z_t, up, down, lat, need_desc, rng):
    v0, p0, p = best_aim(n, dist, z_t, up, down, lat, need_desc, rng)
    if v0 is None:
        return None, None, None
    return p, v0, math.degrees(p0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20000)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    rng = random.Random(20260729)

    rows = []
    for name, pts, dist, z_t, up, down, lat, desc in TARGETS:
        p, v0, deg = hit_rate(args.n, dist, z_t, up, down, lat, desc, rng)
        if p is None:
            rows.append((name, pts, dist, None, None, None, 0.0))
            continue
        rows.append((name, pts, dist, v0, deg, p, pts * p))
    rows.sort(key=lambda r: -r[6])

    if args.md:
        print("誤差は対策込み（厚みFF + テーパくさび + EWMA自動較正）。"
              f"初速 1σ={SIGMA_V * 100:.2f}%。\n")
        print("> **絶対値は過大評価**（真空近似で布の空力・捕捉過程を持たない）。"
              "同じ旗に対し gpusim（布シム N=300）は 37% を出しており、そちらが実測に近い。"
              "この表は「目標間の相対的な狙いやすさ」と「射程に入るか」を見るためのもの。\n")
        print("| 目標 | 得点 | 距離 | 初速 | 仰角 | 命中率 | **期待値/枚** |")
        print("|---|---|---|---|---|---|---|")
        for name, pts, dist, v0, deg, p, ev in rows:
            if v0 is None:
                print(f"| {name} | {pts} | {dist:.1f}m | — | — | 射程外 | — |")
                continue
            print(f"| {name} | {pts} | {dist:.1f}m | {v0:.2f} m/s | {deg:.0f}° | "
                  f"{p * 100:.0f}% | **{ev:.1f}** |")
        print()
        print("### 戦略書 §2.1 との比較")
        print()
        print("| 目標 | 戦略書の想定命中率 | 本解析 | 期待値の順位 |")
        print("|---|---|---|---|")
        assumed = {"旗（横棒 3.0m）": "70〜85%", "移動バケツ（相手静止時・上面1.4m）": "40〜60%",
                   "机① 棚（20点）": "90%+", "固定バケツ②（台H600）": "90%+",
                   "固定バケツ③（台H300）": "90%+", "固定バケツ①（床置き）": "95%+"}
        for i, (name, pts, dist, v0, deg, p, ev) in enumerate(rows, 1):
            got = "射程外" if p is None else f"{p * 100:.0f}%"
            print(f"| {name} | {assumed.get(name, '—')} | {got} | {i}位 |")
    else:
        for name, pts, dist, v0, deg, p, ev in rows:
            if v0 is None:
                print(f"  {name:32s} {pts:3d}点 {dist:.1f}m  射程外")
            else:
                print(f"  {name:32s} {pts:3d}点 {dist:.1f}m  v={v0:.2f} {deg:4.0f}°  "
                      f"命中{p * 100:5.1f}%  期待値 {ev:5.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
