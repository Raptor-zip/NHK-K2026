"""旋回体・可動部の**掃引包絡**を出す — 静止時の座標で判断するのをやめる.

    python scripts/sweep_envelope.py [--md]

同じ間違いを2回した。

  1. 元のコード: `FEED_RAMP_X1 = 175  # 旋回体の漏斗手前で10mm縁切り`
     → 漏斗が旋回体と一緒に回ることを勘定に入れず、静止時の X だけを見ていた
  2. その修正: `FEED_RAMP_X1 = 115  # 漏斗の -X 端 125.6 から 10mm 逃げる`
     → **また静止時の座標**。ヨー±30° に振ると漏斗は X=94 まで来る

固定物を置いてよい範囲は「静止時の旋回体の外」ではなく、
**ヨー全域を振ったときに旋回体が占める領域の外**である。
ヨー角を刻んで包絡を取り、固定側が使える境界を数値で出す。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_assembly as A  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402

# 調べる Z 帯（斜路が通る高さ）と、ヨーの刻み
Z_BANDS = [(700.0, 760.0), (760.0, 810.0), (810.0, 840.0), (840.0, 900.0)]
YAW_STEPS = 7          # -30..+30 を 7 点（10° 刻み）
# ⚠ 仰角は **3点では足りなかった**。20/45/70 だけを見て包絡を出したところ、
#   仰角60° で漏斗が斜路に当たる状態を見落とした（総当たり干渉チェックで発覚）。
#   端点と中間だけでは、途中で最も張り出す姿勢を飛ばすことがある。
PITCHES = tuple(P.PITCH_MIN + (P.PITCH_MAX - P.PITCH_MIN) * i / 5 for i in range(6))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    # ヨー×仰角を振って、旋回体グループの最小 X（＝固定側に最も食い込む点）を Z 帯ごとに取る
    band_min_x = {b: 1e9 for b in Z_BANDS}
    band_at = {b: None for b in Z_BANDS}
    for i in range(YAW_STEPS):
        yaw = -P.YAW_LIMIT + 2 * P.YAW_LIMIT * i / (YAW_STEPS - 1)
        for pitch in PITCHES:
            pose = dict(P.POSE_MATCH, yaw=yaw, pitch=pitch)
            for n, s, b in V.solids_with_bbox(A.build(pose)):
                if "/turret" not in n:
                    continue
                for band in Z_BANDS:
                    if b.max.Z < band[0] or b.min.Z > band[1]:
                        continue
                    if b.min.X < band_min_x[band]:
                        band_min_x[band] = b.min.X
                        band_at[band] = (yaw, pitch, n.split("/")[-1])

    if args.md:
        print(f"ヨー ±{P.YAW_LIMIT:.0f}° を {YAW_STEPS} 点、仰角 "
              f"{'/'.join(f'{p:.0f}' for p in PITCHES)}° で振ったときに、"
              "旋回体が **-X 方向へどこまで来るか**。\n")
        print("| Z 帯 | 旋回体の最小 X | そのときのヨー/仰角 | 部品 | 固定物を置ける上限 X |")
        print("|---|---|---|---|---|")
        for band in Z_BANDS:
            if band_at[band] is None:
                print(f"| {band[0]:.0f}..{band[1]:.0f} | — | — | 旋回体が来ない | 制約なし |")
                continue
            yaw, pitch, name = band_at[band]
            print(f"| {band[0]:.0f}..{band[1]:.0f} | **{band_min_x[band]:.1f}** | "
                  f"{yaw:+.0f}° / {pitch:.0f}° | {name} | "
                  f"**{band_min_x[band] - 10:.0f}**（10mm 逃げる） |")
        print()
        print(f"- 現在の斜路: X {P.FEED_RAMP_X0:.0f}..{P.FEED_RAMP_X1:.0f} / "
              f"Z {P.FEED_RAMP_Z0:.0f}..{P.FEED_RAMP_Z1:.0f}")
        print("- **静止時（ヨー0°）の座標で判断してはいけない。**"
              "旋回体は回るので、固定物が置けるのは上の「掃引包絡の外」だけ。")
    else:
        for band in Z_BANDS:
            if band_at[band] is None:
                print(f"  Z {band[0]:.0f}..{band[1]:.0f}  旋回体が来ない")
                continue
            yaw, pitch, name = band_at[band]
            print(f"  Z {band[0]:6.0f}..{band[1]:6.0f}  最小X {band_min_x[band]:7.1f}  "
                  f"(yaw{yaw:+.0f} pitch{pitch:.0f} {name})  → 固定物は X <= "
                  f"{band_min_x[band] - 10:.0f}")
        print(f"\n  現在の斜路 X {P.FEED_RAMP_X0:.0f}..{P.FEED_RAMP_X1:.0f} / "
              f"Z {P.FEED_RAMP_Z0:.0f}..{P.FEED_RAMP_Z1:.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
