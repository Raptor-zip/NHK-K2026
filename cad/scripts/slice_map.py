"""指定領域に何が集まっているかを一覧する — 数値をいじって隙間を探すのをやめる.

    python scripts/slice_map.py --x 0 200 --z 690 880 [--pose stowed]

斜路まわりの干渉を「側板を 314→310 に」のように数値で逃がそうとして失敗した。
310 はキャリッジ(±313)とサイドプレート(±304.5)の**ちょうど間**で、
片方から逃げると必ずもう片方に当たる位置だった。

隙間を探す前に、**その領域に何がどの範囲で存在するのか**を全部並べて見る。
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

POSES = {"match": "POSE_MATCH", "stowed": "POSE_STOWED", "loading": "POSE_LOADING"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, nargs=2, default=[0.0, 200.0])
    ap.add_argument("--y", type=float, nargs=2, default=[-400.0, 400.0])
    ap.add_argument("--z", type=float, nargs=2, default=[690.0, 880.0])
    ap.add_argument("--pose", default="stowed", choices=sorted(POSES))
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    pose = getattr(P, POSES[args.pose])
    sol = V.solids_with_bbox(A.build(pose))
    x0, x1 = args.x
    y0, y1 = args.y
    z0, z1 = args.z

    rows = []
    for n, s, b in sol:
        if b.max.X < x0 or b.min.X > x1:
            continue
        if b.max.Y < y0 or b.min.Y > y1:
            continue
        if b.max.Z < z0 or b.min.Z > z1:
            continue
        rows.append((n.replace("/tr_robot/", ""), b))
    # |Y| の外側から順に並べる（横方向の押し合いを見たいので）
    rows.sort(key=lambda r: -max(abs(r[1].min.Y), abs(r[1].max.Y)))

    if args.md:
        print(f"姿勢 **{args.pose}** / 領域 X {x0:.0f}..{x1:.0f}, "
              f"Y {y0:.0f}..{y1:.0f}, Z {z0:.0f}..{z1:.0f} に存在する部品\n")
        print("| 部品 | X | Y | Z | 外側の \\|Y\\| |")
        print("|---|---|---|---|---|")
        for n, b in rows:
            print(f"| {n} | {b.min.X:.0f}..{b.max.X:.0f} | {b.min.Y:.0f}..{b.max.Y:.0f} | "
                  f"{b.min.Z:.0f}..{b.max.Z:.0f} | {max(abs(b.min.Y), abs(b.max.Y)):.1f} |")
    else:
        print(f"姿勢 {args.pose} / X {x0:.0f}..{x1:.0f} Y {y0:.0f}..{y1:.0f} "
              f"Z {z0:.0f}..{z1:.0f}  → {len(rows)} 個")
        for n, b in rows:
            print(f"  {n:<44} X {b.min.X:7.1f}..{b.max.X:7.1f}  "
                  f"Y {b.min.Y:7.1f}..{b.max.Y:7.1f}  Z {b.min.Z:7.1f}..{b.max.Z:7.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
