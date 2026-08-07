"""SRX3616 の取付穴を外/中間/内レールに分類し、穴あけ図に落とす.

    python scripts/rail_holes.py [--md]

規格表に取付穴の位置は載っていない。これまで「M4×6箇所」と概算で置いていたが、
公式 STEP から読むと **φ4.5 が 28 面 / X 位置 20 箇所**あった。
3段引きなので外・中間・内の3部材の穴が混在している。

**外レールの穴だけが、こちらが板に開ける穴**（中間・内レールはキャリッジ側）。
Y/Z で分類して、外レールの穴位置を確定させる。

前提: `cad/vendor/misumi/SRX3616.stp`（取得手順は同ディレクトリの README）
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from build123d import import_step  # noqa: E402

import tr_params as P  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RAIL = os.path.abspath(os.path.join(HERE, "..", "vendor", "misumi", "SRX3616.stp"))


def to_slots(xs, gap=20.0):
    """近接した円弧2つを1つの長穴にまとめる。

    X 位置が「0と13」「23と28」のように2つ1組で近接しているのは、
    **長穴（スロット）の両端の円弧**が別々の円筒面として出ているため。
    取付調整のために長穴になっているのは3段引きレールとして自然。
    """
    slots, cur = [], []
    for x in sorted(xs):
        if cur and x - cur[-1] > gap:
            slots.append((cur[0], cur[-1]))
            cur = []
        cur.append(x)
    if cur:
        slots.append((cur[0], cur[-1]))
    return slots


def collect_holes(shape, r_lo=2.0, r_hi=2.5):
    """指定半径の円筒面の中心を集める。"""
    out = []
    for f in shape.faces():
        try:
            if "CYLINDER" not in str(f.geom_type).upper():
                continue
            r = f.radius
        except Exception:
            continue
        if not (r_lo <= r <= r_hi):
            continue
        c = f.center()
        out.append((round(c.X, 1), round(c.Y, 1), round(c.Z, 1), round(r, 2)))
    return sorted(set(out))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(RAIL):
        print(f"公式 CAD が無い: {RAIL}\n  取得手順は cad/vendor/misumi/README.md")
        return 1

    rail = import_step(RAIL)
    b = rail.bounding_box()
    holes = collect_holes(rail)

    # 3段引きなので厚み方向（一番薄い軸）で3層に分かれる。
    # 一番厚みのある軸＝レール長、次＝レール高さ、最薄＝3枚の重なり方向。
    span = {"X": b.max.X - b.min.X, "Y": b.max.Y - b.min.Y, "Z": b.max.Z - b.min.Z}
    long_ax = max(span, key=span.get)
    thin_ax = min(span, key=span.get)
    idx = {"X": 0, "Y": 1, "Z": 2}

    # 3段引きは厚み方向に3枚重なる。どの穴がどのレールかは
    # **厚み方向の位置だけでは決まらない**（表裏の面が別々に出るため）。
    # レール高さ方向（中間の軸）も併せて見る。
    mid_ax = [a for a in ("X", "Y", "Z") if a not in (long_ax, thin_ax)][0]
    layers: dict[tuple, list] = {}
    for h in holes:
        layers.setdefault((h[idx[thin_ax]], round(h[idx[mid_ax]] / 5) * 5), []).append(h)

    if args.md:
        print(f"`{os.path.basename(RAIL)}` の φ4.5 取付穴を、"
              f"厚み方向（{thin_ax}）で層に分けたもの。\n")
        print(f"- レール長手 = **{long_ax}** 軸（{span[long_ax]:.1f} mm）")
        print(f"- 厚み方向 = **{thin_ax}** 軸（{span[thin_ax]:.1f} mm、3段ぶんの重なり）")
        print(f"- 検出した穴（円筒面）: **{len(holes)} 面**\n")
        print(f"| {thin_ax}（面） | {mid_ax}（高さ） | 円弧 | **長穴** {long_ax} [mm] |")
        print("|---|---|---|---|")
        for (z, y), hs in sorted(layers.items()):
            xs = sorted({h[idx[long_ax]] for h in hs})
            sl = to_slots(xs)
            txt = ", ".join(f"**{a:.0f}〜{b:.0f}**" if a != b else f"{a:.0f}" for a, b in sl)
            print(f"| {z:.1f} | {y:+.0f} | {len(hs)} 面 | {len(sl)}箇所: {txt} |")
        print()
        print("### 読み方")
        print()
        print("- X 位置が「0と13」のように**2つ1組で近接**するのは、"
              "**長穴（スロット）の両端の円弧**が別々の円筒面として出るため。"
              "取付調整のために長穴になっているのは3段引きレールとして自然")
        print(f"- {thin_ax}=0.9 と {thin_ax}=18.2 はレールの**表と裏**。"
              f"{mid_ax}=±5 が両端の長穴（13mm）、{mid_ax}=0 が中間の長穴（5〜6mm）")
        print(f"- {thin_ax}=8.6/10.5 の {mid_ax}=±10・{long_ax}=187 は中央のストッパ")
        print()
        print("> **等間隔ではない。** 0, 23, 108, 248, 362 と不規則なので、"
              "規格表からの推定では絶対に当てられない。"
              "`tr_params.RAIL_HOLES` に取り込み済み（ASSEMBLY.md 工程7）")
    else:
        print(f"長手={long_ax}({span[long_ax]:.1f})  厚み={thin_ax}({span[thin_ax]:.1f})  "
              f"穴 {len(holes)} 面")
        for (z, y), hs in sorted(layers.items()):
            xs = sorted({h[idx[long_ax]] for h in hs})
            sl = to_slots(xs)
            print(f"  {thin_ax}={z:6.1f} {mid_ax}≈{y:6.0f}  {len(hs):2d}面 → 長穴{len(sl)}箇所  "
                  + ", ".join(f"{a:.0f}〜{b:.0f}" if a != b else f"{a:.0f}" for a, b in sl))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
