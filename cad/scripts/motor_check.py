"""DJI 公式 CAD とモーター寸法の照合.

    python scripts/motor_check.py [--md]

`tr_params.py` のモーター寸法には `[MEAS]`（実測で確定）が7項目あった。
データシートに寸法記入が無く、推定で置いていたもの。
DJI RoboMaster の公式 STEP（`cad/vendor/dji/`）が取れたので突き合わせる。

  M3508: 本体外径・インロー高さ
  M2006: 本体外径・本体長・軸径・軸長・取付PCD  ← **全部推定だった**
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from build123d import import_step  # noqa: E402

import tr_params as P  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.abspath(os.path.join(HERE, "..", "vendor", "dji"))


def cylinders(shape, r_lo=0.0, r_hi=1e9):
    """円筒面の (半径, 中心) を集める。"""
    out = []
    for f in shape.faces():
        try:
            if "CYLINDER" not in str(f.geom_type).upper():
                continue
            r = f.radius
        except Exception:
            continue
        if r_lo <= r <= r_hi:
            c = f.center()
            out.append((round(r, 2), (round(c.X, 1), round(c.Y, 1), round(c.Z, 1))))
    return out


def report(path, label, expect):
    """1モデルを読んで (項目, 設計値, 実測) の行を返す。"""
    if not os.path.exists(path):
        return [(f"{label}", "—", f"公式CADが無い: {os.path.basename(path)}")]
    m = import_step(path)
    b = m.bounding_box()
    dims = sorted((b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z))
    rows = [(f"**{label}** 外形（小→大）", "—",
             f"{dims[0]:.2f} × {dims[1]:.2f} × {dims[2]:.2f} mm")]
    rows.append((f"{label} 全長", f"{expect['len']:.1f}", f"**{dims[2]:.2f}**"))
    # 本体外径 = 最も面数の多い大きめの円筒
    cyl = cylinders(m, 5.0, 60.0)
    hist = {}
    for r, _ in cyl:
        hist[r] = hist.get(r, 0) + 1
    if hist:
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:6]
        rows.append((f"{label} 円筒面 半径→面数", "—",
                     " / ".join(f"r{r}×{n}" for r, n in top)))
        r_body = max(r for r, _ in top)
        rows.append((f"{label} 本体外径（最大円筒×2）", f"{expect['dia']:.1f}",
                     f"**{r_body * 2:.2f}**"))
    # 出力軸 = 小径の円筒
    shaft = cylinders(m, 2.0, 8.0)
    if shaft:
        rs = sorted({r for r, _ in shaft})
        rows.append((f"{label} 小径円筒（軸候補）", f"軸 φ{expect['shaft']:.0f}",
                     ", ".join(f"φ{r * 2:.1f}" for r in rs[:6])))
    # 取付穴 = M3/M4 のバカ穴。その中心が乗る円の直径が PCD
    holes = [c for r, c in cylinders(m, 1.5, 2.6)]
    if holes:
        rr = sorted({round(math.hypot(x, y), 1) for x, y, _ in holes})
        rows.append((f"{label} 取付穴中心の半径", f"PCD {expect['pcd']:.0f} → r{expect['pcd'] / 2:.0f}",
                     ", ".join(f"{v:.1f}" for v in rr[:6])))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    rows = []
    rows += report(os.path.join(VENDOR, "M3508_P19.step"), "M3508",
                   dict(len=P.M3508_TOTAL_LEN, dia=P.M3508_BODY_DIA,
                        shaft=P.M3508_SHAFT_DIA, pcd=P.M3508_BOLT_PCD))
    rows += report(os.path.join(VENDOR, "M2006_P36.step"), "M2006",
                   dict(len=P.M2006_BODY_LEN, dia=P.M2006_BODY_DIA,
                        shaft=P.M2006_SHAFT_DIA, pcd=P.M2006_BOLT_PCD))

    if args.md:
        print("DJI 公式 STEP（`cad/vendor/dji/`）と `tr_params.py` の突き合わせ。\n")
        print("> `[MEAS]` と書いていた項目は**データシートに寸法記入が無く推定**だった。"
              "M2006 は5項目すべてが推定値。\n")
        print("| 項目 | 設計値 | 公式 CAD |")
        print("|---|---|---|")
        for n, mine, off in rows:
            print(f"| {n} | {mine} | {off} |")
    else:
        for n, mine, off in rows:
            print(f"  {n:<34} 設計 {mine:>16}   公式 {off}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
