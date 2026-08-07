"""MISUMI 公式 CAD と自作モデルの照合.

    python scripts/vendor_check.py [--md]

これまで MISUMI にログインできず、カタログの規格表から手作業でモデリングしていた。
ログイン後に公式 STEP を取得できたので（`cad/vendor/misumi/`）、
**自分が置いた寸法が実物と合っているか**を機械的に突き合わせる。

規格表に載っている寸法（全長・幅・高さ）は合っていて当然だが、
**載っていない寸法**——取付穴の位置、溝の実形状、断面積——はこちらが推定していた。
そこが本当の照合対象。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from build123d import Axis, import_step  # noqa: E402

import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.abspath(os.path.join(HERE, "..", "vendor", "misumi"))


def bbox_of(shape):
    b = shape.bounding_box()
    return (b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    rows = []

    # --- HFS5-2020: 断面積とねじれ定数が静解析の入力になっている ---
    path = os.path.join(VENDOR, "HFS5-2020-840.stp")
    if os.path.exists(path):
        off = import_step(path)
        sx, sy, sz = bbox_of(off)
        vol = off.volume
        # 押し出し方向（一番長い軸）で割れば断面積が出る
        ln = max(sx, sy, sz)
        area = vol / ln
        rows.append(("HFS5-2020 断面 W×H", f"{P.EXT_W:.1f} × {P.EXT_W:.1f}",
                     f"{min(sx, sy, sz):.2f} × {sorted((sx, sy, sz))[1]:.2f}"))
        rows.append(("HFS5-2020 断面積 [mm²]", f"{P.EXT_AREA:.1f}", f"{area:.1f}"))
        rows.append(("HFS5-2020 単位質量 [kg/m]", f"{P.EXT_MASS_PER_M:.3f}",
                     f"{area * P.DENSITY['A6005C'] / 1000:.3f}"))
        # 自作プロファイルと比べる
        mine = L.ext2020(ln)
        rows.append(("HFS5-2020 体積比（自作/公式）", "1.000",
                     f"{mine.volume / vol:.3f}"))

    # --- SRX3616: 取付穴の位置は規格表に無く、こちらが推定していた ---
    path = os.path.join(VENDOR, "SRX3616.stp")
    if os.path.exists(path):
        rail = import_step(path)
        sx, sy, sz = bbox_of(rail)
        dims = sorted((sx, sy, sz))
        rows.append(("SRX3616 レール長 [mm]", f"{P.RAIL_LEN:.1f}", f"{dims[2]:.2f}"))
        rows.append(("SRX3616 高さ(幅) [mm]", f"{P.RAIL_H:.1f}", f"{dims[1]:.2f}"))
        rows.append(("SRX3616 厚み [mm]", f"{P.RAIL_T:.1f}", f"{dims[0]:.2f}"))
        # ⚠ 取付穴の位置を 3D から読もうとしたが、**円筒面が1つも無かった**。
        #   MISUMI のダウンロード画面にも「一部仕様が省略されたデータを提供しております」
        #   とある。公式データ＝実物ではない。
        #   → 取付穴の位置は **2D 図面（DXF）から読む**こと。
        # 取付穴を数える。
        # ⚠ `f.geom_type` は **GeomType 列挙型**であって文字列ではない。
        #    `f.geom_type != "CYLINDER"` と書くと常に真になり、
        #    「円筒面が1つも無い」という誤った結論になる（実際そう報告してしまった）。
        holes = {}
        for f in rail.faces():
            try:
                if "CYLINDER" not in str(f.geom_type).upper():
                    continue
                r = round(f.radius, 2)
                c = f.center()
            except Exception:
                continue
            holes.setdefault(r, []).append((round(c.X, 1), round(c.Y, 1), round(c.Z, 1)))
        rows.append(("SRX3616 円筒面の半径→個数", "—",
                     " / ".join(f"r{r}×{len(v)}" for r, v in sorted(holes.items()))))
        # 取付穴の長手位置を出す。
        # ⚠ `dims` は**ソート済み**の寸法配列なので、`max(range(3), key=dims.__getitem__)`
        #    は常に 2 を返す。座標軸のインデックスには使えない（使って間違えた）。
        #    長手軸は bbox の生の寸法から決めること。
        raw = (sx, sy, sz)
        axis_i = max(range(3), key=lambda i: raw[i])
        axis_name = "XYZ"[axis_i]
        for r, pts in sorted(holes.items()):
            if len(pts) < 6:               # 数の多いものが取付穴
                continue
            vals = sorted({p[axis_i] for p in pts})
            rows.append((f"SRX3616 φ{r * 2:.1f} 穴（{len(pts)}面）の {axis_name} 位置",
                         "推定で6箇所",
                         f"**{len(vals)}箇所**: " + ", ".join(f"{v:.0f}" for v in vals[:12])))

    if args.md:
        print("MISUMI 公式 STEP（`cad/vendor/misumi/`）と自作モデルの突き合わせ。\n")
        print("| 項目 | 自作モデル | 公式 CAD |")
        print("|---|---|---|")
        for n, mine, off in rows:
            print(f"| {n} | {mine} | **{off}** |")
    else:
        for n, mine, off in rows:
            print(f"  {n:<34} 自作 {mine:>22}   公式 {off}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
