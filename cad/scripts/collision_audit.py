"""衝突形状の被覆監査 — シムが「見ていない」部分を洗い出す.

    python scripts/collision_audit.py [--md] [--grid 5]

今日2件、同じ種類のバグを踏んだ。

  * フォークのすくい斜面が机の天板より下を向いていたのに、
    URDF の衝突形状が斜面を含んでいなかったので**シムが接触を0回と報告していた**
  * 干渉チェックが局所座標のまま比較していたので、**必ず「干渉なし」になっていた**

どちらも「チェックが通った」だけで「チェックが機能していた」わけではない。
そこで**手書きの衝突形状が CAD の実形状をちゃんと覆っているか**を機械的に確かめる。

やりかた
  リンクごとに、CAD の各ソリッドの外接直方体を格子点でサンプリングし、
  「どの衝突ボックスにも入らない点」の割合を出す。
  はみ出しているソリッドを、はみ出し体積の大きい順に並べる。

注意（この監査の限界）
  * 外接直方体でサンプリングするので、斜めの部材や円筒は**過大に報告される**。
    数字そのものではなく「机や競技物に当たる向きにはみ出していないか」を見ること。
  * 逆に、衝突形状が実形状より**大きい**ぶんは検出しない（そちらは安全側）。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf"))

import tr_params as P  # noqa: E402
import tr_urdf as U  # noqa: E402

# リンク名 → 衝突ボックス群。ここに無いリンクは「形状の外接直方体そのもの」なので
# 定義上 100% 被覆になり、監査の対象外。
BOXSETS = {
    "base_link": U.BASE_COLLISION_BOXES,
    "grabber_slide": U.GRABBER_COLLISION_BOXES,
    "fork_tilt": U.FORK_COLLISION_BOXES,
}


# 衝突形状に入れない部品（意図的な省略）。除かないとノイズで本題が埋もれる。
SKIP = ("fasteners", "cables", "mascot", "decor", "screw", "washer")


def labeled_leaves(shape):
    """(ラベル, Solid) を返す。`node.solids()` を呼ぶとラベルが消えるので葉ノードを返す。"""
    out = []

    def walk(node, path):
        label = getattr(node, "label", "") or ""
        newpath = f"{path}/{label}" if label else path
        if any(s in newpath for s in SKIP):
            return
        kids = list(getattr(node, "children", []) or [])
        if kids:
            for c in kids:
                walk(c, newpath)
        else:
            out.append((newpath.lstrip("/"), node))

    walk(shape, "")
    return out


def inside_any(pt, boxes, tol=0.5):
    x, y, z = pt
    for _, (x0, x1), (y0, y1), (z0, z1) in boxes:
        if (x0 - tol <= x <= x1 + tol and y0 - tol <= y <= y1 + tol
                and z0 - tol <= z <= z1 + tol):
            return True
    return False


def audit_link(name, boxes, grid):
    """実形状の**頂点**をサンプリングする。

    外接直方体の格子で測ると、斜め材や細長い部材の bbox が巨大になり、
    「はみ出し体積 98,624 cm³」という機体の体積を超える値が出てしまった。
    頂点なら形状が実際に在る位置しか見ないので、この過大評価が起きない。
    """
    shape = U.LINK_SHAPE[name]()
    rows = []
    for label, s in labeled_leaves(shape):
        try:
            pts = [(v.X, v.Y, v.Z) for v in s.vertices()]
        except Exception:
            continue
        if not pts:
            continue
        miss = [p for p in pts if not inside_any(p, boxes)]
        if miss:
            rows.append((label.split("/")[-1], len(miss) / len(pts), len(miss),
                         min(p[2] for p in miss)))
    rows.sort(key=lambda r: (-r[1], r[3]))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=int, default=5)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    all_rows = {}
    for name, boxes in BOXSETS.items():
        all_rows[name] = audit_link(name, boxes, args.grid)

    if args.md:
        print("手書きの衝突形状が CAD の実形状を覆っているかの監査。"
              "**はみ出しているぶんはシムから見えない**。\n")
        print("| リンク | 部品 | はみ出した頂点 | はみ出しの最下点 | 机の天板 760 との関係 |")
        print("|---|---|---|---|---|")
        any_row = False
        for name, rows in all_rows.items():
            for label, frac, n_miss, zmiss in rows[:8]:
                any_row = True
                # フォークはリンク座標なので、机との比較には joint 原点を足す
                world = zmiss + (P.FORK_Z if name == "fork_tilt" else 0.0)
                rel = ("—" if name != "fork_tilt"
                       else ("⚠ **天板より下**" if world < P.DESK_H
                             else f"天板 +{world - P.DESK_H:.1f}mm"))
                print(f"| {name} | {label} | {frac * 100:.0f}%（{n_miss}点） | "
                      f"{zmiss:.1f} | {rel} |")
        if not any_row:
            print("| — | — | — | — | はみ出しなし ✅ |")
        print()
        print("- **`base_link` のはみ出しは想定内**。衝突形状は13個の直方体で"
              "構造を近似したもので、斜め材や細かい部品を1つずつ覆う意図はない。"
              "見るべきは「机や競技物に当たる向き」にはみ出していないか。")
        print("- 衝突形状が実形状より**大きい**ぶんは検出しない（そちらは安全側）。")
    else:
        for name, rows in all_rows.items():
            print(f"[{name}] はみ出しのある部品 {len(rows)} 個")
            for label, frac, n_miss, zmiss in rows[:8]:
                print(f"    {label:28s} はみ出し {frac * 100:5.1f}% ({n_miss:3d}点)  "
                      f"最下点 z={zmiss:8.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
