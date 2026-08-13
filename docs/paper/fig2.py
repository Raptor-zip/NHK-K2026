"""論文 Fig.2 —— 位相最適化の入力（設計領域と境界条件）と出力（輪郭）.

    cd cad && .venv/bin/python ../docs/paper/fig2.py [板名]

なぜ専用スクリプトなのか
------------------------
`cad/src/topo/draw.py` が出す図は**解き直すときに次へ動かすパラメータを
判断するための入力**で、内部の語（frac / rmin / dx）や 1 本ごとの荷重値まで
描いてある。論文の図に必要なのはそこではなく、
「元の板のどこを拘束し、どこから荷重が入り、結果どういう形になったか」の
3 点だけなので、別に描く。

⚠ 荷重の数値は 1 本ずつ書かない。20 本以上あって読めないうえ、
  読者が知りたいのは個々の値ではなく**荷重が入る位置と向き**である。
  大きさは矢印の長さで示し、最大値だけ凡例に書く。
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

CAD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cad")
sys.path.insert(0, os.path.join(CAD, "scripts"))
sys.path.insert(0, os.path.join(CAD, "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib import font_manager as fm             # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402
from matplotlib.lines import Line2D                   # noqa: E402

import topo_opt as T                                  # noqa: E402

JP = ("Noto Sans CJK JP", "Noto Sans JP", "IPAexGothic", "IPAGothic",
      "IPAPGothic", "TakaoPGothic")
_have = {f.name for f in fm.fontManager.ttflist}
for _n in JP:
    if _n in _have:
        plt.rcParams["font.family"] = [_n, "DejaVu Sans"]
        break
plt.rcParams["axes.unicode_minus"] = False

C_DOMAIN = "#d8dade"     # 元の板（設計領域）
C_SHAPE = "#3f4b5b"      # 最適化後の材料
C_FIX = "#1f4e9c"        # 拘束
C_LOAD = "#b03028"       # 荷重
C_SEAT = "#2f7d3f"       # 座面（消させない）
FS = 9.5


def rect_xy_raw(r):
    """Rect = (x0, y0, x1, y1) を (minx, miny, maxx, maxy) に正規化。"""
    x0, y0, x1, y1 = r
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def rect_xy(r):
    """Rect = (x0, y0, x1, y1) を matplotlib の (xy, w, h) に。"""
    x0, y0, x1, y1 = r
    return (min(x0, x1), min(y0, y1)), abs(x1 - x0), abs(y1 - y0)


def draw_domain(ax, reg):
    if reg.domain:
        ax.add_patch(Polygon(reg.domain, closed=True, facecolor=C_DOMAIN,
                             edgecolor="#7a8090", lw=1.0, zorder=1))
    else:
        ax.add_patch(Rectangle((-reg.w / 2, -reg.h / 2), reg.w, reg.h,
                               facecolor=C_DOMAIN, edgecolor="#7a8090",
                               lw=1.0, zorder=1))


def panel_bc(ax, reg):
    """(a) 設計領域と境界条件。"""
    draw_domain(ax, reg)

    # 拘束（動かない相手に留まっている座）
    for r in reg.fixed:
        xy, w, h = rect_xy(r)
        ax.add_patch(Rectangle(xy, w, h, facecolor="none", edgecolor=C_FIX,
                               lw=1.6, hatch="//////", zorder=4))

    # 座面（密度 1 固定）。169 個を 1 つずつ描くと団子になって板が見えないので、
    # 重なりを潰した**領域**として塗る（数ではなく「どこが消せないか」を示す図）。
    from shapely.geometry import box as sbox
    from shapely.ops import unary_union
    seats = unary_union([sbox(*rect_xy_raw(r)) for r in reg.solid])
    panel_bc.seats = seats
    for poly in (seats.geoms if hasattr(seats, "geoms") else [seats]):
        if poly.is_empty:
            continue
        ax.add_patch(Polygon(np.array(poly.exterior.coords), closed=True,
                             facecolor=C_SEAT, alpha=0.30, edgecolor=C_SEAT,
                             lw=0.8, zorder=5))

    # 逃げ（密度 0 固定）
    for c in reg.void:
        cx, cy, rr = c
        ax.add_patch(Circle((cx, cy), rr, facecolor="white",
                            edgecolor="#404040", lw=0.9, zorder=6))

    # 荷重。長さだけで大きさを示す（数値は書かない）
    fmax = max((abs(f) for _r, f, _a in reg.loads), default=1.0)
    L = 0.20 * max(reg.w, reg.h)
    for r, f, ang in reg.loads:
        (x0, y0), w, h = rect_xy(r)
        cx, cy = x0 + w / 2, y0 + h / 2
        ln = L * (abs(f) / fmax) ** 0.5
        dx, dy = ln * np.cos(ang), ln * np.sin(ang)
        ax.annotate("", (cx, cy), (cx - dx, cy - dy),
                    arrowprops=dict(arrowstyle="-|>", lw=1.3, color=C_LOAD,
                                    mutation_scale=9), zorder=7)
    return fmax


def panel_shape(ax, reg, js, seats=None):
    """(b) 最適化後の輪郭。元の外形を破線で、座面を薄く重ねて比較できるようにする。"""
    outer = np.array(js["outer"])
    ax.add_patch(Polygon(outer, closed=True, facecolor=C_SHAPE,
                         edgecolor="#20262f", lw=0.8, zorder=3))
    for hole in js["holes"]:
        ax.add_patch(Polygon(np.array(hole), closed=True, facecolor="white",
                             edgecolor="#20262f", lw=0.8, zorder=4))
    dom = reg.domain or [(-reg.w / 2, -reg.h / 2), (reg.w / 2, -reg.h / 2),
                         (reg.w / 2, reg.h / 2), (-reg.w / 2, reg.h / 2)]
    ax.add_patch(Polygon(dom, closed=True, facecolor="none",
                         edgecolor="#7a8090", lw=1.0, ls=(0, (4, 3)), zorder=5))
    # 座面を重ねる。(a) の緑と (b) の形の対応を、図の中で確かめられるようにする
    if seats is not None:
        for poly in (seats.geoms if hasattr(seats, "geoms") else [seats]):
            if poly.is_empty:
                continue
            ax.add_patch(Polygon(np.array(poly.exterior.coords), closed=True,
                                 facecolor="none", edgecolor=C_SEAT, lw=0.9,
                                 zorder=6))


def main() -> int:
    name = sys.argv[1] if len(sys.argv) > 1 else "pitch_side_R"
    regions, _skipped, _outside = T.collect()
    reg = next((r for r in regions if r.name == name), None)
    if reg is None:
        print("板が見つからない:", name)
        return 1
    with open(os.path.join(CAD, "out", "topo", f"{name}.json")) as f:
        js = json.load(f)

    fig, axes = plt.subplots(1, 2, figsize=(6.6, 3.15))
    fmax = panel_bc(axes[0], reg)
    panel_shape(axes[1], reg, js, seats=panel_bc.seats)

    for ax, ttl in zip(axes, ["(a) 設計領域と境界条件", "(b) 最適化後の輪郭"]):
        ax.set_aspect("equal")
        ax.set_xlim(-reg.w / 2 - 12, reg.w / 2 + 12)
        ax.set_ylim(-reg.h / 2 - 12, reg.h / 2 + 12)
        ax.set_xlabel("$x$ [mm]", fontsize=FS)
        ax.tick_params(labelsize=FS - 1.2)
        ax.set_title(ttl, fontsize=FS + 0.5, pad=5)
    axes[0].set_ylabel("$y$ [mm]", fontsize=FS)
    axes[1].tick_params(labelleft=False)

    handles = [
        Line2D([], [], marker="s", ls="", ms=8, mfc=C_DOMAIN, mec="#7a8090",
               label="設計領域（元の板）"),
        Line2D([], [], marker="s", ls="", ms=8, mfc="none", mec=C_FIX, mew=1.6,
               label="拘束"),
        Line2D([], [], marker=r"$\rightarrow$", ls="", ms=11, mec=C_LOAD,
               mfc=C_LOAD, label=f"荷重（長さ $\\propto$ 大きさ，最大 {fmax:.1f} N）"),
        Line2D([], [], marker="s", ls="", ms=8, mfc=(0.18, 0.49, 0.25, 0.30),
               mec=C_SEAT, label="座面（密度 1 固定）"),
        Line2D([], [], marker="o", ls="", ms=6, mfc="white", mec="#404040",
               label="逃げ（密度 0 固定）"),
        Line2D([], [], marker="s", ls="", ms=8, mfc=C_SHAPE, mec="#20262f",
               label="最適化後の材料"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=FS - 0.5,
               frameon=False, handletextpad=0.5, columnspacing=1.4,
               labelspacing=0.3, bbox_to_anchor=(0.5, -0.012))
    fig.subplots_adjust(left=0.085, right=0.995, top=0.93, bottom=0.30,
                        wspace=0.06)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs", "fig2.png")
    fig.savefig(out, dpi=400, facecolor="white")
    print(out)
    print(f"{name}: 面積 {js['area']:.0f} mm2 / 最小幅 {js['min_width']:.2f} mm "
          f"/ 穴 {len(js['holes'])} / 拘束 {len(reg.fixed)} / 荷重 {len(reg.loads)} "
          f"/ 座面 {len(reg.solid)} / 逃げ {len(reg.void)} / 最大荷重 {fmax:.1f} N "
          f"/ 元質量 {reg.mass0*1000:.1f} g")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
