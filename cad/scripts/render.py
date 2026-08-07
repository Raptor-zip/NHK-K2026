"""機体を PNG に描いて**目で確かめる**ための投影図・断面図.

    python scripts/render.py                     # 既定の6枚
    python scripts/render.py --pose loading      # 姿勢を変える
    python scripts/render.py --section y=0       # 断面（平面 y=0 で切る）
    python scripts/render.py --zoom turret       # 部位を拡大

なぜ要るか
-----------
数値の検査だけで進めていたら、**射出ローラーが旋回テーブルを貫いている**という
一目で分かる誤りを長く見落としていた。原因は「台座↔砲塔は ROTATING（接触可）」と
宣言して検査から外していたことだが、図を一度見れば宣言の誤りにも気付けた。

数値は「どこが何mm」を教えるが、「その形が機械として成立しているか」は
投影図のほうが速い。**両方回す。**

描画は matplotlib で、各ソリッドの三角メッシュを指定方向へ正射影して
塗りつぶす。深度でソートするだけの簡易版だが、貫通・浮き・向き違いは見える。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

import tr_assembly as A  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402

POSES = {"match": lambda: P.POSE_MATCH,
         "stowed": lambda: P.POSE_STOWED,
         "loading": lambda: P.POSE_LOADING}

# 視線方向 → (横軸, 縦軸, 奥行き軸, 反転)
VIEWS = {
    "front": (1, 2, 0, +1),    # +X から見る（横軸 Y、縦軸 Z）
    "side":  (0, 2, 1, -1),    # -Y から見る（横軸 X、縦軸 Z）
    "top":   (0, 1, 2, -1),    # 上から見る（横軸 X、縦軸 Y）
}

# 部位ごとの表示範囲 [mm]（(x0,x1,y0,y1,z0,z1)）
ZOOMS = {
    "turret": (60, 420, -360, 360, 780, 1060),
    "grabber": (-500, 200, -360, 360, 740, 900),
    "hopper": (-460, 120, -360, 360, 560, 780),
    "base": (-460, 500, -400, 400, 0, 200),
    # 表示器の取付（横材 ↔ 後柱の金具・枠の爪・画面）
    "display": (-330, -240, -400, 400, 400, 560),
    # 横材と後柱の継手だけ（L 金具 2 個とその溝ナット）
    "display_joint": (-300, -255, 300, 380, 440, 520),
}


def tri_of(solid, tol=0.6):
    """ソリッドの三角メッシュ（頂点配列, 面配列）。"""
    verts, faces = solid.tessellate(tolerance=tol)
    return np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(faces)


# 実体を持たない検証用の形状。既定では描かない（描くと中が見えない）
VIRTUAL = ("mascot_envelope", "bucket_2l_datum")


def color_of(name):
    """部品名から安定した色を作る。

    ⚠ `.solids()` で取り出したソリッドは**親に付けた色を持っていない**。
      材質色をそのまま使おうとして全部が同じ灰色になった。
      干渉を目で探すのが目的なので、**隣り合う部品が違う色**であればよい。
      名前のハッシュから HSV を振る（同じ部品はいつも同じ色）。
    """
    import colorsys
    import hashlib
    h = int(hashlib.md5(name.encode()).hexdigest()[:8], 16)
    hue = (h % 997) / 997.0
    sat = 0.35 + ((h >> 10) % 100) / 400.0
    val = 0.62 + ((h >> 17) % 100) / 320.0
    return colorsys.hsv_to_rgb(hue, sat, min(val, 0.95))


def draw(ax, sol, view, box=None, section=None, lw=0.15, hide=VIRTUAL, only=None):
    h, v, d, sgn = VIEWS[view]
    polys, colors, depth = [], [], []
    for name, solid, bb in sol:
        leaf = name.split("/")[-1].split("#")[0]
        if any(k in leaf for k in hide):
            continue
        if only and not any(k in leaf for k in only):
            continue
        if box is not None:
            if (bb.max.X < box[0] or bb.min.X > box[1] or bb.max.Y < box[2]
                    or bb.min.Y > box[3] or bb.max.Z < box[4] or bb.min.Z > box[5]):
                continue
        try:
            pts, fcs = tri_of(solid)
        except Exception:
            continue
        if len(fcs) == 0:
            continue
        rgb = color_of(leaf)
        tri = pts[fcs]                                  # (F,3,3)
        if section is not None:
            axis, val, keep = section
            c = tri[:, :, axis].mean(axis=1)
            tri = tri[(c - val) * keep >= 0]
            if len(tri) == 0:
                continue
        polys.append(tri[:, :, [h, v]])
        colors.append(rgb)
        depth.append(sgn * tri[:, :, d].mean())
    order = np.argsort(depth)
    for i in order:
        ax.add_collection(PolyCollection(polys[i], facecolors=[colors[i]],
                                         edgecolors=(0, 0, 0, 0.35), linewidths=lw))
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xticks([]); ax.set_yticks([])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="match", choices=list(POSES))
    ap.add_argument("--out", default="out/render")
    ap.add_argument("--section", default=None,
                    help="断面。例 y=0 / x=235 / z=870（切った手前側を捨てる）")
    ap.add_argument("--zoom", default=None, choices=list(ZOOMS))
    ap.add_argument("--views", default="front,side,top")
    ap.add_argument("--hide", default="", help="名前に含むと描かない（カンマ区切り）")
    ap.add_argument("--only", default="", help="名前に含むものだけ描く（カンマ区切り）")
    ap.add_argument("--box", default=None,
                    help="表示範囲 x0,x1,y0,y1,z0,z1（--zoom より優先）")
    args = ap.parse_args()

    sol = V.solids_with_bbox(A.build(POSES[args.pose]()))
    box = ZOOMS.get(args.zoom)
    if args.box:
        box = tuple(float(v) for v in args.box.split(","))
    section = None
    if args.section:
        axis_name, val = args.section.split("=")
        section = ("xyz".index(axis_name.strip()), float(val), -1.0)

    views = args.views.split(",")
    fig, axes = plt.subplots(1, len(views), figsize=(7 * len(views), 7))
    if len(views) == 1:
        axes = [axes]
    hide = VIRTUAL + tuple(x for x in args.hide.split(",") if x)
    only = tuple(x for x in args.only.split(",") if x) or None
    for ax, vw in zip(axes, views):
        draw(ax, sol, vw, box=box, section=section, hide=hide, only=only)
        title = vw
        if args.zoom:
            title += f" / {args.zoom}"
        if args.section:
            title += f" / 断面 {args.section}"
        ax.set_title(title, fontsize=11)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    tag = args.pose + ("_" + args.zoom if args.zoom else "")
    tag += ("_sec" + args.section.replace("=", "") if args.section else "")
    tag += ("_only" + args.only.replace(",", "-") if args.only else "")
    path = f"{args.out}_{tag}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor="white")
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
