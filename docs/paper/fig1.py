"""論文 Fig.1 —— 機体の系統色分け投影図（side / front）+ 寸法注記.

    cd cad && .venv/bin/python ../docs/paper/fig1.py

なぜ専用スクリプトなのか
------------------------
`cad/scripts/render.py` の投影図は「干渉を目で探す」ためのもので、
部品名のハッシュから色を振る（隣り合う部品が違う色であればよい）。
論文の図として読ませたいのは個々の部品ではなく **系統の配置と寸法** なので、
色は系統（走行・シャシー・装填・射出・マスト…）で与え、
系統ごとの番号を投影上の重心に置き、凡例と対応させる。

寸法は組立から実測して描く（数値をこの図に手書きしない）。
"""

from __future__ import annotations

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
from matplotlib.collections import PolyCollection     # noqa: E402
from matplotlib.lines import Line2D                   # noqa: E402

import tr_assembly as A                               # noqa: E402
import tr_params as P                                 # noqa: E402
import validate as V                                  # noqa: E402

JP = ("Noto Sans CJK JP", "Noto Sans JP", "IPAexGothic", "IPAGothic",
      "IPAPGothic", "TakaoPGothic")
_have = {f.name for f in fm.fontManager.ttflist}
for _n in JP:
    if _n in _have:
        plt.rcParams["font.family"] = [_n, "DejaVu Sans"]
        break
plt.rcParams["axes.unicode_minus"] = False

# ---------------------------------------------------------------- 系統の定義
# (番号, 表示名, 色, 判定関数)  番号 0 = 締結要素（番号を振らない）
FASTEN = "#c9ccd1"


def _grp(path: str) -> str:
    """ラベルパス /tr_robot/<link>/<group>/... から (link, group) を取る。"""
    p = path.split("/")
    link = p[2] if len(p) > 2 else ""
    grp = p[3] if len(p) > 3 else ""
    return link, grp


SYSTEMS = [
    (1, "走行系（メカナム 4 輪・駆動）", "#4878a8",
     lambda l, g: l.startswith("wheel_") or g == "drive_mount"),
    (2, "シャシー（ベース・側面トラス・デッキ）", "#8d9aa8",
     lambda l, g: g in ("base_frame", "side_frame", "deck", "skirt")),
    (3, "装填（フォーク・上押さえ・レール）", "#d1802f",
     lambda l, g: l in ("grabber", "slide_rails", "slide_rails_fixed")
     or g == "grabber_fixed"),
    (4, "ホッパー", "#c4a83c",
     lambda l, g: g == "hopper"),
    (5, "分離機構（シンギュレータ）", "#b5563f",
     lambda l, g: l == "singulator"),
    (6, "搬送斜路", "#7a9e4f",
     lambda l, g: g == "feed_ramp"),
    (7, "砲塔・射出ユニット", "#3f7d6e",
     lambda l, g: l == "turret"),
    (8, "マスト・移動バケツ", "#6b5b95",
     lambda l, g: g in ("mast", "bucket")),
    (9, "椅子・マスコット", "#a8785f",
     lambda l, g: g == "chair"),
    (10, "センサ・電装・配線", "#4f4f4f",
     lambda l, g: g in ("sensors", "electronics", "cables")),
]
SYS_BY_N = {n: (nm, c) for n, nm, c, _ in SYSTEMS}

VIRTUAL = ("mascot_envelope", "bucket_2l_datum")
VIEWS = {"side": (0, 2, 1, -1), "front": (1, 2, 0, +1)}


def classify(path: str) -> int:
    link, grp = _grp(path)
    if grp in ("auto_fasteners", "fasteners"):
        return 0
    for n, _nm, _c, f in SYSTEMS:
        if f(link, grp):
            return n
    return 0


def tri_of(solid, tol=0.8):
    verts, faces = solid.tessellate(tolerance=tol)
    return np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(faces)


def shade(rgb_hex: str, k: float) -> tuple:
    """面の向きに応じた明暗（k: 0..1）。単色の塗りつぶしよりも形が読める。"""
    r = int(rgb_hex[1:3], 16) / 255
    g = int(rgb_hex[3:5], 16) / 255
    b = int(rgb_hex[5:7], 16) / 255
    w = 0.35 + 0.65 * k
    return (min(r * w + 0.12 * (1 - k), 1), min(g * w + 0.12 * (1 - k), 1),
            min(b * w + 0.12 * (1 - k), 1))


def draw(ax, sol, view):
    """投影して塗る。戻り値は系統ごとの投影点群（番号の置き場所を出すため）。"""
    h, v, d, sgn = VIEWS[view]
    polys, colors, depth = [], [], []
    pts_by_sys: dict[int, list] = {}
    for name, tri, _bb in sol:
        s = classify(name)
        proj = tri[:, :, [h, v]]
        # 面法線の奥行成分で陰影を付ける
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(n, axis=1)
        ln[ln == 0] = 1.0
        k = np.abs(n[:, d]) / ln                          # 0..1
        base = FASTEN if s == 0 else SYS_BY_N[s][1]
        for i in range(len(proj)):
            polys.append(proj[i])
            colors.append(shade(base, float(k[i])))
        depth += [sgn * tri[i, :, d].mean() for i in range(len(tri))]
        if s:
            pts_by_sys.setdefault(s, []).append(proj.reshape(-1, 2))
    order = np.argsort(np.array(depth))
    ax.add_collection(PolyCollection([polys[i] for i in order],
                                     facecolors=[colors[i] for i in order],
                                     edgecolors=(0, 0, 0, 0.16), linewidths=0.10))
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    return {s: np.concatenate(v_) for s, v_ in pts_by_sys.items()}


# ---------------------------------------------------------------- 寸法の描画
DIM = dict(color="#1a1a1a", lw=0.8)
# 注記の下地（機体と重なっても字が読めるように）
BOX = dict(facecolor="white", alpha=0.82, edgecolor="none", pad=1.2)
FS = 8.2


def vdim(ax, x, z0, z1, label, side=1, tick=26):
    """鉛直寸法線（x 位置に立てる）。"""
    ax.annotate("", (x, z0), (x, z1),
                arrowprops=dict(arrowstyle="<->", shrinkA=0, shrinkB=0, **DIM))
    for z in (z0, z1):
        ax.plot([x - tick * 0.5, x + tick * 0.5], [z, z], **DIM)
    ax.text(x + side * 16, (z0 + z1) / 2, label, fontsize=FS, rotation=90,
            ha="left" if side > 0 else "right", va="center", color="#1a1a1a")


def hdim(ax, z, x0, x1, label, below=True, tick=26):
    ax.annotate("", (x0, z), (x1, z),
                arrowprops=dict(arrowstyle="<->", shrinkA=0, shrinkB=0, **DIM))
    for x in (x0, x1):
        ax.plot([x, x], [z - tick * 0.5, z + tick * 0.5], **DIM)
    ax.text((x0 + x1) / 2, z + (-30 if below else 22), label, fontsize=FS,
            ha="center", va="top" if below else "bottom", color="#1a1a1a")


def hline(ax, z, x0, x1, label, ls=(0, (5, 3)), color="#7a2f2f", va="bottom"):
    ax.plot([x0, x1], [z, z], ls=ls, lw=0.9, color=color, zorder=5)
    ax.text(x1, z + (8 if va == "bottom" else -8), label, fontsize=FS - 0.3,
            ha="right", va=va, color=color, zorder=6)


def badge(ax, x, z, n):
    ax.scatter([x], [z], s=176, marker="o", facecolor="white",
               edgecolor="#1a1a1a", linewidth=0.9, zorder=20)
    ax.text(x, z, str(n), fontsize=8.4, ha="center", va="center",
            color="#1a1a1a", zorder=21)


# ---------------------------------------------------------------- 主処理
def main() -> int:
    # 組立とテセレーションで数分かかるので、投影の素材だけ pickle に取る。
    # ⚠ キャッシュは図の体裁を詰めるための一時物。CAD を直したら --rebuild。
    import pickle
    here = os.path.dirname(os.path.abspath(__file__))
    cache = os.path.join(here, "figs", ".fig1_cache.pkl")
    if "--rebuild" not in sys.argv and os.path.exists(cache):
        with open(cache, "rb") as f:
            sol = pickle.load(f)
    else:
        sol = []
        for name, solid, bb in V.solids_with_bbox(A.build(P.POSE_MATCH)):
            leaf = name.split("/")[-1].split("#")[0]
            if any(k in leaf for k in VIRTUAL):
                continue
            try:
                pts, fcs = tri_of(solid)
            except Exception:
                continue
            if len(fcs) == 0:
                continue
            sol.append((name, pts[fcs].astype(np.float32),
                        np.array([[bb.min.X, bb.min.Y, bb.min.Z],
                                  [bb.max.X, bb.max.Y, bb.max.Z]])))
        with open(cache, "wb") as f:
            pickle.dump(sol, f)

    # --- 実測（この図に描く数値は全部ここから出す） --------------------
    def bbox(pred):
        lo = np.array([+1e9] * 3)
        hi = np.array([-1e9] * 3)
        for name, _t, bb in sol:
            if not pred(name):
                continue
            lo = np.minimum(lo, bb[0])
            hi = np.maximum(hi, bb[1])
        return lo, hi

    is_bucket = lambda n: _grp(n)[1] == "bucket"          # noqa: E731
    lo_a, hi_a = bbox(lambda n: True)                     # 全体（バケツ込み）
    lo_m, hi_m = bbox(lambda n: not is_bucket(n))         # 機体（バケツ除く）
    lo_b, hi_b = bbox(is_bucket)

    z_top = hi_m[2]                 # 機体最高点
    z_buck = hi_b[2]                # バケツ上面
    x0, x1 = lo_a[0], hi_a[0]
    y0, y1 = lo_a[1], hi_a[1]

    fig = plt.figure(figsize=(7.05, 4.35))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.72, 1.0], wspace=0.02,
                          left=0.012, right=0.988, top=0.985, bottom=0.215)
    ax_s = fig.add_subplot(gs[0, 0])
    ax_f = fig.add_subplot(gs[0, 1])

    pts_s = draw(ax_s, sol, "side")
    pts_f = draw(ax_f, sol, "front")

    # --- 側面図: 基準線と寸法 -----------------------------------------
    ax_s.plot([x0 - 120, x1 + 190], [0, 0], color="#1a1a1a", lw=1.0)
    for i in range(14):
        xx = x0 - 120 + i * (x1 - x0 + 310) / 13
        ax_s.plot([xx, xx - 26], [0, -26], color="#1a1a1a", lw=0.6)

    ax_s.plot([x0 - 300, x1 + 210], [P.BUCKET_2L_Z] * 2, ls=(0, (5, 3)),
              lw=0.9, color="#7a2f2f", zorder=5)
    ax_s.text(x0 - 300, P.BUCKET_2L_Z + 30,
              f"2 L 目盛り {P.BUCKET_2L_Z:.0f}（規定 3.2.3c）",
              fontsize=FS - 0.3, ha="left", va="bottom", color="#7a2f2f", zorder=6,
              bbox=BOX)
    ax_s.plot([x0 - 300, x0 - 40], [P.DESK_H] * 2, ls=(0, (5, 3)), lw=0.9,
              color="#2f4f7a", zorder=5)
    ax_s.text(x0 - 300, P.DESK_H - 18, f"机 上面 {P.DESK_H:.0f}",
              fontsize=FS - 0.3, ha="left", va="top", color="#2f4f7a", zorder=6)

    vdim(ax_s, x1 + 110, 0, z_top, f"機体最高点 {z_top:.0f}（競技姿勢）", side=1)
    vdim(ax_s, x1 + 250, 0, z_buck, f"バケツ上面 {z_buck:.0f}", side=1)
    hdim(ax_s, -95, x0, x1, f"全長 {x1 - x0:.0f}")

    # 射出点（ヨー軸 = 仰角軸 = ニップ）
    ax_s.scatter([P.TURRET_X], [P.NIP_Z], s=46, marker="+", color="#7a2f2f",
                 linewidth=1.4, zorder=22)
    ax_s.annotate(f"射出点 (X {P.TURRET_X:.0f}, Z {P.NIP_Z:.0f})\n"
                  "ヨー軸＝仰角軸＝ニップ",
                  (P.TURRET_X, P.NIP_Z), (x0 - 300, P.NIP_Z + 35),
                  fontsize=FS - 0.3, color="#7a2f2f", ha="left", va="bottom",
                  bbox=BOX,
                  arrowprops=dict(arrowstyle="-", lw=0.7, color="#7a2f2f"))
    # ホッパー開口
    ax_s.annotate(f"ホッパー開口 {P.HOP_TOP_Z:.0f}",
                  (P.HOP_X0 + 60, P.HOP_TOP_Z), (x0 - 300, P.HOP_TOP_Z + 230),
                  fontsize=FS - 0.3, color="#5a4a10", ha="left", bbox=BOX,
                  arrowprops=dict(arrowstyle="-", lw=0.7, color="#5a4a10"))
    # 座標系
    ox, oz = x0 - 290, 250
    ax_s.annotate("", (ox + 150, oz), (ox, oz),
                  arrowprops=dict(arrowstyle="->", lw=0.9, color="#1a1a1a"))
    ax_s.annotate("", (ox, oz + 150), (ox, oz),
                  arrowprops=dict(arrowstyle="->", lw=0.9, color="#1a1a1a"))
    ax_s.text(ox + 158, oz, "+X", fontsize=FS, va="center")
    ax_s.text(ox, oz + 160, "+Z", fontsize=FS, ha="center")

    ax_s.text(0.5, -0.045, "(a) 側面（$-Y$ から見る。$+X$ が投擲方向）",
              transform=ax_s.transAxes, ha="center", va="top", fontsize=FS + 0.6)

    # --- 正面図 -------------------------------------------------------
    ax_f.plot([y0 - 120, y1 + 120], [0, 0], color="#1a1a1a", lw=1.0)
    hline(ax_f, P.BUCKET_2L_Z, y0 - 90, y1 + 90, "")
    hdim(ax_f, -95, y0, y1, f"全幅 {y1 - y0:.0f}")
    ax_f.text(0.5, -0.045, "(b) 正面（$+X$ から見る）",
              transform=ax_f.transAxes, ha="center", va="top", fontsize=FS + 0.6)

    # --- 系統の番号（投影上の重心へ置く） ------------------------------
    # 側面図に置く系統と、正面図に置く系統を分ける（重なりを避ける）
    # 重心が他の系統と重なる番号だけ、絶対位置を指定する
    ABS = {1: (-300, 50), 2: (-120, 165), 3: (-395, 800), 4: (-300, 620),
           5: (30, 665), 8: (-150, 1385), 9: (250, 480), 10: (447, 70)}
    for s, pts in pts_s.items():
        cx, cz = ABS.get(s, (pts[:, 0].mean(), pts[:, 1].mean()))
        badge(ax_s, cx, cz, s)

    for ax in (ax_s, ax_f):
        ax.autoscale_view()
    ax_s.set_xlim(x0 - 320, x1 + 330)
    ax_s.set_ylim(-190, z_buck + 120)
    ax_f.set_xlim(y0 - 210, y1 + 210)
    ax_f.set_ylim(-190, z_buck + 120)

    # --- 凡例 ---------------------------------------------------------
    handles = [Line2D([], [], marker="s", ls="", markersize=6.4,
                      markerfacecolor=c, markeredgecolor="none",
                      label=f"{n}. {nm}") for n, nm, c, _ in SYSTEMS]
    handles.append(Line2D([], [], marker="s", ls="", markersize=6.4,
                          markerfacecolor=FASTEN, markeredgecolor="none",
                          label="締結要素（ボルト・ナット・リベット）"))
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=FS - 0.2,
               frameon=False, handletextpad=0.5, columnspacing=1.1,
               labelspacing=0.34, bbox_to_anchor=(0.5, -0.004))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs", "fig1.png")
    fig.savefig(out, dpi=400, facecolor="white")
    print(out)
    print(f"機体 {x1 - x0:.0f} x {y1 - y0:.0f} x {z_top:.0f} / バケツ上面 {z_buck:.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
