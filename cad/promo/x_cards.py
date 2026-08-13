"""X（旧 Twitter）に貼る画像を作る。

    ../.venv/bin/python promo/x_cards.py            # 全部
    ../.venv/bin/python promo/x_cards.py --only topo

出力は `promo/out/x-ready/`（gpusim の同名フォルダと同じ流儀）。
X のタイムラインは 16:9 で切られるので **1600×900** に揃える。

⚠ **数字は必ず生成物から読む。** 手で書いた値はすぐ腐る。
  - 板の外形    … `out/topo/<板>.json`（最適化後）と `out/topo/_base.json`（元）
  - 質量        … 元の質量 × 面積比。板厚も密度も変わらないので面積比が質量比になる
  - 機体の寸法  … `tr_params` から読む

⚠ 図の中で日本語を使うので **Noto Sans CJK JP** を明示する。指定しないと
  matplotlib が豆腐（□）で描き、書き出すまで気づけない。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.font_manager import FontProperties  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path  # noqa: E402

TOPO = os.path.join(ROOT, "out", "topo")
STILLS = os.path.join(HERE, "out", "stills")
OUT = os.path.join(HERE, "out", "x-ready")

# 配色。X は暗い背景で見る人が多いので、地を暗くして線を明るく置く
BG = "#0b0e14"
FG = "#e8ecf2"
MUTED = "#7d8896"
ACCENT = "#4fc3f7"      # 最適化後の板
GHOST = "#3a4250"       # 元の外形
WARN = "#ff6b6b"

JP = "Noto Sans CJK JP"
plt.rcParams["font.family"] = JP
plt.rcParams["axes.unicode_minus"] = False
BOLD = FontProperties(family=JP, weight="bold")


def poly_area(poly: list[list[float]]) -> float:
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2


def plate_rows() -> list[dict]:
    """最適化した板を、元の外形・最適化後の外形・質量つきで集める。"""
    base = json.load(open(os.path.join(TOPO, "_base.json")))
    rows = []
    for name in sorted(os.listdir(TOPO)):
        if not name.endswith(".json") or name.startswith("_"):
            continue
        key = name[:-5]
        b = base.get(key)
        if not b:
            continue
        d = json.load(open(os.path.join(TOPO, name)))
        a_base = poly_area(b["outline"])
        a_opt = d["area"]
        m_base = b["part"]["mass"]
        rows.append(
            dict(
                name=key,
                base=b["outline"],
                outer=d["outer"],
                holes=d.get("holes", []),
                a_base=a_base,
                a_opt=a_opt,
                m_base=m_base,
                m_opt=m_base * a_opt / a_base,
            )
        )
    # 削った量の多い順。X では「効いた板」から目に入ってほしい
    rows.sort(key=lambda r: r["m_base"] - r["m_opt"], reverse=True)
    return rows


def compound_path(outer: list, holes: list) -> Path:
    """外形＋穴を 1 つの Path にする（穴は逆回りにして塗りを抜く）。"""
    verts: list[tuple[float, float]] = []
    codes: list[int] = []

    def ring(pts, reverse=False):
        pp = list(reversed(pts)) if reverse else list(pts)
        verts.append((pp[0][0], pp[0][1]))
        codes.append(Path.MOVETO)
        for x, y in pp[1:]:
            verts.append((x, y))
            codes.append(Path.LINETO)
        verts.append((pp[0][0], pp[0][1]))
        codes.append(Path.CLOSEPOLY)

    def signed(pts):
        s = 0.0
        for i in range(len(pts)):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % len(pts)]
            s += x1 * y2 - x2 * y1
        return s

    ring(outer, reverse=signed(outer) < 0)
    for h in holes:
        if len(h) >= 3:
            ring(h, reverse=signed(h) > 0)
    return Path(verts, codes)


# --------------------------------------------------------------- 1. トポロジー
def card_topo(path: str) -> None:
    rows = plate_rows()
    m0 = sum(r["m_base"] for r in rows) * 1000
    m1 = sum(r["m_opt"] for r in rows) * 1000

    # ---- 板のグリッド（3 列 × 2 段。6 枚だけを大きく見せる）
    # ⚠ 18 枚を並べると 1 枚が小さすぎて、穴の入り方も外形の変わり方も読めない。
    # ⚠ **細長い板は図から外す**（縦横比 4 超）。正方形のマスに入れると
    #   ただの縦棒になり、「形が変わった」ことが伝わらない。削減量の順位は
    #   高いが（yaw_arm_f/r）、伝わらない絵を大きく載せる意味がない。
    #   合計の数字は 18 枚ぶんのままなので、嘘にはならない。
    def slim(r: dict) -> bool:
        xs = [p[0] for p in r["base"]]
        ys = [p[1] for p in r["base"]]
        wpl, hpl = max(xs) - min(xs), max(ys) - min(ys)
        return max(wpl, hpl) / max(min(wpl, hpl), 1e-6) > 4

    shown = [r for r in rows if not slim(r)][:6]

    # 全部を同じ縮尺・同じ画面で見せる（`heat` と同じ流儀）
    span = 0.0
    for r in shown:
        xs = [p[0] for p in r["base"]]
        ys = [p[1] for p in r["base"]]
        span = max(span, (max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2)
    span *= 1.08

    with plt.style.context("default"):
        plt.rcParams["font.family"] = JP
        plt.rcParams["axes.unicode_minus"] = False
        fig, axes = plt.subplots(2, 3, figsize=(16, 9), dpi=100, constrained_layout=True)
        fig.suptitle(
            f"板 18 枚を荷重の形に切り直した: {m0 / 1000:.1f} kg → {m1 / 1000:.1f} kg"
            f"（−{m0 - m1:,.0f} g・−{(m0 - m1) / m0 * 100:.0f} %）\n"
            "破線 = 元の外形　塗り = 最適化後　高専ロボコン2026 雑巾投擲選手権 TR",
            fontsize=21, linespacing=1.9)

        for i, r in enumerate(shown):
            ax = axes[i // 3, i % 3]
            ax.add_patch(PathPatch(compound_path(r["base"], []), facecolor="none",
                                   edgecolor="0.45", lw=1.4, ls=(0, (5, 4))))
            ax.add_patch(PathPatch(compound_path(r["outer"], r["holes"]),
                                   facecolor="#4c72b0", edgecolor="#25406b", lw=1.0))
            cut = (r["m_base"] - r["m_opt"]) * 1000
            ax.set_title(f"{r['name']}\n"
                         f"{r['m_base'] * 1000:.0f} g → {r['m_opt'] * 1000:.0f} g"
                         f"（−{cut:.0f} g）", fontsize=13)
            ax.set_aspect("equal")
            ax.set_xlim(-span, span)
            ax.set_ylim(-span, span)
            if i // 3 == 1:
                ax.set_xlabel("x [mm]", fontsize=11)
            if i % 3 == 0:
                ax.set_ylabel("y [mm]", fontsize=11)

        fig.savefig(path, facecolor="white")
        plt.close(fig)
    print(f"  -> {path}")


# --------------------------------------------------------------- 1b. ヒートマップ
# 図に出す板。形の変わり方が読めて、解くのが速いものを選ぶ
HEAT_PLATES = ["yaw_side_R", "pitch_side_R", "pitch_side_L", "mount_brk_fl"]


def card_heat(path: str, iters: int = 80) -> None:
    """密度場と von Mises 応力のヒートマップ。matplotlib 素のまま。

    ⚠ **`out/topo/*.json` を書き換えない。** 輪郭は生成物ではなく CAD への
      入力で、`topo_opt.py` を回すと更新されてしまう（screws.json まで
      古くなる）。ここは解いた結果を**メモリの中だけ**で使い、重ねる輪郭は
      凍結済みの JSON から読む。

    ⚠ `topo_cache` が CAD より古いと、解く境界条件が実機とずれる。
      その時は先に `scripts/topo_cache.py` を回すこと（警告が出る）。
    """
    import numpy as np

    import topo_opt

    regions, _skipped, _outside = topo_opt.collect()
    by_name = {r.name: r for r in regions}

    picked = []
    for nm in HEAT_PLATES:
        reg = by_name.get(nm)
        if reg is None:
            print(f"  ⚠ {nm} は設計領域に無い（飛ばす）")
            continue
        # ⚠ **`simp.solve` を直に呼ばない。** 凍結してある輪郭は
        #   `solve_auto`（分断していたら太らせ、細すぎたらフィルタを広げる
        #   ループ）が出したもので、生の 1 回解きとは違う形になる。実際
        #   pitch_side_L は密度場がトラス、凍結輪郭は塊で、重ねると
        #   まったく合わなかった。図に出すなら**同じ経路で解く**。
        res, _out, _bad, log = topo_opt.solve_auto(reg, iters=iters)
        frozen = json.load(open(os.path.join(TOPO, f"{nm}.json")))
        picked.append((nm, reg, res, frozen))
        print(f"  解いた {nm}: 体積率 {res.vol_frac:.3f} / "
              f"コンプライアンス {res.compliance:.1f} N·mm"
              + (f" / {len(log)} 回やり直し" if log else ""))
    if not picked:
        raise SystemExit("解ける板が無い")

    # 全部の板を同じ縮尺・同じ画面で見せる。板ごとに窓を変えると
    # 段が揃わず、板の大小も分からなくなる
    span = max(max(reg.w, reg.h) for _nm, reg, _res, _f in picked) / 2 * 1.06

    # ここだけ matplotlib の既定の見た目に戻す（無機質な解析図にする）
    with plt.style.context("default"):
        plt.rcParams["font.family"] = JP
        plt.rcParams["axes.unicode_minus"] = False
        fig, axes = plt.subplots(2, len(picked), figsize=(16, 9), dpi=100,
                                 constrained_layout=True)
        if len(picked) == 1:
            axes = axes.reshape(2, 1)
        # ⚠ 注記を figure の下端（y=0.01 など）に置かない。constrained_layout は
        #   fig.text の場所を知らないので、x 軸ラベルとカラーバーの数字に重なる。
        fig.suptitle("トポロジー最適化（SIMP 法）: 密度場と von Mises 応力\n"
                     "白線・灰線 = 実際に部品になった輪郭　"
                     "高専ロボコン2026 雑巾投擲選手権 TR",
                     fontsize=21, linespacing=1.9)

        for col, (nm, reg, res, frozen) in enumerate(picked):
            ext = [-reg.w / 2, reg.w / 2, -reg.h / 2, reg.h / 2]
            out = np.asarray(frozen["outer"] + [frozen["outer"][0]])

            ax = axes[0, col]
            im0 = ax.imshow(res.dens, origin="lower", extent=ext, cmap="viridis",
                            vmin=0.0, vmax=1.0, interpolation="nearest")
            ax.plot(out[:, 0], out[:, 1], "-", color="w", lw=1.1)
            for hole in frozen.get("holes", []):
                h = np.asarray(list(hole) + [hole[0]])
                ax.plot(h[:, 0], h[:, 1], "-", color="w", lw=0.8)
            ax.set_title(f"{nm}\n{reg.mat} t{reg.t:.0f} / 体積率 {res.vol_frac:.2f}",
                         fontsize=13)

            # ⚠ von Mises は**密度 1 換算**なので、空の要素で発散する
            #   （実測 1.3e7 MPa）。材料がある所だけ見せ、上限は 99 パーセンタイル。
            ax = axes[1, col]
            vm = np.where(res.dens > 0.5, res.vm, np.nan)
            hi = float(np.nanpercentile(vm, 99)) if np.isfinite(vm).any() else 1.0
            im1 = ax.imshow(vm, origin="lower", extent=ext, cmap="inferno",
                            vmin=0.0, vmax=hi, interpolation="nearest")
            ax.plot(out[:, 0], out[:, 1], "-", color="0.35", lw=1.0)
            ax.set_title(f"最大 {np.nanmax(vm):.2f} MPa", fontsize=13)
            fig.colorbar(im1, ax=ax, shrink=0.9, pad=0.02)

            for row in (0, 1):
                a = axes[row, col]
                a.set_aspect("equal")
                a.set_xlim(-span, span)
                a.set_ylim(-span, span)
                if row == 1:
                    a.set_xlabel("x [mm]", fontsize=11)
                if col == 0:
                    a.set_ylabel(("密度 ρ [-]" if row == 0 else "von Mises [MPa]")
                                 + "\ny [mm]", fontsize=12)

        fig.colorbar(im0, ax=axes[0, :].tolist(), shrink=0.9, pad=0.02)
        fig.savefig(path, facecolor="white")
        plt.close(fig)
    print(f"  -> {path}")


# --------------------------------------------------------------- 2. 一気通貫
def card_pipeline(path: str) -> None:
    """1 つの CAD から、検査・シミュレーター・映像が出ていることを見せる。"""
    import tr_params as P  # noqa: E402

    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor=BG)
    fig.subplots_adjust(0, 0, 1, 1)

    fig.text(0.05, 0.905, "CAD を直すと、全部が追従する",
             color=FG, fontsize=40, fontproperties=BOLD, va="center")

    # 写真（プロモのレンダリング）
    ax = fig.add_axes([0.05, 0.155, 0.44, 0.63])
    ax.set_facecolor(BG)
    ax.axis("off")
    hero = os.path.join(STILLS, "hero.png")
    if os.path.exists(hero):
        img = plt.imread(hero)
        # 16:9 の枠に収まるよう中央を切る
        h, w = img.shape[:2]
        want = 1.30
        nw = int(min(w, h * want))
        nh = int(nw / want)
        img = img[(h - nh) // 2 : (h + nh) // 2, (w - nw) // 2 : (w + nw) // 2]
        ax.imshow(img)

    boxes = [
        (f"CAD  1,224 ソリッド・{len(P.MOTORS)} 軸", True),
        ("検査 26 項目", False),
        ("DXF ／ STL ／ STEP", False),
        ("試合シミュレーター  全 12 関節", False),
        ("プロモ映像  材質 28 種", False),
    ]
    bx, by = 0.545, 0.715
    for head, first in boxes:
        fig.text(bx, by, "▍" + head, color=ACCENT if first else FG, fontsize=25,
                 fontproperties=BOLD)
        if head != boxes[-1][0]:
            fig.text(bx + 0.006, by - 0.062, "↓", color=GHOST, fontsize=19)
        by -= 0.128

    fig.text(0.05, 0.055, "外形 964 × 803 × 1191 mm ／ 砲口高さ 1,000 mm",
             color=MUTED, fontsize=14)
    fig.text(0.955, 0.055, "高専ロボコン2026  雑巾投擲選手権  TR", color=MUTED, fontsize=14, ha="right")

    fig.savefig(path, facecolor=BG)
    plt.close(fig)
    print(f"  -> {path}")


# --------------------------------------------------------------- 3. 工夫あつめ
def card_details(path: str) -> None:
    panels = [
        ("lidar_front.png", "LiDAR の水平出し座"),
        ("display.png", "点いている表示器"),
        ("grabber.png", "机に触れずに攫うフォーク"),
        ("turret.png", "対向ローラー射出"),
    ]

    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor=BG)
    fig.subplots_adjust(0, 0, 1, 1)

    fig.text(0.045, 0.925, "図面に出ない所を、実体で作る", color=FG, fontsize=40,
             fontproperties=BOLD, va="center")

    for i, (fname, head) in enumerate(panels):
        cx, cy = i % 2, i // 2
        ax = fig.add_axes([0.045 + cx * 0.478, 0.455 - cy * 0.375, 0.44, 0.30])
        ax.set_facecolor(BG)
        ax.axis("off")
        p = os.path.join(STILLS, fname)
        if os.path.exists(p):
            img = plt.imread(p)
            h, w = img.shape[:2]
            want = 2.35
            nh = int(min(h, w / want))
            nw = int(nh * want)
            img = img[(h - nh) // 2 : (h + nh) // 2, (w - nw) // 2 : (w + nw) // 2]
            ax.imshow(img)
        fig.text(0.045 + cx * 0.478, 0.455 - cy * 0.375 + 0.315, "▍" + head,
                 color=FG, fontsize=22, fontproperties=BOLD)

    fig.text(0.955, 0.045, "高専ロボコン2026  雑巾投擲選手権  TR", color=MUTED, fontsize=14, ha="right")
    fig.savefig(path, facecolor=BG)
    plt.close(fig)
    print(f"  -> {path}")


CARDS = {"topo": card_topo, "heat": card_heat,
         "pipeline": card_pipeline, "details": card_details}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="カンマ区切り: " + " / ".join(CARDS))
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    want = [s for s in args.only.split(",") if s] or list(CARDS)
    for key in want:
        if key not in CARDS:
            raise SystemExit(f"知らないカード: {key}（{' / '.join(CARDS)}）")
        print(f"=== {key} ===")
        CARDS[key](os.path.join(OUT, f"{key}.png"))
    print(f"できた: {OUT}")


if __name__ == "__main__":
    main()
