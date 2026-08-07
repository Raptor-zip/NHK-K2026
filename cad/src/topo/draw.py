"""トポロジー最適化の図 — **人間の鑑賞用ではなく、Claude が読んで直すための PNG**.

この 4 つの関数が出す画像の読み手は マルチモーダル LLM で、
「次にどの板の frac / rmin をどちらへ動かすか」をこの絵だけで決める。
だから普通のグラフとは要件が違う:

* 文字は大きく（本文 13pt、目盛り 12pt。図の高さの 2% を下回らないよう
  figure の高さを 8 inch で頭打ちにしている）。
* **色だけで情報を伝えない**。判断に要る数字（体積率・最小部材幅・最大応力・
  削減量・不良の有無）は全部そのまま図の中に文字で書く。
* 背景は白。⚠ 半透明の重ね（alpha）は使わない。Claude は重なった色を
  「別の値」と読み違える。重ねたい所は白フチ（同じ形を白の太線で先に描く）で
  抜いてコントラストを稼ぐ。
* 画像は長辺 1200〜1600px。dpi を上げても読み取り精度は上がらず、
  読み込みコスト（トークン）だけ増える。

⚠ 記号は 3 つの図で必ず同じにすること。図ごとに色や形が変わると
  Claude が「ヒートマップのこの青」と「平面図のこの青」を対応させられない。
  そのために境界条件の描画は `draw_bc()` 一本に集約してある。

日本語フォントについて
----------------------
matplotlib の既定フォント（DejaVu Sans）は日本語を持たないので、そのままだと
全部 豆腐（□）になり Claude が読めない。起動時に `Noto Sans CJK JP` などを
探して `font.family` に差し込む（`_setup_font()`）。**見つからなかった場合は
`JP_OK = False` になり、すべてのラベルが英数字にフォールバックする**
（`_t("固定", "FIXED")` の第 2 引数）。豆腐のまま出ることはない。

⚠ CJK フォントは U+2212（本物のマイナス記号）を持たないことがあり、
  matplotlib の既定 `axes.unicode_minus=True` のままだと**軸の負の目盛りだけ
  豆腐になる**。`axes.unicode_minus=False` にして ASCII の '-' を使わせる。

⚠ フォールバック時に落ちないのは**この モジュールが書く文言だけ**。
  `Region.name` や `check` の不良メッセージが日本語なら、それは豆腐になる。
  日本語フォントの無い環境で回すなら板名を英数字にすること。

単位は core と同じ（長さ mm / 力 N / 応力 MPa / 質量 kg）。
"""

from __future__ import annotations

import logging
import os
import re
from contextlib import contextmanager
from typing import Sequence

import matplotlib

matplotlib.use("Agg")  # ⚠ pyplot を import する前に。ヘッドレスで落ちないように

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib.colors import LogNorm, Normalize  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle as MplCircle  # noqa: E402
from matplotlib.patches import Patch, Polygon, Rectangle  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

from .core import (E_ALU, RIB_MIN, SIGMA_ALLOW, Outline, Region,  # noqa: E402
                   TopoResult)

# ---------------------------------------------------------------------------
# 日本語フォント
# ---------------------------------------------------------------------------

# `fc-list :lang=ja` で出てくる代表的なものを優先度順に。
# 最後の Droid Sans Fallback は字形が中華フォントだが豆腐よりはるかにまし。
_JP_FONTS = (
    "Noto Sans CJK JP",
    "Noto Sans JP",
    "IPAexGothic",
    "IPAGothic",
    "IPAPGothic",
    "TakaoPGothic",
    "VL PGothic",
    "Noto Sans CJK SC",
    "Droid Sans Fallback",
)


def _setup_font() -> bool:
    """日本語の出るフォントを探して rcParams に差す。見つからなければ False。"""
    # ⚠ axes.unicode_minus は日本語フォントの有無にかかわらず切る。
    #   CJK フォントに U+2212 が無いと負の目盛りだけ豆腐になる。
    plt.rcParams["axes.unicode_minus"] = False
    have = {f.name for f in fm.fontManager.ttflist}
    for name in _JP_FONTS:
        if name in have:
            plt.rcParams["font.family"] = [name, "DejaVu Sans"]
            return True
    return False


JP_OK: bool = _setup_font()


def _t(ja: str, en: str) -> str:
    """日本語フォントがあれば日本語、無ければ英数字。豆腐は絶対に出さない。"""
    return ja if JP_OK else en


# ---------------------------------------------------------------------------
# 共通の記号（⚠ 3 つの図で同一。ここを変えたら全部変わる）
# ---------------------------------------------------------------------------
C_FIX = "#1565c0"     # 青 斜線ハッチの矩形 : 固定（完全拘束）
C_LOAD = "#d32f2f"    # 赤 矢印             : 荷重（長さ ∝ 力）
C_SOLID = "#2e7d32"   # 緑 枠               : 座面（密度 1 固定）
C_VOID = "#111111"    # 黒枠＋白抜きの円     : 逃げ（密度 0 固定）
C_OVER = "#ff2d95"    # マゼンタ            : 許容応力 超過
C_MAT = "#c9c9c9"     # 材料のグレー
C_BOX = "#8a8a8a"     # 元の外接矩形（破線）

# 図中の文字サイズ。⚠ 12pt を下回らせないこと（Claude が読めない）。
FS_BASE = 13.0
FS_TICK = 12.0
FS_TITLE = 16.0

# 画像の長辺 [px] の目安。これを超えると読み込みコストだけ増える。
PX_LONG = 1400.0

# --- 図の骨組みの寸法 [inch] ------------------------------------------------
# ⚠ figure の高さは 8.0 inch で頭打ちにする。12pt の文字が画像高さの 2% を
#   切らないための上限（12/72 / 8.0 = 2.08%）。ここを緩めると字が相対的に
#   小さくなって Claude が読めなくなる。
FIG_H_MAX = 8.0
FIG_W_MIN, FIG_W_MAX = 11.0, 15.0
TOP_IN = 0.55        # suptitle
TITLE_IN = 0.44      # 各パネルのタイトル
XLAB_IN = 0.62       # x 目盛り + x ラベル
YLAB_IN = 0.78       # y 目盛り + y ラベル
CBAR_IN = 1.05       # カラーバー + その目盛り + ラベル
GAP_IN = 0.30        # パネル同士の縦の隙間
COL_GAP_IN = 0.90    # ⚠ 横の隙間。狭いと左パネルのカラーバーのラベル（"log E"）
                     #   と右パネルの y 軸ラベル（"y [mm]"）が重なって
                     #   どちらも読めなくなる。0.55 では足りなかった
LEGEND_IN = 0.52     # 下端の凡例
LINE_IN = 0.29       # 情報欄 1 行（13pt × 行間 1.5）
N_INFO = 5           # 情報欄の行数（列ごと）
INFO_IN = LEGEND_IN + 0.16 + N_INFO * LINE_IN + 0.10
PW_MIN = 3.1         # パネルの最小幅。⚠ これ未満だとタイトルが枠から溢れる

# ファイル名の決まり。
# ⚠ `scripts/topo_opt.py` が実際に書いている名前と**一字一句合わせる**こと。
#   ここがずれると review.md から画像が見えなくなる（リンク切れの Markdown を
#   Claude に渡しても、そこに何が写っていたか分からない）。
#     画像 … <ROOT>/out/topo/<板名>_{shape,load}.png と sheet.png
#     md   … <ROOT>/out/topo.md      → 相対で "topo/..." になる
MD_IMG_DIR = "topo"
SHEET_NAME = "sheet.png"


def _slug(name: str) -> str:
    """ファイル名に使える形へ。

    ⚠ 置き換えるのは**パス区切りだけ**。空白や記号まで潰すと、板名をそのまま
      使って PNG を書いている呼び出し側とファイル名がずれる。
    """
    return re.sub(r"[/\\\x00-\x1f]+", "_", name.strip()) or "plate"


def shape_name(name: str) -> str:
    """最適化後の平面図の PNG 名。"""
    return f"{_slug(name)}_shape.png"


def heat_name(name: str) -> str:
    """荷重ヒートマップの PNG 名（⚠ "_load" であって "_heat" ではない）。"""
    return f"{_slug(name)}_load.png"


# ---------------------------------------------------------------------------
# 小道具
# ---------------------------------------------------------------------------


def _ensure_dir(path: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)


def _num(v: float, nd: int = 1) -> str:
    """NaN / inf を書いても図が壊れないようにする。

    ⚠ 桁が大きい値は指数表記に落とす。ソルバが発散したときの 1e155 を
      そのまま桁区切りで書くと、情報欄の 1 行が図幅を突き抜けて
      隣の列を潰す（実際にやらかした）。
    """
    if v is None or not np.isfinite(v):
        return "n/a"
    if abs(v) >= 1e6:
        return f"{v:.3e}"
    return f"{v:,.{nd}f}"


def _clean(a: np.ndarray, fill: float = 0.0) -> np.ndarray:
    """NaN / inf を潰した float 配列。⚠ 生の res.* を直接 imshow しないこと。"""
    x = np.asarray(a, dtype=float)
    return np.nan_to_num(x, nan=fill, posinf=fill, neginf=fill)


# ⚠ adjustable="datalim" は描画のたびに
#   「Ignoring fixed x limits to fulfill fixed data aspect」を吐く。
#   これは warnings ではなく **logger** 経由なので warnings.filterwarnings では
#   消せない。狙い通りの動作なので、必要な区間だけこのロガーを黙らせる。
_MPL_LOG = logging.getLogger("matplotlib.axes._base")


@contextmanager
def _quiet_mpl():
    lvl = _MPL_LOG.level
    _MPL_LOG.setLevel(logging.ERROR)
    try:
        yield
    finally:
        _MPL_LOG.setLevel(lvl)


def _extent(reg: Region) -> tuple[float, float, float, float]:
    """imshow 用。板ローカル mm。原点は板中心。"""
    return (-reg.w / 2, reg.w / 2, -reg.h / 2, reg.h / 2)


def _outline_mass(res: TopoResult, out: Outline | None) -> float:
    """輪郭の面積から出した実質量 [kg]。⚠ 面積が無ければ密度場から出す。"""
    reg = res.region
    if out is not None and out.area > 0:
        return float(out.area) * reg.t * reg.rho_mat
    return res.mass()


# ---------------------------------------------------------------------------
# 境界条件の描画（⚠ heatmap_png / shape_png / sheet_png が共有）
# ---------------------------------------------------------------------------


def bc_arrow_len(reg: Region) -> float:
    """荷重矢印の最大長 [mm].

    ⚠ 単純に max(w, h) の比にすると、600x60 のような細長い板で矢印が
      板の何倍にもなって図が縦に潰れる。短辺基準にしつつ、正方形に近い板で
      矢印が消えないよう長辺の 5% を下限に置く。
    """
    return max(0.18 * min(reg.w, reg.h), 0.05 * max(reg.w, reg.h))


def bc_margin(reg: Region) -> tuple[float, float]:
    """矢印とラベルがはみ出しても切れないようにする表示余白 [mm]。

    ⚠ 矢の長さ a に加えてラベルを 0.3a 外へ逃がしているので、1.45 倍見る。
    """
    a = bc_arrow_len(reg)
    return 0.06 * reg.w + 1.45 * a, 0.06 * reg.h + 1.45 * a


def _rect_patch(ax, r, *, ec: str, hatch: str | None, lw: float, z: float):
    """白フチ → 本体 の 2 枚重ねで矩形を描く（どんな背景でも枠が見える）。"""
    x0, y0, x1, y1 = r
    w, h = x1 - x0, y1 - y0
    ax.add_patch(Rectangle((x0, y0), w, h, fill=False, edgecolor="white",
                           linewidth=lw + 2.6, zorder=z))
    ax.add_patch(Rectangle((x0, y0), w, h, fill=False, edgecolor=ec,
                           linewidth=lw, hatch=hatch, zorder=z + 0.1))


class _LabelPlacer:
    """注記が重ならないよう、置いた矩形を覚えて上下にずらす.

    ⚠ 完璧な回避は狙わない。**座面と荷重が同じ座に来る**（ねじ座に力が
      掛かるのは普通に起きる）ときに「座面」と「1,800 N」が完全に重なって
      両方読めなくなるのを防げれば十分。

    ⚠ 文字の大きさはデータ座標へ換算して見積もる。軸の箱 [inch] は
      `ax.get_position()` から取れるが、`adjustable="datalim"` だと描画時に
      表示範囲がさらに広がるので、この見積もりは**やや小さめ**に出る。
      その分ずらし量も控えめになるが、完全な重なりは避けられる。
    """

    def __init__(self, ax, fs: float):
        fig = ax.figure
        # ⚠ get_position() は内部で apply_aspect() を呼ぶので、ここで
        #   datalim の「範囲を広げた」ログが出る。狙い通りなので黙らせる。
        #   （副作用として、この時点で表示範囲は確定済み＝見積もりが正確になる）
        with _quiet_mpl():
            bb = ax.get_position()
        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        self.wpi = (x1 - x0) / max(bb.width * fig.get_figwidth(), 1e-6)
        self.hpi = (y1 - y0) / max(bb.height * fig.get_figheight(), 1e-6)
        self.fs = fs
        self.boxes: list[tuple[float, float, float, float]] = []

    def _size(self, s: str) -> tuple[float, float]:
        # 全角は fs pt、半角は 0.55*fs pt とみなす
        w_in = sum(1.0 if ord(c) > 0x2000 else 0.55 for c in s) * self.fs / 72
        return (w_in + 0.10) * self.wpi, (self.fs / 72 * 1.6) * self.hpi

    @staticmethod
    def _hit(a, b) -> bool:
        return (abs(a[0] - b[0]) * 2 < (a[2] + b[2])
                and abs(a[1] - b[1]) * 2 < (a[3] + b[3]))

    def place(self, x: float, y: float, s: str, ha: str, va: str) -> float:
        """重ならない y を返す（x は動かさない）。"""
        w, h = self._size(s)
        cx = x + (w / 2 if ha == "left" else (-w / 2 if ha == "right" else 0.0))
        cy0 = y + (h / 2 if va == "bottom" else (-h / 2 if va == "top" else 0.0))
        for k in range(9):
            dy = h * 1.08 * ((k + 1) // 2) * (1.0 if k % 2 else -1.0)
            box = (cx, cy0 + dy, w, h)
            if not any(self._hit(box, b) for b in self.boxes):
                self.boxes.append(box)
                return y + dy
        self.boxes.append((cx, cy0, w, h))
        return y


def _label(ax, x: float, y: float, s: str, color: str, fs: float, z: float,
           ha: str = "center", va: str = "center",
           placer: _LabelPlacer | None = None) -> None:
    """不透明な白箱つきの注記。⚠ alpha は使わない（読み違えの元）。"""
    if placer is not None:
        y = placer.place(x, y, s, ha, va)
    ax.text(x, y, s, color=color, fontsize=fs, ha=ha, va=va, zorder=z,
            clip_on=False, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.18", facecolor="white",
                      edgecolor=color, linewidth=0.9))


def draw_bc(ax, reg: Region, *, fs: float = 11.0, label: bool = True,
            lw: float = 2.0, max_label: int = 8) -> None:
    """境界条件を描く。**3 つの図すべてがこれを呼ぶこと**。

    固定 = 青の斜線ハッチ矩形 ／ 荷重 = 赤の矢印（長さ ∝ 力）＋ "NNN N"
    座面 = 緑の枠           ／ 逃げ = 白抜きの円

    ⚠ `label=False` にできるのは一覧図（sheet_png）だけ。主図では必ず
      文字を出す。色だけで区別させると Claude が取り違える。
    """
    z = 6.0
    pl = _LabelPlacer(ax, fs) if label else None

    # ⚠ 矩形の注記は**左下の隅**に寄せる。中心に置くと、同じ座に荷重が
    #   掛かっている板（座面＝荷重点は普通に起きる）で「座面」と「NNN N」が
    #   ぶつかりやすい。隅と矢の尾なら初期位置からしてずれる。
    def _corner(r) -> tuple[float, float]:
        pad = 0.02 * min(reg.w, reg.h)
        return r[0] + pad, r[1] + pad

    # --- 座面（密度 1 固定）: 緑の枠 ---------------------------------------
    for i, r in enumerate(reg.solid):
        _rect_patch(ax, r, ec=C_SOLID, hatch=None, lw=lw, z=z)
        if label and i < max_label:
            cx, cy = _corner(r)
            _label(ax, cx, cy, _t("座面", "SOLID"), C_SOLID, fs, z + 1,
                   ha="left", va="bottom", placer=pl)

    # --- 固定: 青の斜線ハッチ ----------------------------------------------
    for i, r in enumerate(reg.fixed):
        _rect_patch(ax, r, ec=C_FIX, hatch="///", lw=lw, z=z + 0.2)
        if label and i < max_label:
            cx, cy = _corner(r)
            _label(ax, cx, cy, _t("固定", "FIXED"), C_FIX, fs, z + 1,
                   ha="left", va="bottom", placer=pl)

    # --- 逃げ（密度 0 固定）: 白抜きの円 ------------------------------------
    for i, (cx, cy, rr) in enumerate(reg.void):
        ax.add_patch(MplCircle((cx, cy), rr, facecolor="white",
                               edgecolor=C_VOID, linewidth=lw, zorder=z + 0.4))
        if label and i < max_label:
            _label(ax, cx, cy, _t("逃げ", "VOID"), C_VOID, fs, z + 1, placer=pl)

    # --- 荷重: 赤の矢印（長さ ∝ 力）----------------------------------------
    if reg.loads:
        mags = [float(np.hypot(fx, fy)) for _, fx, fy in reg.loads]
        fmax = max(mags) or 1.0
        amax = bc_arrow_len(reg)
        for (r, fx, fy), mag in zip(reg.loads, mags):
            cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
            # 力ゼロの座は向きが決まらないので × 印だけ置く
            if mag <= 0:
                ax.plot([cx], [cy], marker="x", color=C_LOAD, ms=10, mew=2.4,
                        zorder=z + 1)
                continue
            ln = amax * (0.35 + 0.65 * mag / fmax)  # 小さい力も見えるよう下駄
            ux, uy = fx / mag, fy / mag
            # ⚠ 矢尻を作用点に落とす（板を「押している」向きに読ませる）
            tx, ty = cx - ux * ln, cy - uy * ln
            for col, w_ in ((("white"), lw + 2.4), ((C_LOAD), lw + 0.4)):
                ax.annotate("", xy=(cx, cy), xytext=(tx, ty),
                            annotation_clip=False, zorder=z + 0.5,
                            arrowprops=dict(arrowstyle="-|>", color=col,
                                            linewidth=w_, mutation_scale=16 + 4 * lw,
                                            shrinkA=0, shrinkB=0))
            if label:
                # ⚠ ラベルは矢の**尾のさらに外側**へ、力の大小によらず一定の
                #   距離だけ逃がす。ln に比例させると小さい荷重（矢が短い）で
                #   ラベルが作用点に重なる。
                off = 0.30 * amax
                _label(ax, tx - ux * off, ty - uy * off,
                       f"{mag:,.0f} N", C_LOAD, fs, z + 1, placer=pl)


def bc_legend_handles() -> tuple[list, list[str]]:
    """記号の凡例。⚠ どの図でも同じ並びで出すこと。"""
    hs = [
        Patch(facecolor="none", edgecolor=C_FIX, hatch="///", linewidth=2.0),
        Line2D([0], [0], color=C_LOAD, lw=3.0, marker=">", ms=9,
               markerfacecolor=C_LOAD),
        Patch(facecolor="none", edgecolor=C_SOLID, linewidth=2.0),
        Line2D([0], [0], color=C_VOID, lw=2.0, marker="o", ms=10,
               markerfacecolor="white", linestyle="none"),
        Line2D([0], [0], color=C_BOX, lw=2.0, linestyle="--"),
    ]
    # ⚠ 凡例は 5 個を横 1 列に並べる。文言を長くすると figure 幅 11 inch に
    #   収まらず両端が切れる（実際に切れた）。短いまま保つこと。
    ls = [
        _t("固定", "FIXED"),
        _t("荷重(長さ∝力)", "LOAD (len∝F)"),
        _t("座面(密度1)", "SOLID (d=1)"),
        _t("逃げ(密度0)", "VOID (d=0)"),
        _t("元の外接矩形", "orig. bbox"),
    ]
    return hs, ls


# ---------------------------------------------------------------------------
# 図の骨組み
# ---------------------------------------------------------------------------


class _Layout:
    """`_two_panel_fig` の戻り値。パネル幅 pw はタイトルの長短を決めるのに使う。"""

    __slots__ = ("fig", "ax_a", "ax_b", "dpi", "pw", "ph")

    def __init__(self, fig, ax_a, ax_b, dpi: float, pw: float, ph: float):
        self.fig, self.ax_a, self.ax_b = fig, ax_a, ax_b
        self.dpi, self.pw, self.ph = dpi, pw, ph


def _two_panel_fig(reg: Region) -> _Layout:
    """主図＋副図の 2 枚を持つ figure を作る。寸法は全部 inch で積み上げる。

    ⚠ 板の縦横比で並べ方を変える。横長の板（600x60）を横に 2 枚並べると
      1 枚が幅 8mm の帯になって何も読めない。横長 → 縦積み、縦長/正方 → 横並び。

    ⚠ figure サイズを「タイトル・軸ラベル・カラーバー・情報欄・凡例」の
      inch 積み上げから決めているのは、以前 figure 比率で決めて
      **情報欄の最終行と凡例が重なった／縦長の板でタイトルが枠から溢れた**
      から。比率で置くと縦横比が変わった瞬間に破綻する。
    """
    ar = reg.w / max(reg.h, 1e-9)
    mx, my = bc_margin(reg)
    # ⚠ 表示に使う縦横比は「余白込み」。板の w/h で組むと矢印のはみ出し分だけ
    #   箱と中身の比がずれて、余計な空白が出る。
    ar_pad = (reg.w + 2 * mx) / max(reg.h + 2 * my, 1e-9)

    # ⚠ 縦積みにするかどうかは **ar_pad が 2.2 以上か** で決める。
    #   高さ 8 inch の枠で 2 段に積むと 1 段の高さは 1.4 inch しか取れず、
    #   ar_pad が 2.2 を切るとパネル幅がタイトルの入る 3.1 inch を割る。
    #   （240x180 の板を縦積みにして、幅 8 inch の枠に 1.4 inch の板を
    #     置いた結果、x 軸が ±800mm まで伸びて板が豆粒になった）
    if ar_pad >= 2.2:                     # 細長い板 → 地図を縦に積む
        rows, cols = 2, 1
    else:                                 # 正方形〜縦長 → 横に並べる
        rows, cols = 1, 2

    ph = (FIG_H_MAX - TOP_IN - INFO_IN - rows * (TITLE_IN + XLAB_IN)
          - (rows - 1) * GAP_IN) / rows
    # 幅の上限は「figure 幅 15 inch に収まること」から逆算する
    pw_max = (FIG_W_MAX - 0.30 - YLAB_IN - cols * CBAR_IN
              - (cols - 1) * COL_GAP_IN) / cols
    pw = float(np.clip(ph * ar_pad, PW_MIN, pw_max))
    if pw < ph * ar_pad - 1e-9:
        # 幅で頭打ちになった → 高さを合わせて詰める（空白を残さない）
        ph = pw / ar_pad

    fig_h = (TOP_IN + rows * (ph + TITLE_IN + XLAB_IN)
             + (rows - 1) * GAP_IN + INFO_IN)
    block_w = YLAB_IN + cols * (pw + CBAR_IN) + (cols - 1) * COL_GAP_IN
    fig_w = float(np.clip(block_w + 0.30, FIG_W_MIN, FIG_W_MAX))
    pad = max(0.0, (fig_w - block_w) / 2.0)
    dpi = float(np.clip(PX_LONG / max(fig_w, fig_h), 95.0, 150.0))

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi, facecolor="white")
    gs = fig.add_gridspec(
        rows, cols,
        left=(pad + YLAB_IN) / fig_w,
        right=(pad + YLAB_IN + cols * pw
               + (cols - 1) * (CBAR_IN + COL_GAP_IN)) / fig_w,
        top=1.0 - (TOP_IN + TITLE_IN) / fig_h,
        bottom=(INFO_IN + XLAB_IN) / fig_h,
        hspace=(GAP_IN + TITLE_IN + XLAB_IN) / ph,
        wspace=(COL_GAP_IN + CBAR_IN) / pw,
    )
    axes = [fig.add_subplot(gs[i // cols, i % cols]) for i in range(2)]
    return _Layout(fig, axes[0], axes[1], dpi, pw, ph)


def _setup_plate_ax(ax, reg: Region, *, fs: float = FS_TICK,
                    show_box: bool = False) -> None:
    """mm 目盛り・等倍・原点十字。⚠ 全ての板の図でこれを通すこと。"""
    mx, my = bc_margin(reg)
    # ⚠ adjustable="datalim"。既定の "box" だと軸の箱そのものが板の縦横比まで
    #   縮み、100x300 の板でタイトルの置ける幅が 2 inch を切って文字が
    #   両側にはみ出す。datalim なら箱は動かさず表示範囲だけ広げるので、
    #   等倍は保ったままタイトルの幅が確保できる。
    ax.set_xlim(-reg.w / 2 - mx, reg.w / 2 + mx)
    ax.set_ylim(-reg.h / 2 - my, reg.h / 2 + my)
    ax.set_aspect("equal", adjustable="datalim")
    # 原点（板中心）の十字。⚠ これが無いと Claude が座標を読み違える。
    ax.axhline(0, color="#555555", lw=1.0, ls=(0, (5, 4)), zorder=1.6)
    ax.axvline(0, color="#555555", lw=1.0, ls=(0, (5, 4)), zorder=1.6)
    ax.plot([0], [0], marker="+", color="#000000", ms=13, mew=2.0, zorder=1.7)
    ax.tick_params(labelsize=fs, length=4)
    ax.set_xlabel("x [mm]", fontsize=fs)
    ax.set_ylabel("y [mm]", fontsize=fs)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=6, steps=[1, 2, 2.5, 5, 10]))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5, steps=[1, 2, 2.5, 5, 10]))
    for s in ax.spines.values():
        s.set_color("#444444")
    if show_box:
        ax.add_patch(Rectangle((-reg.w / 2, -reg.h / 2), reg.w, reg.h,
                               fill=False, edgecolor=C_BOX, linewidth=1.8,
                               linestyle="--", zorder=1.8))


def _info_band(fig, cols: Sequence[Sequence[str]],
               colors: Sequence[str] | None = None,
               fs: float = FS_BASE) -> None:
    """図の下端に数値を並べる。⚠ 判断に要る数字は全部ここに文字で出す。

    位置は inch で決める。凡例（下端 LEGEND_IN）の上に載せるので、
    ⚠ 行数が N_INFO を超えると凡例に食い込む。増やすなら N_INFO も上げること。
    """
    fh = fig.get_figheight()
    y_top = (LEGEND_IN + 0.16 + N_INFO * LINE_IN) / fh
    # ⚠ 3 列目（判定・警告）は文が長いので幅を広めに取る。等分にすると
    #   「⚠ 変位／コンプライアンスが有限でない — 解が発散」が右端で切れる。
    xs = (0.022, 0.345, 0.665)
    for i, lines in enumerate(cols):
        col = (colors[i] if colors and i < len(colors) else "#111111")
        x = xs[i] if i < len(xs) else 0.022 + i * 0.32
        fig.text(x, y_top, "\n".join(lines[:N_INFO]), fontsize=fs,
                 va="top", ha="left", linespacing=1.5, color=col)


def _legend(fig, n_handles: int = 5, fs: float = 12.0) -> None:
    """記号の凡例を下端に。⚠ 3 つの図で同じ並び・同じ文言。"""
    hs, ls = bc_legend_handles()
    fig.legend(hs[:n_handles], ls[:n_handles], loc="lower center",
               ncol=n_handles, fontsize=fs, frameon=True, facecolor="white",
               edgecolor="#999999", bbox_to_anchor=(0.5, 0.012),
               handlelength=2.2, borderpad=0.45, columnspacing=1.6)


def _colorbar(fig, im, ax, label: str, pw: float, fs: float = FS_TICK,
              extend: str = "neither"):
    """軸の右に inset でカラーバーを付ける。

    ⚠ `fig.colorbar(ax=...)` は元の軸から場所を奪って軸を縮めるので、
      inch で組んだレイアウトが狂う。inset なら軸の大きさは変わらない。
    ⚠ ラベルは短い ASCII にする。縦書き（90°回転）の日本語はパネルが
      低いとき（横長の板）にカラーバーの高さを超えてはみ出す。
    """
    cw = float(np.clip(0.22 / max(pw, 0.5), 0.022, 0.085))
    cax = ax.inset_axes([1.0 + cw * 0.5, 0.0, cw, 1.0])
    cb = fig.colorbar(im, cax=cax, extend=extend)
    cb.set_label(label, fontsize=fs)
    cb.ax.tick_params(labelsize=fs - 1)
    return cb


def _panel_title(ax, short: str, long_: str, pw: float, color: str = "#111111",
                 fs: float = FS_BASE) -> None:
    """パネルのタイトル。⚠ 幅が足りないときは短い方に落とす。

    縦長の板ではパネル幅が 3 inch ほどしかなく、全角 17 文字で溢れる。
    溢れた文字は隣のパネルのタイトルと重なって**どちらも読めなくなる**ので、
    幅で分岐して短い文言を使う。数字は下の情報欄に必ず出してあるので
    落としても情報は失われない。
    """
    # 全角 1 文字 ≒ fs pt、半角 ≒ fs/2 pt として必要幅 [inch] を見積もる
    need = sum(1.0 if ord(c) > 0x2000 else 0.5 for c in long_) * fs / 72.0
    ax.set_title(long_ if need <= pw * 0.98 else short,
                 fontsize=fs, fontweight="bold", pad=7, color=color)


def _save(fig, path: str, dpi: float) -> None:
    _ensure_dir(path)
    with _quiet_mpl():
        fig.savefig(path, dpi=dpi, facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 1. 荷重ヒートマップ
# ---------------------------------------------------------------------------


def heatmap_png(res: TopoResult, path: str, title: str = "") -> None:
    """「どこに荷重が加わるか」の図。**最適化前（密度が全部 1）の解**を渡す前提。

    主図 : ひずみエネルギー密度（対数）。⚠ エネルギーは端部と中央で 4〜6 桁
           違うので線形だと 1 点だけ赤く残りあとは真っ黒になる。LogNorm 必須。
    副図 : von Mises 応力 [MPa]（線形）。SIGMA_ALLOW 超過をマゼンタで別扱いし、
           超過面積比を数字で書く。
    """
    reg = res.region
    lay = _two_panel_fig(reg)
    fig, ax_e, ax_v = lay.fig, lay.ax_a, lay.ax_b
    ext = _extent(reg)

    # --- 主図: ひずみエネルギー密度（対数）--------------------------------
    e = _clean(res.energy)
    pos = e[e > 0]
    if pos.size:
        emax = float(pos.max())
        emin = float(pos.min())
        # ⚠ 下限を上限の 1e-6 で切る。これをやらないと 1 要素の桁外れな
        #   極小値でカラーバーが 12 桁に伸びて、絵が全部同じ色になる。
        emin = max(emin, emax * 1e-6)
        if not (emin < emax):
            emin, emax = emax * 1e-3, emax
    else:
        # ⚠ energy が全部 0 / 負でも落とさない（LogNorm は vmin<=0 で例外）
        emin, emax = 1e-12, 1.0
    e_plot = np.clip(e, emin, emax)   # 0 と負は下限へクリップ
    im_e = ax_e.imshow(e_plot, origin="lower", extent=ext, cmap="YlOrRd",
                       norm=LogNorm(vmin=emin, vmax=emax),
                       interpolation="nearest", zorder=1)
    _setup_plate_ax(ax_e, reg)
    _colorbar(fig, im_e, ax_e, "log E", lay.pw)
    _panel_title(
        ax_e,
        _t("[主図] 荷重の通り道（対数）", "[main] load path (log)"),
        _t(f"[主図] 荷重の通り道 — ひずみエネルギー密度（対数 "
           f"{emin:.1e}〜{emax:.1e}）",
           f"[main] load path - strain energy density "
           f"(log {emin:.1e}..{emax:.1e})"),
        lay.pw)

    # --- 副図: von Mises -----------------------------------------------------
    vm = _clean(res.vm)
    vmax_real = float(vm.max()) if vm.size else 0.0
    over = vm > SIGMA_ALLOW
    over_ratio = float(over.mean()) * 100.0 if vm.size else 0.0
    cmap_v = plt.get_cmap("YlGnBu").copy()
    cmap_v.set_over(C_OVER)           # ⚠ 超過は別色。色相の続きにしない
    im_v = ax_v.imshow(vm, origin="lower", extent=ext, cmap=cmap_v,
                       norm=Normalize(vmin=0.0, vmax=SIGMA_ALLOW, clip=False),
                       interpolation="nearest", zorder=1)
    if over.any():
        # 色に加えて等高線でも境界を出す（色だけに頼らない）
        X, Y = reg.cell_centers()
        try:
            ax_v.contour(X, Y, vm, levels=[SIGMA_ALLOW], colors="black",
                         linewidths=2.0, zorder=2)
        except Exception:
            pass
    _setup_plate_ax(ax_v, reg)
    _colorbar(fig, im_v, ax_v, "MPa", lay.pw, extend="max")
    ov_txt = (_t(f"超過 {over_ratio:.1f}%", f"over {over_ratio:.1f}%")
              if over.any() else _t("超過なし", "no overstress"))
    _panel_title(
        ax_v,
        _t(f"[副図] von Mises [MPa] / {ov_txt}",
           f"[sub] von Mises [MPa] / {ov_txt}"),
        _t(f"[副図] von Mises 応力 — 許容 {SIGMA_ALLOW:.0f} MPa 超はマゼンタ"
           f"／{ov_txt}",
           f"[sub] von Mises - over {SIGMA_ALLOW:.0f} MPa in magenta / {ov_txt}"),
        lay.pw, color=(C_OVER if over.any() else "#111111"))

    # --- 境界条件（両方に同じ記号で）----------------------------------------
    for ax in (ax_e, ax_v):
        draw_bc(ax, reg, fs=FS_TICK - 1.0, lw=2.0)

    # --- 表題と数値 ---------------------------------------------------------
    ttl = title or _t(f"{reg.name} — 荷重ヒートマップ（最適化前・密度1）",
                      f"{reg.name} - load heatmap (pre-opt, dens=1)")
    fig.suptitle(ttl, fontsize=FS_TITLE, fontweight="bold",
                 y=1.0 - 0.16 / fig.get_figheight(), va="top")

    ftot = sum(float(np.hypot(fx, fy)) for _, fx, fy in reg.loads)
    left = [
        _t(f"板名 : {reg.name}", f"plate : {reg.name}"),
        _t(f"寸法 : {reg.w:.0f} x {reg.h:.0f} t{reg.t:.1f}  {reg.mat}",
           f"size  : {reg.w:.0f} x {reg.h:.0f} t{reg.t:.1f}  {reg.mat}"),
        _t(f"格子 : {reg.grid()[0]} x {reg.grid()[1]} (dx {reg.dx:.1f}mm)",
           f"grid  : {reg.grid()[0]} x {reg.grid()[1]} (dx {reg.dx:.1f}mm)"),
        _t(f"荷重 : {len(reg.loads)} 箇所 計 {ftot:,.0f} N",
           f"loads : {len(reg.loads)} pts, {ftot:,.0f} N"),
        _t(f"固定 : {len(reg.fixed)} 箇所 ／ 逃げ {len(reg.void)} 個",
           f"fixed : {len(reg.fixed)} / void {len(reg.void)}"),
    ]
    mid = [
        _t(f"最大変位 : {_num(res.disp_max, 3)} mm",
           f"max disp : {_num(res.disp_max, 3)} mm"),
        _t(f"最大 vM  : {_num(vmax_real, 1)} MPa（許容 {SIGMA_ALLOW:.0f}）",
           f"max vM   : {_num(vmax_real, 1)} MPa (allow {SIGMA_ALLOW:.0f})"),
        _t(f"超過面積 : {over_ratio:.2f} %", f"over area: {over_ratio:.2f} %"),
        _t(f"コンプラ : {_num(res.compliance, 1)} N·mm",
           f"compliance: {_num(res.compliance, 1)} N.mm"),
        _t(f"E={E_ALU / 1000:.0f} GPa ／ 目標体積率 {reg.frac * 100:.0f}%",
           f"E={E_ALU / 1000:.0f} GPa / target vf {reg.frac * 100:.0f}%"),
    ]
    # ⚠ 警告文は全角 18 文字以内に収めること。3 列目の幅を超えると
    #   右端で切れて、肝心の「何をすべきか」が読めなくなる。
    warn: list[str] = []
    # ⚠ 「固定が 2 か所あるか」で見てはいけない。1 か所でも**面で**留まって
    #   いれば節点が複数拘束されて回転は止まる。実際 yaw_arm は旋回リングに
    #   1 宣言で留まる 640mm のアームで、この判定だと毎回警告が出ていた
    #   （`scripts/topo_opt.FIX_SPAN_MIN` と同じ見方に揃える）。
    if reg.fixed:
        fx0 = min(r[0] for r in reg.fixed); fx1 = max(r[2] for r in reg.fixed)
        fy0 = min(r[1] for r in reg.fixed); fy1 = max(r[3] for r in reg.fixed)
        span = float(np.hypot(fx1 - fx0, fy1 - fy0))
    else:
        span = 0.0
    if span < 20.0:
        warn.append(_t(f"⚠ 固定の広がり {span:.0f}mm（剛体運動）",
                       f"! fixed span {span:.0f}mm (rigid body)"))
    if over_ratio > 0:
        warn.append(_t(f"⚠ 許容 {SIGMA_ALLOW:.0f} MPa 超過 {over_ratio:.2f}%",
                       f"! {over_ratio:.2f}% over {SIGMA_ALLOW:.0f} MPa"))
        warn.append(_t("　→ frac を +0.05", "  -> frac +0.05"))
    if not reg.loads:
        warn.append(_t("⚠ 荷重が 1 つも無い", "! no loads defined"))
    if not np.isfinite(res.disp_max) or not np.isfinite(res.compliance):
        warn.append(_t("⚠ 変位/コンプラが非有限（発散）",
                       "! non-finite disp/compliance"))
    right = [_t("【判定】", "[verdict]")] + (
        warn or [_t("不良なし（境界条件・応力とも問題なし）",
                    "no defects (BC and stress OK)")])

    _info_band(fig, [left, mid, right],
               colors=["#111111", "#111111", (C_OVER if warn else "#2e7d32")])
    _legend(fig, n_handles=4)
    _save(fig, path, lay.dpi)


# ---------------------------------------------------------------------------
# 2. 最適化後の平面図
# ---------------------------------------------------------------------------


def _draw_outline(ax, out: Outline, *, lw: float = 2.2, face: str = C_MAT,
                  edge: str = "black", z: float = 2.0) -> None:
    """外形と穴。⚠ 穴は「白で上塗り」で表現する。

    matplotlib の複合パスは巻き方向で塗りが変わって当てにならないので、
    外形を塗ってから穴を白で描き直す。切り抜き図面としてそのまま読める。
    """
    if len(out.outer) >= 3:
        ax.add_patch(Polygon(np.asarray(out.outer, dtype=float), closed=True,
                             facecolor=face, edgecolor=edge, linewidth=lw,
                             joinstyle="round", zorder=z))
    for h in out.holes:
        if len(h) >= 3:
            ax.add_patch(Polygon(np.asarray(h, dtype=float), closed=True,
                                 facecolor="white", edgecolor=edge,
                                 linewidth=max(1.2, lw - 0.4), zorder=z + 0.1))


def _outline_lines(ax, out: Outline, *, color: str, lw: float, z: float) -> None:
    """輪郭を線だけで重ねる（密度場の副図に赤で乗せる用）。"""
    rings = ([out.outer] if len(out.outer) >= 3 else []) + [
        h for h in out.holes if len(h) >= 3]
    for r in rings:
        a = np.asarray(r, dtype=float)
        a = np.vstack([a, a[:1]])
        ax.plot(a[:, 0], a[:, 1], color=color, lw=lw, zorder=z,
                solid_joinstyle="round")


def shape_png(res: TopoResult, out: Outline, path: str, title: str = "") -> None:
    """「最適化した板材の形」の平面図。

    主図 : 切り抜き図面そのもの（黒の実線＋薄いグレーの材料）。元の外接矩形を
           破線で重ね、どれだけ削れたかを一目で分かるようにする。
    副図 : しきい値をかける前の生の密度場。輪郭が**どこで切られたか**が分かる。
    """
    reg = res.region
    lay = _two_panel_fig(reg)
    fig, ax_s, ax_d = lay.fig, lay.ax_a, lay.ax_b

    # --- 主図: 切り抜き形状 --------------------------------------------------
    _setup_plate_ax(ax_s, reg, show_box=True)
    _draw_outline(ax_s, out)
    draw_bc(ax_s, reg, fs=FS_TICK - 1.0, lw=2.0)
    box_area = reg.w * reg.h
    vf_out = (out.area / box_area) if box_area > 0 else 0.0
    _panel_title(
        ax_s,
        _t(f"[主図] 切り抜き形状 — 体積率 {vf_out * 100:.1f}%",
           f"[main] cut outline - vf {vf_out * 100:.1f}%"),
        _t(f"[主図] 切り抜き形状（実線）と元の外接矩形（破線）— "
           f"体積率 {vf_out * 100:.1f}%",
           f"[main] cut outline (solid) vs original bbox (dashed) - "
           f"vf {vf_out * 100:.1f}%"),
        lay.pw)

    # --- 副図: 生の密度場 ----------------------------------------------------
    dens = np.clip(_clean(res.dens), 0.0, 1.0)
    # ⚠ vmax を 1 より少し上に取る。dens=1 を真っ黒にすると、上に重ねる
    #   赤い輪郭線が黒に埋もれて「どこで切ったか」が読めなくなる。
    im_d = ax_d.imshow(dens, origin="lower", extent=_extent(reg), cmap="Greys",
                       vmin=0.0, vmax=1.18, interpolation="nearest", zorder=1)
    _setup_plate_ax(ax_d, reg, show_box=True)
    _outline_lines(ax_d, out, color="#e00000", lw=2.2, z=4.0)
    draw_bc(ax_d, reg, fs=FS_TICK - 1.0, lw=1.8, label=False)
    _colorbar(fig, im_d, ax_d, "dens", lay.pw)
    _panel_title(
        ax_d,
        _t(f"[副図] 密度場＋輪郭（赤）平均 {res.vol_frac * 100:.1f}%",
           f"[sub] density + outline (red), mean {res.vol_frac * 100:.1f}%"),
        _t(f"[副図] しきい値前の密度場（濃い＝材料）＋抽出した輪郭（赤）— "
           f"平均密度 {res.vol_frac * 100:.1f}%",
           f"[sub] raw density field (dark=material) + extracted outline (red) - "
           f"mean {res.vol_frac * 100:.1f}%"),
        lay.pw)

    # --- 表題と数値 ---------------------------------------------------------
    ttl = title or _t(f"{reg.name} — 最適化後の平面図",
                      f"{reg.name} - optimized plan view")
    fig.suptitle(ttl, fontsize=FS_TITLE, fontweight="bold",
                 y=1.0 - 0.16 / fig.get_figheight(), va="top")

    mass = _outline_mass(res, out)
    dmass_g = (reg.mass0 - mass) * 1000.0
    dmass_pct = (dmass_g / (reg.mass0 * 1000.0) * 100.0) if reg.mass0 > 0 else 0.0
    thin = out.min_width < RIB_MIN

    left = [
        _t(f"板名 : {reg.name}", f"plate : {reg.name}"),
        _t(f"寸法 : {reg.w:.0f} x {reg.h:.0f} t{reg.t:.1f}  {reg.mat}",
           f"size  : {reg.w:.0f} x {reg.h:.0f} t{reg.t:.1f}  {reg.mat}"),
        _t(f"面積 : {out.area:,.0f} mm²（箱 {box_area:,.0f}）",
           f"area  : {out.area:,.0f} mm2 (box {box_area:,.0f})"),
        _t(f"体積率: {vf_out * 100:.1f} %（目標 {reg.frac * 100:.0f}%）",
           f"vol.fr: {vf_out * 100:.1f} % (target {reg.frac * 100:.0f}%)"),
        _t(f"連結 : {out.n_parts} 個 ／ 穴 {len(out.holes)} 個",
           f"parts : {out.n_parts} / holes {len(out.holes)}"),
    ]
    mid = [
        _t(f"最小幅: {_num(out.min_width, 2)} mm（下限 {RIB_MIN:.0f}）",
           f"min w : {_num(out.min_width, 2)} mm (limit {RIB_MIN:.0f})"),
        _t(f"質量 : {_num(mass, 4)} kg（元 {_num(reg.mass0, 4)}）",
           f"mass  : {_num(mass, 4)} kg (was {_num(reg.mass0, 4)})"),
        _t(f"削減 : {dmass_g:+,.1f} g ({dmass_pct:+.1f} %)",
           f"saved : {dmass_g:+,.1f} g ({dmass_pct:+.1f} %)"),
        _t("パラメータ（次の一手はここを動かす）", "params (move these)"),
        f"  frac {reg.frac:.2f} / rmin {reg.rmin:.1f} / dx {reg.dx:.1f}",
    ]
    # ⚠ 警告文は全角 18 文字以内。3 列目の幅を超えると右端で切れる。
    bad: list[str] = []
    if out.n_parts != 1:
        bad.append(_t(f"⚠ 連結成分 {out.n_parts} 個（1 でないと不可）",
                      f"! {out.n_parts} disconnected parts"))
    if thin:
        bad.append(_t(f"⚠ 最小幅 {out.min_width:.2f} < {RIB_MIN:.0f} mm",
                      f"! min width {out.min_width:.2f} < {RIB_MIN:.0f} mm"))
        bad.append(_t("　→ rmin を +0.5〜1.0 mm 上げる",
                      "  -> raise rmin by +0.5..1.0 mm"))
    if out.area <= 0:
        bad.append(_t("⚠ 面積 0 — 輪郭が取れていない",
                      "! zero area - no outline extracted"))
    right = [_t("【判定】", "[verdict]")] + (
        bad or [_t("製造制約 OK（連結 1・最小幅クリア）",
                   "manufacturing OK (1 part, width OK)")])

    _info_band(fig, [left, mid, right],
               colors=["#111111", "#111111", ("#c62828" if bad else "#2e7d32")])
    _legend(fig, n_handles=5)
    _save(fig, path, lay.dpi)


# ---------------------------------------------------------------------------
# 3. 全板の一覧
# ---------------------------------------------------------------------------

Item = tuple[TopoResult, Outline, list[str]]


# 輪郭を使えなくする不良。⚠ **これ以外は「見て直したいが組めなくはない」。**
# 座面の角が丸めで 1mm² 落ちるのと、板が 2 つに割れているのを同じ赤枠で
# 出すと、どちらも「不良 16 枚」になって次に直すべき板が選べない。
# `scripts/topo_opt.FATAL` と同じ語を見る（片方だけ直すとずれる）。
FATAL_WORDS = ("連結成分", "最小部材幅", "より重い")


def is_fatal(bad) -> bool:
    return any(any(k in m for k in FATAL_WORDS) for m in bad)


def _sheet_key(it: Item):
    """使えない板を先頭へ。同じなら削減量の少ない（＝伸びしろの無い）順。"""
    res, out, bad = it
    saved = (res.region.mass0 - _outline_mass(res, out)) * 1000.0
    return (0 if is_fatal(bad) else (1 if bad else 2), saved)


def sheet_png(items: list[Item], path: str) -> None:
    """全板を 1 枚に並べた一覧。**次にどの板を直すかをこれ 1 枚で決める**用。

    ⚠ 不良のある板を上に並べ替える。名前順に並べると、下の方の不良板を
      Claude が見落とす。
    """
    items = sorted(items, key=_sheet_key)
    n = max(1, len(items))
    rows = 1 if n <= 3 else (2 if n <= 8 else 3)
    cols = int(np.ceil(n / rows))

    # 1 コマの内訳 [inch]。⚠ 見出しと注記は figure 座標で置くので、
    #   ここで確保した高さがそのまま文字の入る場所になる。
    art_h = 2.45          # 形の図
    # ⚠ 諸元は 2 行に折る。1 行に詰めるとコマ幅（約 4 inch）を超えて
    #   隣のコマの枠に食い込む。不良欄は 3 行分（2 件 + 「他 N 件」）。
    ttl_h, sub_h, bad_h = 0.36, 0.56, 0.86
    cell_w = 3.55
    cell_h = ttl_h + art_h + sub_h + bad_h
    head_in, foot_in = 1.25, 0.90   # 上の見出し帯 / 下の凡例帯

    # ⚠ **上限で切ると、コマの中身だけ計算どおりに置かれて図がずれる。**
    #   15 枚（3 行 5 列）で必要な高さは 14.8 inch なのに 12.0 で切っていた
    #   ので、注記が次の行のタイトルに重なって**どちらも読めなかった**。
    #   コマの数だけ紙を大きくする。読み込みコストより読めることが先。
    fig_w = float(np.clip(cols * cell_w + 0.4, 8.5, 19.5))
    fig_h = float(max(5.0, rows * cell_h + head_in + foot_in))
    dpi = float(np.clip(1800.0 / max(fig_w, fig_h), 85.0, 150.0))

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi, facecolor="white")
    grid_top = 1.0 - head_in / fig_h
    grid_bot = foot_in / fig_h
    cw = (1.0 - 0.03) / cols                      # コマの幅（figure 比）
    chh = (grid_top - grid_bot) / rows            # コマの高さ（figure 比）

    tot_saved = 0.0
    tot_mass = 0.0
    n_bad = 0
    for i, (res, out, bad) in enumerate(items):
        reg = res.region
        r, c = i // cols, i % cols
        # コマの外枠（figure 座標）。⚠ 軸の spines を枠にすると、等倍表示で
        #   軸が形の縦横比まで縮むので枠の大きさがコマごとにばらばらになり、
        #   「赤枠＝不良」が目立たなくなる。枠はコマそのものに描く。
        x0 = 0.015 + c * cw
        y1 = grid_top - r * chh
        y0 = y1 - chh
        fig.add_artist(Rectangle(
            (x0 + 0.004, y0 + 0.004), cw - 0.012, chh - 0.010,
            transform=fig.transFigure, fill=False,
            # 赤＝そのままでは使えない / 橙＝使えるが直したい / 灰＝合格
            edgecolor=("#c62828" if is_fatal(bad)
                       else ("#ef8c00" if bad else "#cccccc")),
            linewidth=(3.0 if is_fatal(bad) else (2.0 if bad else 1.0)),
            zorder=0.5))

        ax = fig.add_axes([x0 + 0.02, y1 - (ttl_h + art_h) / fig_h,
                           cw - 0.04, art_h / fig_h])
        ax.set_aspect("equal", adjustable="datalim")
        m = 0.06 * max(reg.w, reg.h)
        ax.set_xlim(-reg.w / 2 - m, reg.w / 2 + m)
        ax.set_ylim(-reg.h / 2 - m, reg.h / 2 + m)
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)

        ax.add_patch(Rectangle((-reg.w / 2, -reg.h / 2), reg.w, reg.h,
                               fill=False, edgecolor=C_BOX, linewidth=1.4,
                               linestyle="--", zorder=1.5))
        _draw_outline(ax, out, lw=1.8)
        # ⚠ 小さい図なので文字は出さない。記号（色と形）だけ主図と揃える。
        draw_bc(ax, reg, label=False, lw=1.4)

        mass = _outline_mass(res, out)
        saved_g = (reg.mass0 - mass) * 1000.0
        tot_saved += saved_g
        tot_mass += mass
        pct = (saved_g / (reg.mass0 * 1000.0) * 100.0) if reg.mass0 > 0 else 0.0
        n_bad += 1 if is_fatal(bad) else 0
        xc = x0 + cw / 2

        fig.text(xc, y1 - 0.06 / fig_h,
                 f"{reg.name}   {saved_g:+,.0f} g ({pct:+.0f}%)",
                 ha="center", va="top", fontsize=14.0, fontweight="bold",
                 color=("#c62828" if bad else "#111111"))
        vf = (out.area / (reg.w * reg.h) * 100.0) if reg.w * reg.h else 0.0
        fig.text(xc, y1 - (ttl_h + art_h + 0.04) / fig_h,
                 _t(f"{reg.w:.0f}x{reg.h:.0f} t{reg.t:.1f} ／ 体積率 {vf:.0f}%\n"
                    f"最小幅 {_num(out.min_width, 1)}mm ／ frac {reg.frac:.2f}"
                    f" ／ rmin {reg.rmin:.1f}",
                    f"{reg.w:.0f}x{reg.h:.0f} t{reg.t:.1f} / vf {vf:.0f}%\n"
                    f"minw {_num(out.min_width, 1)}mm / frac {reg.frac:.2f}"
                    f" / rmin {reg.rmin:.1f}"),
                 ha="center", va="top", fontsize=12.0, color="#333333",
                 linespacing=1.35)
        if bad:
            # ⚠ 1 行 26 文字で切る。長い不良メッセージをそのまま出すと
            #   隣のコマに食い込んで両方読めなくなる。全文は review.md にある。
            msg = "\n".join(("・" if JP_OK else "- ") + b[:26] for b in bad[:2])
            if len(bad) > 2:
                msg += _t(f"\n… 他 {len(bad) - 2} 件", f"\n... +{len(bad) - 2} more")
            fig.text(xc, y1 - (ttl_h + art_h + sub_h + 0.04) / fig_h, msg,
                     ha="center", va="top", fontsize=12.5, color="#c62828",
                     fontweight="bold", linespacing=1.35)

    head = _t(f"板材トポロジー最適化 一覧 — {len(items)} 枚中 "
              f"{len(items) - n_bad} 枚が使える / 合計削減 {tot_saved:+,.0f} g",
              f"topology optimization sheet - {len(items) - n_bad}/{len(items)} "
              f"usable / total saved {tot_saved:+,.0f} g")
    fig.suptitle(head, fontsize=19.0, fontweight="bold",
                 y=1.0 - 0.13 / fig_h, va="top",
                 color=("#c62828" if n_bad else "#111111"))
    fig.text(0.5, 1.0 - 0.66 / fig_h,
             _t(f"合計質量 {tot_mass * 1000:,.0f} g ／ 赤枠＝使えない・"
                f"橙枠＝直したい・灰枠＝合格 ／ "
                f"不良のある板を左上から並べてある",
                f"total mass {tot_mass * 1000:,.0f} g / red frame = has defects "
                f"/ sorted defects-first from top-left"),
             ha="center", va="top", fontsize=14.0, color="#333333")
    # ⚠ 記号は主図と同じ。一覧だけ見て判断されても取り違えないよう凡例を出す
    hs, ls = bc_legend_handles()
    fig.legend(hs, ls, loc="lower center", ncol=5, fontsize=12.0, frameon=True,
               facecolor="white", edgecolor="#999999",
               bbox_to_anchor=(0.5, 0.12 / fig_h), handlelength=2.2,
               borderpad=0.45, columnspacing=1.6)

    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# 4. 図と対になる Markdown
# ---------------------------------------------------------------------------

# ⚠ この表は Claude が「次にどのパラメータをどちらへ動かすか」を決めるための
#   もの。症状 → 動かす向き の対応をここに固定しておく（毎回考えさせない）。
_HINTS_JA = [
    ("最小部材幅 < 6 mm", "min_width", "rmin を 0.5〜1 mm 上げる（部材が太る）"),
    ("連結成分 > 1", "n_parts", "frac を +0.05 する／rmin を上げる"),
    ("von Mises > 90 MPa", "vm", "frac を +0.05 する（材料を増やす）"),
    ("超過ゼロ・変位も小さい", "disp_max", "frac を -0.05 する（もっと削れる）"),
    ("形がギザギザ・格子が見える", "dx", "dx を半分にする（計算時間は 4 倍）"),
    ("穴が細かく散る", "rmin", "rmin を上げる（チェッカーボードを潰す）"),
]
_HINTS_EN = [
    ("min width < 6 mm", "min_width", "raise rmin by 0.5-1 mm"),
    ("parts > 1", "n_parts", "frac +0.05 or raise rmin"),
    ("von Mises > 90 MPa", "vm", "frac +0.05 (add material)"),
    ("no overstress, small disp", "disp_max", "frac -0.05 (can remove more)"),
    ("jagged / grid visible", "dx", "halve dx (4x slower)"),
    ("scattered small holes", "rmin", "raise rmin"),
]


def review_md(items: list[Item], path: str) -> None:
    """図と対になる Markdown。Claude が次の一手を決めるための数値表。

    画像は `topo/<板名>_shape.png` / `topo/<板名>_load.png` / `topo/sheet.png`
    として埋め込む（`scripts/topo_opt.py` が実際に書いている名前）。
    ⚠ PNG を書く側は `shape_name()` / `heat_name()` / `SHEET_NAME` で名前を
      作ること。名前がずれると Markdown から画像が見えなくなる。
    """
    items_sorted = sorted(items, key=_sheet_key)
    L: list[str] = []
    A = L.append

    tot_saved = sum((r.region.mass0 - _outline_mass(r, o)) * 1000.0
                    for r, o, _ in items)
    tot_mass = sum(_outline_mass(r, o) for r, o, _ in items)
    n_bad = sum(1 for _, _, b in items if b)

    A(_t("# 板材トポロジー最適化 レビュー", "# Plate topology optimization review"))
    A("")
    A(_t(f"- 板 **{len(items)} 枚** / 不良のある板 **{n_bad} 枚**",
         f"- **{len(items)} plates** / **{n_bad}** with defects"))
    A(_t(f"- 合計質量 **{tot_mass:.3f} kg** / 合計削減 **{tot_saved:+,.0f} g**",
         f"- total mass **{tot_mass:.3f} kg** / total saved **{tot_saved:+,.0f} g**"))
    A("")
    A(f"![]({MD_IMG_DIR}/{SHEET_NAME})")
    A("")

    # --- 判断の目安 ---------------------------------------------------------
    A(_t("## 次の一手の目安", "## What to change next"))
    A("")
    A(_t("| 症状 | 見る数字 | 動かす先 |", "| symptom | metric | action |"))
    A("|---|---|---|")
    for sym, met, act in (_HINTS_JA if JP_OK else _HINTS_EN):
        A(f"| {sym} | `{met}` | {act} |")
    A("")

    # --- 一覧表 -------------------------------------------------------------
    A(_t("## 一覧（不良のある板が上）", "## Table (defective plates first)"))
    A("")
    hdr = _t(
        "| 板 | 寸法 [mm] | t | 面積 [mm²] | 体積率 | 最小幅 [mm] | 連結 | "
        "質量 [kg] | 削減 [g] | 削減 % | 最大vM [MPa] | 変位 [mm] | "
        "コンプラ [N·mm] | frac | rmin | dx | 不良 |",
        "| plate | size [mm] | t | area [mm2] | vf | min w [mm] | parts | "
        "mass [kg] | saved [g] | saved % | max vM [MPa] | disp [mm] | "
        "compliance | frac | rmin | dx | defects |")
    A(hdr)
    A("|" + "---|" * 17)
    for res, out, bad in items_sorted:
        reg = res.region
        box = reg.w * reg.h
        mass = _outline_mass(res, out)
        saved_g = (reg.mass0 - mass) * 1000.0
        pct = (saved_g / (reg.mass0 * 1000.0) * 100.0) if reg.mass0 > 0 else 0.0
        vmax = float(_clean(res.vm).max()) if res.vm.size else 0.0
        flag = f"**{len(bad)}**" if bad else "-"
        mw = f"**{out.min_width:.2f}**" if out.min_width < RIB_MIN else \
            f"{out.min_width:.2f}"
        np_ = f"**{out.n_parts}**" if out.n_parts != 1 else "1"
        vm_s = f"**{vmax:.1f}**" if vmax > SIGMA_ALLOW else f"{vmax:.1f}"
        A(f"| {reg.name} | {reg.w:.0f}x{reg.h:.0f} | {reg.t:.1f} | "
          f"{out.area:,.0f} | {(out.area / box * 100 if box else 0):.1f}% | {mw} | "
          f"{np_} | {mass:.4f} | {saved_g:+,.1f} | {pct:+.1f}% | {vm_s} | "
          f"{res.disp_max:.3f} | {res.compliance:,.0f} | {reg.frac:.2f} | "
          f"{reg.rmin:.1f} | {reg.dx:.1f} | {flag} |")
    A("")

    # --- 板ごと -------------------------------------------------------------
    for res, out, bad in items_sorted:
        reg = res.region
        box = reg.w * reg.h
        mass = _outline_mass(res, out)
        saved_g = (reg.mass0 - mass) * 1000.0
        mark = " ⚠" if bad else ""
        A(f"## {reg.name}{mark}")
        A("")
        A(_t("最適化後の平面図（切り抜き形状＋密度場）",
             "optimized plan view (outline + density field)"))
        A("")
        A(f"![]({MD_IMG_DIR}/{shape_name(reg.name)})")
        A("")
        A(_t("荷重ヒートマップ（最適化前・密度1 で解いたもの）",
             "load heatmap (solved before optimization, dens=1)"))
        A("")
        A(f"![]({MD_IMG_DIR}/{heat_name(reg.name)})")
        A("")
        A(_t(f"- 寸法 {reg.w:.0f} x {reg.h:.0f} x t{reg.t:.1f} / {reg.mat} / "
             f"格子 {reg.grid()[0]}x{reg.grid()[1]}",
             f"- {reg.w:.0f} x {reg.h:.0f} x t{reg.t:.1f} / {reg.mat} / "
             f"grid {reg.grid()[0]}x{reg.grid()[1]}"))
        A(_t(f"- パラメータ **frac={reg.frac:.2f} / rmin={reg.rmin:.1f} mm / "
             f"dx={reg.dx:.1f} mm**（反復 {res.iters} 回）",
             f"- params **frac={reg.frac:.2f} / rmin={reg.rmin:.1f} mm / "
             f"dx={reg.dx:.1f} mm** ({res.iters} iters)"))
        A(_t(f"- 面積 {out.area:,.0f} mm²（体積率 "
             f"{(out.area / box * 100 if box else 0):.1f}%）/ 最小部材幅 "
             f"{out.min_width:.2f} mm / 連結成分 {out.n_parts}",
             f"- area {out.area:,.0f} mm2 (vf "
             f"{(out.area / box * 100 if box else 0):.1f}%) / min width "
             f"{out.min_width:.2f} mm / parts {out.n_parts}"))
        A(_t(f"- 質量 {mass:.4f} kg（元 {reg.mass0:.4f} kg、削減 {saved_g:+,.1f} g）",
             f"- mass {mass:.4f} kg (was {reg.mass0:.4f} kg, saved "
             f"{saved_g:+,.1f} g)"))
        if reg.note:
            A(f"- {reg.note}")
        A("")
        if bad:
            A(_t("**不良**", "**Defects**"))
            A("")
            for b in bad:
                A(f"- ⚠ {b}")
        else:
            A(_t("不良なし。", "No defects."))
        A("")

    _ensure_dir(path)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
