"""トポロジー最適化の共通型 — ここだけが 4 つのモジュールの契約.

なぜ「長方形から丸穴を抜く」をやめるのか
----------------------------------------
これまでの `L.lighten` / `L.lighten_path` は、板の**外形を長方形のまま**にして
中に円を並べていた。そのため

  * 荷重が絶対に通らない**隅**が、外形の一部として必ず残る
    （yaw_side は使用率 24%。4 分の 3 は「そこに板がある」以外の理由が無い）
  * 円は等方なので、力の向きに沿った**方向性のある材料配置**ができない
  * 「どこを残すか」ではなく「どこに穴を開けるか」を決める問題なので、
    最適な形が探索空間に入っていない

トポロジー最適化は逆向きに解く。板の占めうる領域を全部材料で埋め、
**荷重を支えるのに寄与しない要素から順に消す**。残るのは荷重経路そのもの
の形（トラス・アーチ）で、隅は最初から残らない。

⚠ 以前このファイルの前身にあたるコメントで「SIMP にしない」と書いていたが、
  その理由（板厚は製造制約で決まる／格子依存）は**外形を決めない**前提の
  話だった。外形まで決めるなら SIMP でしか出せない形がある。格子依存は
  密度フィルタ（`rmin`）で切る。フィルタ半径は「最小部材の半幅」という
  製造上の意味を持つ数字なので、根拠をコメントに残せる。

座標系
------
板の**平面ローカル座標** [mm]。原点は設計領域の外接矩形の中心、
x が `w` の方向、y が `h` の方向。板厚方向は見ない（平面応力）。
組立の 3D 座標との対応は `scripts/topo_opt.py` が持つ。

単位
----
長さ mm / 力 N / 応力 MPa (= N/mm²) / 質量 kg / 密度 kg/mm³
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --- 材料（A5052-H32）------------------------------------------------------
# ⚠ ヤング率は最適化の**結果の形には効かない**（線形弾性なので密度分布は
#   荷重の比だけで決まる）。コンプライアンスの絶対値と、たわみの見積もりに
#   使う。ここを変えても形は変わらないことを承知の上で持つ。
E_ALU = 70_000.0          # MPa
NU_ALU = 0.33
SIGMA_ALLOW = 90.0        # MPa A5052-H32 耐力 195 に安全率 2.2

# --- 製造制約 --------------------------------------------------------------
# ⚠ **フィルタ半径は最小部材の半幅そのもの**。ここを格子ピッチの都合で
#   決めると、切れない形（幅 1mm のリブ）が平気で出てくる。
#   アルミ t3〜t6 をレーザー／ウォータージェットで切る前提で、
#   リブは 6mm 以上、内側の隅 R は工具径の半分以上。
RIB_MIN = 6.0             # mm 残す材料の最小幅
EDGE_MIN = 4.0            # mm ねじ穴の縁から外形までの最小残肉

Rect = tuple[float, float, float, float]      # (x0, y0, x1, y1) 板ローカル
Circle = tuple[float, float, float]           # (x, y, r)


@dataclass
class Region:
    """1 枚の板の設計領域と境界条件.

    `fixed` … 動かない相手に留まっている座。ここを完全拘束する。
    `loads` … 力を受ける座と、その力 [N]。板の面内成分だけ持つ。
    `solid` … 密度を 1 に固定する領域（ねじの座面・軸受座）。**消させない**
    `void`  … 密度を 0 に固定する領域（軸の逃げ穴・配線の通し穴）。**埋めさせない**

    ⚠ `fixed` が 1 か所しかない板は解が発散する（剛体運動が止まらない）。
      呼ぶ側で 2 か所以上あることを確かめること。実際 1 点留めの板は
      `assembly_check` の「1点留め」で別途落ちる。
    """

    name: str
    w: float
    h: float
    t: float
    fixed: list[Rect] = field(default_factory=list)
    loads: list[tuple[Rect, float, float]] = field(default_factory=list)
    solid: list[Rect] = field(default_factory=list)
    void: list[Circle] = field(default_factory=list)
    # 矩形の逃げ。⚠ **宣言に無い相手も抜く場所を作る。** 仰角モーターは
    #   pitch_side に留まっていない（ウォームブラケットに留まる）ので締結の
    #   宣言には出てこないが、板と同じ場所を占めるので**穴が要る**。
    #   宣言だけ見ていると、外形を切り直したときにここが埋まって
    #   「未宣言接触 pitch_side_R ↔ pitch_motor」になる（実際なった）。
    void_rect: list[Rect] = field(default_factory=list)
    # 材料を置いてよい範囲（板ローカルの多角形）。None なら外接矩形の全面。
    # ⚠ **元が長方形でない板は、外接矩形を設計領域にすると改悪になる。**
    #   ガセット（直角三角形）は矩形の半分しか肉が無いので、矩形全体を
    #   使ってよいことにすると 62g → 105g と**重くなった**。元の外形の
    #   内側でしか置けないようにすれば、少なくとも重くはならない。
    #   元の肉抜き穴は埋めて渡す（穴の位置ごと作り直すのが目的なので）。
    domain: list[tuple[float, float]] | None = None
    # 体積率の上限（設計領域の比で決まる）。自動リトライが太らせる際の頭打ち。
    frac_max: float = 0.90
    # 目標体積率。**元の板の実肉に対してではなく、外接矩形に対して**。
    frac: float = 0.35
    rmin: float = RIB_MIN / 2.0
    dx: float = 2.0                # 格子ピッチ [mm]
    mat: str = "A5052"
    rho_mat: float = 2.68e-6       # kg/mm³
    # 元の板の質量 [kg]。削減量を出すのに使う（最適化そのものには効かない）
    mass0: float = 0.0
    note: str = ""

    def grid(self) -> tuple[int, int]:
        """(nx, ny) 要素数。⚠ 必ず 1 以上。"""
        return max(1, round(self.w / self.dx)), max(1, round(self.h / self.dx))

    def cell_centers(self) -> tuple[np.ndarray, np.ndarray]:
        """要素中心の板ローカル座標 (X, Y)。どちらも (ny, nx)。"""
        nx, ny = self.grid()
        x = -self.w / 2 + (np.arange(nx) + 0.5) * self.w / nx
        y = -self.h / 2 + (np.arange(ny) + 0.5) * self.h / ny
        return np.meshgrid(x, y)


@dataclass
class TopoResult:
    """最適化の結果。すべて (ny, nx) の格子上。"""

    region: Region
    dens: np.ndarray               # 密度 0..1
    energy: np.ndarray             # ひずみエネルギー密度（ヒートマップの元）
    vm: np.ndarray                 # von Mises 応力 [MPa]（密度 1 換算）
    disp_max: float                # 最大変位 [mm]
    compliance: float              # コンプライアンス [N·mm]
    history: list[float] = field(default_factory=list)
    iters: int = 0

    @property
    def vol_frac(self) -> float:
        return float(self.dens.mean())

    def mass(self) -> float:
        """密度場から出した質量 [kg]。"""
        r = self.region
        return float(self.dens.sum()) * (r.w / r.grid()[0]) * (
            r.h / r.grid()[1]) * r.t * r.rho_mat


@dataclass
class Outline:
    """密度場から取り出した切り抜き形状（板ローカル mm）。

    `outer` … 外形。反時計回りの (x, y) 列。
    `holes` … 内側の穴。時計回りの (x, y) 列。
    """

    outer: list[tuple[float, float]]
    holes: list[list[tuple[float, float]]] = field(default_factory=list)
    area: float = 0.0
    min_width: float = 0.0         # 実測した最小部材幅 [mm]
    n_parts: int = 1               # 連結成分の数。⚠ 1 でなければ不良


def rect_mask(reg: Region, rects, grow: float = 0.0) -> np.ndarray:
    """矩形の並びを格子のブールマスクにする。"""
    X, Y = reg.cell_centers()
    m = np.zeros(X.shape, dtype=bool)
    for x0, y0, x1, y1 in rects:
        m |= ((X >= x0 - grow) & (X <= x1 + grow)
              & (Y >= y0 - grow) & (Y <= y1 + grow))
    return m


def circle_mask(reg: Region, circles, grow: float = 0.0) -> np.ndarray:
    X, Y = reg.cell_centers()
    m = np.zeros(X.shape, dtype=bool)
    for cx, cy, r in circles:
        m |= ((X - cx) ** 2 + (Y - cy) ** 2) <= (r + grow) ** 2
    return m


def domain_mask(reg: Region) -> np.ndarray:
    """材料を置いてよい要素（True）。`reg.domain` が無ければ全面 True。"""
    if not reg.domain:
        nx, ny = reg.grid()
        return np.ones((ny, nx), dtype=bool)
    from matplotlib.path import Path            # 依存を増やさない（作図で既に使う）
    X, Y = reg.cell_centers()
    pts = np.column_stack([X.ravel(), Y.ravel()])
    return Path(reg.domain).contains_points(pts).reshape(X.shape)


def dead_mask(reg: Region) -> np.ndarray:
    """**材料を置いてはいけない**要素（True）= 逃げ穴 ∪ 設計領域の外.

    ⚠ ソルバは `void` だけでなくこれを見ること。設計領域の外に材料を
      置けてしまうと、元が三角形の板で外接矩形いっぱいの形が出る。
    """
    dead = circle_mask(reg, reg.void) | ~domain_mask(reg)
    # ⚠ **`void_rect` は座面に譲る。** これは「宣言に無いのに板の場所を
    #   占める相手」を相手の**外接箱**で表したもので、実体はもっと小さい。
    #   箱がねじの座面まで伸びていることがあり、そのまま抜くと座面が
    #   432mm² 欠けて「ねじが留められない」になった（仰角モーター ↔
    #   pitch_side_R）。座面は実体があると分かっている場所なので優先する。
    #   ⚠ 軸の逃げ（`void` の円）は譲らない。軸は本当に板を貫くので、
    #     そこに座面を置くほうが間違い。
    return dead | (rect_mask(reg, reg.void_rect) & ~rect_mask(reg, reg.solid))
