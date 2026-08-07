"""SIMP + OC 法による 2D トポロジー最適化のソルバ.

やっていること
--------------
1. 設計領域を Q4（4 節点四角形）で格子分割し、平面応力で解く
2. 要素の密度 x∈[0,1] に対して E(x) = Emin + x^penal (E0 - Emin)（SIMP）
3. コンプライアンス c = fᵀu を最小化する。感度 dc/dx は随伴不要
   （自己随伴問題なので dc/dx_e = -p x^(p-1)(E0-Emin)·u_eᵀk₁u_e）
4. 密度フィルタ（半径 rmin の円錐重み）で格子依存とチェッカーボードを切る
5. OC 法（optimality criteria）＋二分法で体積制約を満たしながら更新

⚠ **要素剛性行列を定数表で持たない。** 世に出回っている top88 / top99 の
  8×8 定数行列は「正方形要素・厚さ 1」専用で、要素の縦横比が 1 でない板
  （`w/nx ≠ h/ny`）に使うと剛性が間違う。しかも**壊れ方が静かで**、
  解は収束するが部材の向きが実際の荷重経路とずれる、という一番たちの悪い
  間違い方をする。ここでは 2×2 ガウス積分で要素の実寸から毎回組む。

⚠ **フィルタ半径 `reg.rmin` は mm であって格子数ではない。** ここを
  「格子 N 個ぶん」と読むと、dx を細かくしたときにフィルタも一緒に細くなり、
  部材が細く本数が増える（＝格子依存が残る）。最小部材幅という製造上の
  意味が消えるので、必ず mm → 要素数に換算してから重みを作ること。
  `scripts/topo_opt.py` の格子非依存の検査はこれを見ている。

座標系・単位は `core.py` に従う（板ローカル mm / N / MPa）。

節点と要素の番号
----------------
    節点 n = j*(nx+1) + i    (i: x 方向 0..nx, j: y 方向 0..ny)
    dof  = (2n, 2n+1)        (x, y)
    要素 e = j*nx + i        ← `dens.ravel()`（(ny, nx) の C 順）と一致する

⚠ 要素番号と密度配列の並びを一致させておくのは必須。ここがずれると
  「密度場は綺麗なのに応力だけ転置されている」ような症状になる。
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix, csr_matrix
from scipy.sparse.linalg import spsolve

from .core import (E_ALU, NU_ALU, Region, TopoResult, circle_mask,  # noqa: F401
                   dead_mask, rect_mask)

# ---------------------------------------------------------------------------
# 数値の根拠
# ---------------------------------------------------------------------------
# penal = 3.0（既定）
#   SIMP の罰則指数。E ∝ x^p にすると、中間密度は「使った質量のわりに
#   剛性が出ない」状態になるので、最適解が 0/1 に寄る。
#   * p = 1 だと中間密度がそのまま最適になり、灰色の霧が残って切り出せない
#   * p ≥ 3 にすると中間密度の剛性が Hashin–Shtrikman の上限を下回る、
#     つまり「そんな微視構造は作れない」ので、材料として選ばれなくなる。
#     3 という数字はこの下限から来ている（Bendsøe & Sigmund 1999）
#   * p ≥ 5 まで上げると初期の探索が効かなくなり、細い枝が生えないまま
#     局所最適に落ちる
PENAL_DEFAULT = 3.0

# Emin = E0 * 1e-9
#   密度 0 の要素にも極小の剛性を残す。
# ⚠ ここを厳密に 0 にすると、材料が全部消えた領域の節点が K の中で
#   孤立して**特異行列**になり、spsolve が警告も出さずに NaN や
#   1e+300 級のゴミを返す（そのまま感度に入るので密度場が一気に破綻する）。
#   1e-9 は倍精度の丸め（1e-16）から見て十分上、かつ全体剛性への寄与は
#   1e-9 なので結果には効かない、という妥協点。
EMIN_RATIO = 1e-9

# OC の設計変数下限
# ⚠ 0 にすると乗算更新 x ← x·√(...) が 0 に固着して、いちど消えた要素が
#   二度と復活しない。荷重経路が途中で変わる問題（本件は複数荷重）だと
#   これが致命的なので、1e-3 の床を残して「復活の芽」を確保する（top99 と同じ）。
X_MIN = 1e-3

# OC の二分法
# ⚠ ラグランジュ乗数 λ の大きさは荷重と E のスケールに比例して何桁も動く。
#   線形の二分法（l1=0, l2=1e9）だと桁が合わないときに収束せず、
#   体積制約が満たされないまま抜ける。ここでは**対数（幾何）二分**にして
#   λ の桁が何であっても 40 回で追い込む。
OC_BISECT_MAX = 60
OC_BISECT_TOL = 1e-8


# ---------------------------------------------------------------------------
# 要素剛性（2×2 ガウス積分）
# ---------------------------------------------------------------------------
# Q4 の節点は反時計回り: 左下(-1,-1) → 右下(+1,-1) → 右上(+1,+1) → 左上(-1,+1)
_XI_N = np.array([-1.0, 1.0, 1.0, -1.0])
_ETA_N = np.array([-1.0, -1.0, 1.0, 1.0])
_GP = 1.0 / np.sqrt(3.0)          # 2 点ガウス。重みは 1


def _plane_stress_D(E: float, nu: float) -> np.ndarray:
    """平面応力の弾性行列 (3×3)。せん断はエンジニアリングひずみ γ で受ける。"""
    return E / (1.0 - nu ** 2) * np.array([
        [1.0, nu, 0.0],
        [nu, 1.0, 0.0],
        [0.0, 0.0, (1.0 - nu) / 2.0],
    ])


def _B_matrix(xi: float, eta: float, a: float, b: float) -> np.ndarray:
    """自然座標 (ξ, η) でのひずみ–変位行列 B (3×8)。

    要素は長方形（a = 幅/2, b = 高さ/2）なのでヤコビアンは対角
    J = diag(a, b) になり、逆行列を明示的に組む必要がない。

    ⚠ ここを a = b と決め打ちすると、まさに「正方形専用の定数行列」に
      戻ってしまう。a と b は別々に割ること。
    """
    dN_dxi = 0.25 * _XI_N * (1.0 + eta * _ETA_N)
    dN_deta = 0.25 * _ETA_N * (1.0 + xi * _XI_N)
    dN_dx = dN_dxi / a
    dN_dy = dN_deta / b
    B = np.zeros((3, 8))
    B[0, 0::2] = dN_dx          # εxx = du/dx
    B[1, 1::2] = dN_dy          # εyy = dv/dy
    B[2, 0::2] = dN_dy          # γxy = du/dy + dv/dx
    B[2, 1::2] = dN_dx
    return B


def _ke_q4(dxe: float, dye: float, t: float, E: float, nu: float) -> np.ndarray:
    """Q4 要素の剛性行列 (8×8)。要素の**実寸から**組む。"""
    a, b = dxe / 2.0, dye / 2.0
    D = _plane_stress_D(E, nu)
    ke = np.zeros((8, 8))
    det_J = a * b               # 長方形なので一定
    for xi in (-_GP, _GP):
        for eta in (-_GP, _GP):
            B = _B_matrix(xi, eta, a, b)
            ke += t * (B.T @ D @ B) * det_J
    return ke


# ---------------------------------------------------------------------------
# メッシュ・境界条件
# ---------------------------------------------------------------------------
@dataclass
class _Mesh:
    """FEA に必要な、密度に依存しない量をまとめて持つ（毎反復で作り直さない）。"""

    nx: int
    ny: int
    nel: int
    ndof: int
    dxe: float                  # 要素の x 寸法 [mm]
    dye: float                  # 要素の y 寸法 [mm]
    vol_e: float                # 要素体積 [mm³]
    edof: np.ndarray            # (nel, 8) 要素の dof 表
    ke1: np.ndarray             # (8, 8) E=1 の要素剛性（SIMP で E をかける）
    B0: np.ndarray              # (3, 8) 要素中心 (ξ=η=0) の B
    D0: np.ndarray              # (3, 3) E=E0 の弾性行列（密度 1 換算の応力用）
    krow: np.ndarray            # 縮約後の行番号（自由 dof のみ）
    kcol: np.ndarray            # 縮約後の列番号
    kkeep: np.ndarray           # 三つ組のうち残すもののマスク
    nfree: int
    free: np.ndarray
    f: np.ndarray               # (ndof,) 荷重ベクトル
    node_xy: np.ndarray         # (nnode, 2)


def _node_xy(reg: Region, nx: int, ny: int) -> np.ndarray:
    """節点座標 (nnode, 2)。並びは n = j*(nx+1) + i。"""
    xs = -reg.w / 2.0 + np.arange(nx + 1) * (reg.w / nx)
    ys = -reg.h / 2.0 + np.arange(ny + 1) * (reg.h / ny)
    NX, NY = np.meshgrid(xs, ys)          # (ny+1, nx+1)
    return np.column_stack([NX.ravel(), NY.ravel()])


def _nodes_in_rect(node_xy: np.ndarray, rect, tol: float) -> np.ndarray:
    """矩形に入る節点の番号。

    ⚠ 矩形が格子ピッチより小さいと**節点が 1 つも入らない**。
      そのまま返すと「拘束したつもりの座が拘束されていない」「荷重を
      載せたつもりの座に力が載っていない」という、解けてしまうだけに
      気づきにくい事故になる。入らなかったときは矩形中心に最も近い
      節点 1 つを拾ってから、呼び側で警告を出す。
    """
    x0, y0, x1, y1 = rect
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    sel = np.flatnonzero(
        (node_xy[:, 0] >= x0 - tol) & (node_xy[:, 0] <= x1 + tol)
        & (node_xy[:, 1] >= y0 - tol) & (node_xy[:, 1] <= y1 + tol)
    )
    if sel.size:
        return sel
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    d2 = (node_xy[:, 0] - cx) ** 2 + (node_xy[:, 1] - cy) ** 2
    return np.array([int(np.argmin(d2))])


def _build_mesh(reg: Region) -> _Mesh:
    nx, ny = reg.grid()
    nel = nx * ny
    nnode = (nx + 1) * (ny + 1)
    ndof = 2 * nnode
    dxe = reg.w / nx
    dye = reg.h / ny

    # --- 要素の dof 表 -----------------------------------------------------
    I, J = np.meshgrid(np.arange(nx), np.arange(ny))     # (ny, nx)
    n1 = (J * (nx + 1) + I).ravel()      # 左下
    n2 = n1 + 1                          # 右下
    n4 = n1 + (nx + 1)                   # 左上
    n3 = n4 + 1                          # 右上
    edof = np.column_stack([
        2 * n1, 2 * n1 + 1,
        2 * n2, 2 * n2 + 1,
        2 * n3, 2 * n3 + 1,
        2 * n4, 2 * n4 + 1,
    ])

    ke1 = _ke_q4(dxe, dye, reg.t, 1.0, NU_ALU)
    B0 = _B_matrix(0.0, 0.0, dxe / 2.0, dye / 2.0)
    D0 = _plane_stress_D(E_ALU, NU_ALU)

    node_xy = _node_xy(reg, nx, ny)
    tol = 1e-6 * max(reg.w, reg.h)

    # --- 拘束 -------------------------------------------------------------
    if not reg.fixed:
        raise ValueError(
            f"[{reg.name}] 拘束 `fixed` が空。剛体運動が止まらないので "
            f"K は特異になる。留め座を 2 か所以上与えること。"
        )
    fixed_nodes: set[int] = set()
    for rect in reg.fixed:
        got = _nodes_in_rect(node_xy, rect, tol)
        if got.size == 1 and not _rect_contains_node(node_xy, rect, tol):
            warnings.warn(
                f"[{reg.name}] fixed 矩形 {rect} に節点が入らなかった "
                f"（格子ピッチ dx={reg.dx} より小さい）。最寄り節点 1 点で代用する。",
                stacklevel=2,
            )
        fixed_nodes.update(int(n) for n in got)
    if len(fixed_nodes) < 2:
        raise ValueError(
            f"[{reg.name}] 完全拘束された節点が {len(fixed_nodes)} 個しかない。"
            f"1 点留めでは面内回転が止まらず K が特異になる。"
            f"fixed を 2 か所以上（かつ格子ピッチ dx={reg.dx}mm より大きい矩形で）指定すること。"
        )
    fixed_nodes_arr = np.array(sorted(fixed_nodes))
    # 同一点に潰れていないか（矩形を 2 つ書いても同じ節点に落ちることがある）
    pts = node_xy[fixed_nodes_arr]
    if np.ptp(pts[:, 0]) < tol and np.ptp(pts[:, 1]) < tol:
        raise ValueError(
            f"[{reg.name}] 拘束節点が 1 点に潰れている。面内回転が止まらない。"
        )
    fixed_dof = np.concatenate([2 * fixed_nodes_arr, 2 * fixed_nodes_arr + 1])
    free = np.setdiff1d(np.arange(ndof), fixed_dof, assume_unique=False)
    nfree = free.size

    # --- 荷重 -------------------------------------------------------------
    # ⚠ 1 節点に集中させると、その節点まわりだけに極端なひずみエネルギーが
    #   立ち、SIMP が「そこに材料を集める」ことしかしなくなる（針状の
    #   人工的な部材が生える）。座の面積ぶんの節点に**等分**して載せる。
    f = np.zeros(ndof)
    total_f = 0.0
    for rect, fx, fy in reg.loads:
        got = _nodes_in_rect(node_xy, rect, tol)
        if got.size == 1 and not _rect_contains_node(node_xy, rect, tol):
            warnings.warn(
                f"[{reg.name}] load 矩形 {rect} に節点が入らなかった。"
                f"最寄り節点 1 点に全荷重を載せる（応力集中に注意）。",
                stacklevel=2,
            )
        f[2 * got] += fx / got.size
        f[2 * got + 1] += fy / got.size
        total_f += abs(fx) + abs(fy)
    if total_f <= 0.0:
        raise ValueError(
            f"[{reg.name}] 荷重が 0。コンプライアンスが 0 になり "
            f"OC の感度が全部 0 になるので最適化できない。"
        )
    if np.allclose(f[free], 0.0):
        raise ValueError(
            f"[{reg.name}] 荷重が全部 fixed 節点に載っている（自由 dof に力が無い）。"
            f"荷重座と拘束座が重なっていないか確認すること。"
        )

    # --- 疎行列の三つ組（自由 dof への縮約まで先に済ませておく）-----------
    # ⚠ 要素ループで dok/lil に足し込むと 100×100 格子でも 1 反復に数秒かかる。
    #   三つ組（行・列・値）を一括で作って coo → csc に落とすこと。
    #   行・列は密度に依存しないので、ここで 1 回だけ作る。
    iK = np.repeat(edof, 8, axis=1).ravel()      # 各要素 [d0×8, d1×8, ...]
    jK = np.tile(edof, (1, 8)).ravel()           # 各要素 [d0..d7, d0..d7, ...]
    remap = np.full(ndof, -1, dtype=np.int64)
    remap[free] = np.arange(nfree)
    ri, rj = remap[iK], remap[jK]
    kkeep = (ri >= 0) & (rj >= 0)

    return _Mesh(
        nx=nx, ny=ny, nel=nel, ndof=ndof, dxe=dxe, dye=dye,
        vol_e=dxe * dye * reg.t,
        edof=edof, ke1=ke1, B0=B0, D0=D0,
        krow=ri[kkeep], kcol=rj[kkeep], kkeep=kkeep,
        nfree=nfree, free=free, f=f, node_xy=node_xy,
    )


def _rect_contains_node(node_xy: np.ndarray, rect, tol: float) -> bool:
    x0, y0, x1, y1 = rect
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return bool(np.any(
        (node_xy[:, 0] >= x0 - tol) & (node_xy[:, 0] <= x1 + tol)
        & (node_xy[:, 1] >= y0 - tol) & (node_xy[:, 1] <= y1 + tol)
    ))


# ---------------------------------------------------------------------------
# 密度フィルタ（半径は mm）
# ---------------------------------------------------------------------------
def _build_filter(reg: Region, mesh: _Mesh) -> tuple[csr_matrix, np.ndarray]:
    """円錐重み H と行和 Hs を作る。

    重み w(e, f) = max(0, rmin - dist(e, f))、距離は**要素中心間の実距離
    [mm]**。要素が正方形でなくても効く。

    ⚠ ここで「半径 = 何要素ぶん」で書くと、dx を半分にしたときに実効半径も
      半分になり、部材が細く本数が倍になる。mm で書いてから
      rx = ceil(rmin/dxe), ry = ceil(rmin/dye) と要素数に**換算**する。

    ⚠ **rmin は「最小部材の半幅」ではない。実測では最小部材幅 ≒ rmin×1.1〜1.3。**
      `core.py` のコメントは「フィルタ半径＝最小部材の半幅」＝幅 2×rmin と
      読めるが、密度フィルタが実際に保証する長さスケールはフィルタ半径その
      ものに近い。片持ち梁 200×100・dx=2・frac=0.4 で 0.5 二値化したあと、
      幅 W の円板でモルフォロジー開をして測った実効最小幅:

          rmin= 3mm → 8mm   rmin= 6mm → 8mm   rmin=10mm → 11mm

      （rmin が小さいうちは格子 dx=2mm 由来の下限 8mm ≒ 4 要素で頭打ちになる）
      つまり「幅 RIB_MIN=6mm を保証したい」なら rmin は 6 前後を渡すのが安全で、
      3（＝RIB_MIN/2）だと格子が粗いときに 6mm を割りうる。実際 dx=3・rmin=3
      では 2.3% の画素が幅 6mm の円板に入らなかった。
      最終的な幅の判定は密度場ではなく `shape.check` の実測に委ねること。
    """
    nx, ny, nel = mesh.nx, mesh.ny, mesh.nel
    rmin = float(reg.rmin)
    if rmin <= 0.0:
        raise ValueError(f"[{reg.name}] rmin は正の mm でなければならない（{rmin}）。")
    if rmin < max(mesh.dxe, mesh.dye):
        warnings.warn(
            f"[{reg.name}] rmin={rmin}mm が要素寸法 "
            f"({mesh.dxe:.2f}×{mesh.dye:.2f}mm) より小さい。フィルタが効かず "
            f"チェッカーボードが出る。dx を細かくするか rmin を上げること。",
            stacklevel=2,
        )

    rx = int(np.ceil(rmin / mesh.dxe))
    ry = int(np.ceil(rmin / mesh.dye))
    eid = np.arange(nel).reshape(ny, nx)

    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    vals: list[np.ndarray] = []
    # オフセットごとに全要素をまとめて処理する（要素ループにしない）
    for dj in range(-ry, ry + 1):
        for di in range(-rx, rx + 1):
            wgt = rmin - float(np.hypot(di * mesh.dxe, dj * mesh.dye))
            if wgt <= 0.0:
                continue
            j0, j1 = max(0, -dj), min(ny, ny - dj)
            i0, i1 = max(0, -di), min(nx, nx - di)
            if j0 >= j1 or i0 >= i1:
                continue
            src = eid[j0:j1, i0:i1].ravel()
            dst = eid[j0 + dj:j1 + dj, i0 + di:i1 + di].ravel()
            rows.append(dst)
            cols.append(src)
            vals.append(np.full(src.size, wgt))

    H = coo_matrix(
        (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
        shape=(nel, nel),
    ).tocsr()
    Hs = np.asarray(H.sum(axis=1)).ravel()
    return H, Hs


# ---------------------------------------------------------------------------
# FEA
# ---------------------------------------------------------------------------
def _fea(mesh: _Mesh, xphys: np.ndarray, penal: float,
         E0: float, Emin: float) -> tuple[np.ndarray, float, np.ndarray]:
    """1 回解く。戻りは (U(ndof,), compliance, ce(nel,))。

    `ce` は **E=1 換算**のひずみエネルギー u_eᵀk₁u_e。SIMP の感度にも
    ヒートマップにも使い回す。
    """
    Ee = Emin + xphys.ravel() ** penal * (E0 - Emin)
    # (nel, 64) → ravel。要素 e の 64 個は ke1.ravel() と同じ (r, c) 順に並ぶ
    sK = (Ee[:, None] * mesh.ke1.ravel()[None, :]).ravel()[mesh.kkeep]
    K = coo_matrix((sK, (mesh.krow, mesh.kcol)),
                   shape=(mesh.nfree, mesh.nfree)).tocsc()

    u_free = spsolve(K, mesh.f[mesh.free])
    if not np.all(np.isfinite(u_free)):
        raise RuntimeError(
            "剛性行列が特異で変位が発散した（NaN/Inf）。拘束が足りないか、"
            "材料が分断されて浮き島ができている可能性がある。"
        )

    U = np.zeros(mesh.ndof)
    U[mesh.free] = u_free
    Ue = U[mesh.edof]                                  # (nel, 8)
    ce = np.einsum("ij,ij->i", Ue @ mesh.ke1, Ue)      # u_eᵀ k₁ u_e
    c = float(mesh.f @ U)
    return U, c, ce


def _post(mesh: _Mesh, U: np.ndarray, ce: np.ndarray,
          E0: float) -> tuple[np.ndarray, np.ndarray, float]:
    """(energy, vm, disp_max) を作る。どちらも **密度 1 換算**。

    ⚠ 密度で割った値（実効エネルギー）ではなく、その要素が中身の詰まった
      材料だったらどれだけエネルギーを持つか、を入れる。密度が 0 に落ちた
      要素も「本来そこに力が流れうるか」を見せたいから
      （これが `draw` のヒートマップの意味）。
    """
    ny, nx = mesh.ny, mesh.nx
    # ひずみエネルギー密度 [MPa] = u^T k0 u / 要素体積。k0 は E=E0 の剛性
    # なので u^T k0 u = E0 * ce（ke1 は E=1 で組んである）。
    # 契約どおり 1/2 は掛けない（相対値しか使わないヒートマップ用）。
    energy = (E0 * ce / mesh.vol_e).reshape(ny, nx)

    # 要素中心 (ξ=η=0) のひずみ → 応力（E=E0、つまり密度 1 換算）
    eps = U[mesh.edof] @ mesh.B0.T                     # (nel, 3)
    sig = eps @ mesh.D0.T                              # (nel, 3) [MPa]
    sx, sy, txy = sig[:, 0], sig[:, 1], sig[:, 2]
    vm = np.sqrt(np.maximum(sx ** 2 - sx * sy + sy ** 2 + 3.0 * txy ** 2, 0.0))

    disp = np.hypot(U[0::2], U[1::2])
    return energy, vm.reshape(ny, nx), float(disp.max())


# ---------------------------------------------------------------------------
# パッシブ領域
# ---------------------------------------------------------------------------
def _passive(reg: Region) -> tuple[np.ndarray, np.ndarray]:
    """(solid, void) の (ny, nx) ブールマスク。

    ⚠ 両方に入る要素は **void を優先**する。solid はねじ座面のような
      「消したくない」希望だが、void は軸の逃げや配線通しのような
      「埋まっていたら組めない」物理的必須条件だから。
    """
    # ⚠ void は「逃げ穴」だけではない。**設計領域の外**（`reg.domain` の
    #   外側）も同じく材料を置いてはいけない場所。ここを見落とすと、元が
    #   三角形の板で外接矩形いっぱいの形が出て、最適化なのに重くなる
    #   （ガセットが 62g → 105g になった）。`core.dead_mask` が両方を返す。
    solid = rect_mask(reg, reg.solid) if reg.solid else np.zeros(
        reg.cell_centers()[0].shape, dtype=bool)
    void = dead_mask(reg)
    solid = solid & ~void
    return solid, void


# ---------------------------------------------------------------------------
# 本体
# ---------------------------------------------------------------------------
def solve(reg: Region, iters: int = 80, penal: float = PENAL_DEFAULT,
          move: float = 0.2, tol: float = 0.01,
          verbose: bool = False) -> TopoResult:
    """SIMP + OC でコンプライアンス最小の密度場を求める。

    Parameters
    ----------
    iters   最大反復数。80 で大抵の板は落ち着く（history で確認できる）
    penal   SIMP の罰則指数（既定 3.0、根拠はモジュール冒頭）
    move    1 反復で密度が動ける幅。0.2 は OC の標準。大きくすると振動する
    tol     密度の最大変化がこれを下回ったら打ち切り
    """
    if iters < 1:
        raise ValueError("iters は 1 以上でなければならない。")
    E0 = E_ALU
    Emin = E0 * EMIN_RATIO

    mesh = _build_mesh(reg)
    H, Hs = _build_filter(reg, mesh)
    solid, void = _passive(reg)
    ny, nx, nel = mesh.ny, mesh.nx, mesh.nel

    solid_f = solid.ravel()
    void_f = void.ravel()
    active_f = ~(solid_f | void_f)
    n_solid, n_void = int(solid_f.sum()), int(void_f.sum())

    # --- 体積制約が実現可能か -------------------------------------------
    # frac は「外接矩形に対する」体積率（core.py の定義）。
    target = float(reg.frac) * nel
    if n_solid > target + 1e-9:
        raise ValueError(
            f"[{reg.name}] solid 領域だけで体積率 {n_solid / nel:.3f} あり、"
            f"目標 frac={reg.frac} を超えている。frac を上げるか solid を減らすこと。"
        )
    if target > nel - n_void + 1e-9:
        raise ValueError(
            f"[{reg.name}] void を除いた空きが体積率 {(nel - n_void) / nel:.3f} しかなく、"
            f"目標 frac={reg.frac} を置けない。"
        )

    # --- 初期密度：残りの体積を自由要素に均等配分 -------------------------
    x = np.zeros(nel)
    x[solid_f] = 1.0
    n_active = int(active_f.sum())
    if n_active:
        x[active_f] = np.clip((target - n_solid) / n_active, X_MIN, 1.0)

    def forward(xv: np.ndarray) -> np.ndarray:
        """設計変数 x → 物理密度 xPhys（フィルタ＋パッシブの上書き）。

        ⚠ パッシブはフィルタの**後**にも当てる。x だけ固定してフィルタを
          かけると、周りの材料が穴に滲み出して void が埋まる（軸が通らない板が
          できあがる）。
        """
        xp = (H @ xv) / Hs
        xp[solid_f] = 1.0
        xp[void_f] = 0.0
        return xp

    def backward(g: np.ndarray) -> np.ndarray:
        """dφ/dxPhys → dφ/dx（フィルタの連鎖律）。

        dxPhys_f/dx_e = H[f,e]/Hs[f]（f がアクティブなときだけ。パッシブな
        要素の xPhys は x に依らないので 0）。H は対称だが、Hs の割り算は
        行ごとなので H.T を明示的に使う。
        """
        t = (g / Hs) * active_f
        return H.T @ t

    xphys = forward(x)
    history: list[float] = []
    change = 1.0
    it = 0
    U = np.zeros(mesh.ndof)
    ce = np.zeros(nel)
    c = 0.0

    for it in range(1, iters + 1):
        U, c, ce = _fea(mesh, xphys, penal, E0, Emin)
        history.append(c)

        # --- 感度 ---------------------------------------------------------
        # dc/dxPhys = -p x^(p-1) (E0-Emin) · u_eᵀk₁u_e  ≤ 0
        dc_p = -penal * np.maximum(xphys, 1e-12) ** (penal - 1.0) * (E0 - Emin) * ce
        dv_p = np.ones(nel)                       # 体積は密度の総和（要素体積は一定）
        dc = backward(dc_p)
        dv = np.maximum(backward(dv_p), 1e-30)

        # --- OC 更新（λ を対数二分法で追い込む）--------------------------
        # 更新則 x ← x·√(-dc / (λ·dv))。λ が大きいほど体積は減る（単調）。
        ratio = np.maximum(-dc, 0.0) / dv
        scale = float(np.mean(ratio[ratio > 0])) if np.any(ratio > 0) else 1.0
        l1, l2 = scale * 1e-9, scale * 1e9
        xnew = x.copy()
        for _ in range(OC_BISECT_MAX):
            if (l2 - l1) <= OC_BISECT_TOL * (l1 + l2):
                break
            lmid = float(np.sqrt(l1 * l2))        # 幾何平均＝対数軸の中点
            cand = x * np.sqrt(ratio / lmid)
            xnew = np.clip(cand,
                           np.maximum(X_MIN, x - move),
                           np.minimum(1.0, x + move))
            xnew[solid_f] = 1.0
            xnew[void_f] = 0.0
            xp_try = forward(xnew)
            if xp_try.sum() > target:
                l1 = lmid                          # 材料が多い → λ を上げる
            else:
                l2 = lmid
        xphys = forward(xnew)

        change = float(np.max(np.abs(xnew - x)))
        x = xnew
        if verbose:
            print(f"  it {it:3d}  c={c:12.4f}  vol={xphys.mean():.4f}  "
                  f"ch={change:.5f}")
        if change < tol:
            break

    # 最後の密度場で解き直して、返す応力・エネルギーを密度場と辻褄を合わせる
    U, c, ce = _fea(mesh, xphys, penal, E0, Emin)
    history.append(c)
    energy, vm, dmax = _post(mesh, U, ce, E0)

    return TopoResult(
        region=reg,
        dens=xphys.reshape(ny, nx),
        energy=energy,
        vm=vm,
        disp_max=dmax,
        compliance=c,
        history=history,
        iters=it,
    )


def solve_initial(reg: Region) -> TopoResult:
    """最適化せずに「材料で埋めたまま」1 回だけ解く。

    ユーザーに見せる「今どこに力が加わっているか」のヒートマップはこちらを使う。

    ⚠ 最適化後の密度場で荷重ヒートマップを描いてはいけない。消えた要素は
      E→Emin なので**抵抗なく変形する**。密度 1 換算のエネルギーに直すと、
      力が流れていないのに大きな値が出る（片持ち梁 200×100 で実測: 消えた
      領域のエネルギー中央値が、材料で埋めた解の 58 倍）。つまり最適化後の
      図は「力が加わらない場所が暗い」のではなく「消えた場所が一番明るい」
      という、意味が反転した絵になる。
      荷重の説明図と最適化結果の図は**別の解**から作ること。
    """
    E0 = E_ALU
    Emin = E0 * EMIN_RATIO
    mesh = _build_mesh(reg)
    _solid, void = _passive(reg)

    dens = np.ones((mesh.ny, mesh.nx))
    dens[void] = 0.0                      # 逃げ穴だけは最初から空けておく

    U, c, ce = _fea(mesh, dens, 1.0, E0, Emin)   # 全部密度 1 なので penal は無関係
    energy, vm, dmax = _post(mesh, U, ce, E0)

    return TopoResult(
        region=reg,
        dens=dens,
        energy=energy,
        vm=vm,
        disp_max=dmax,
        compliance=c,
        history=[c],
        iters=0,
    )
