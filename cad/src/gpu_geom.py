"""ソリッドの点内外判定を GPU（PyTorch / CUDA）で行う.

なぜ要るか
-----------
`scripts/screw_place.py` は OCC の `BRepClass3d_SolidClassifier` を
**数十万回**呼ぶ。1 点ごとの C++ 呼び出しなので、ねじの配置に 11 分かかる。
`assembly_check` / `sweep_fine` も同じ判定を姿勢ぶんだけ繰り返す。

形は判定の途中で変わらない。**一度だけ三角メッシュにしてしまえば**、
内外判定は「点から出したレイと三角形の交差回数の偶奇」で済み、
GPU で一括にできる。三角形 M 個 × 点 N 個の総当たりでも、
M=2 万・N=1 千で 2×10⁷ 交差判定＝ GPU なら数 ms。

⚠ **これは近似である。** メッシュ化の弦公差（`TOL`）ぶんだけ曲面が内側に
  へこむ。`screw_place` の探り（`PROBE = 0.4mm`）や接触公差（0.5mm）より
  1 桁細かく取ってあるが、**判定が CPU 版と一致することを確かめてから
  使うこと**（`scripts/gpu_bench.py`）。

⚠ **「面の上」は返さない。** OCC の分類は 外 / 面上 / 中 の 3 値だが、
  レイキャストで得られるのは 外 / 中 の 2 値。面上の点は、そのレイが
  三角形の縁を通るかどうかで揺れる。呼ぶ側が `state() == 1` に意味を
  持たせている箇所（`neighbors` の「触れる」判定）は、点を法線方向に
  ずらして 2 回引くこと（`ON_EPS` を足した位置での IN/OUT の食い違い）。

使い方
-------
    import gpu_geom as G
    ins = G.GpuInside(solids)        # 部品 1 個ぶんのソリッド列
    ins.many(pts)                    # (N,3) → bool の numpy 配列
    ins([x, y, z])                   # 1 点（互換。遅いので避ける）

torch は `gpusim/.venv` のものを借りる（`cad/.venv` には入れていない。
CUDA 込みで 5GB あり、2 つ持つ意味がない）。
"""

from __future__ import annotations

import os
import sys

import numpy as np

# --- torch の在り処 -------------------------------------------------------
# ⚠ **`sys.path` の後ろに足すこと。** 前に足すと numpy まで gpusim 側
#   （2.5.1）が使われ、cad の 2.2.6 と入れ替わる。torch は無くても
#   CPU 版で動くようにしてあるので、失敗しても致命ではない。
_GPUSIM = os.environ.get(
    "TR_TORCH_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "gpusim", ".venv", "lib", "python3.12",
                 "site-packages"))
if os.path.isdir(_GPUSIM) and _GPUSIM not in sys.path:
    sys.path.append(os.path.abspath(_GPUSIM))

try:
    import torch
    HAVE_TORCH = True
except Exception:                      # noqa: BLE001
    torch = None
    HAVE_TORCH = False

# 使う装置。TR_GPU=0 で CPU に落とせる（結果の突き合わせ用）。
DEVICE = ("cuda" if (HAVE_TORCH and torch.cuda.is_available()
                     and os.environ.get("TR_GPU", "1") != "0") else "cpu")

# メッシュ化の弦公差 [mm]。`screw_place.PROBE = 0.4` の 1/8。
TOL = 0.05
# レイの向き。軸に平行だと、軸に平行な面・辺を通ったときに交差が
# 二重に数えられる。**無理数寄りの向き**にして退化を避ける。
RAY = np.array([0.5773502691896258, 0.5773502691896258 * 1.2679491924311228,
                0.5773502691896258 * 0.7320508075688772], dtype=np.float64)
RAY /= np.linalg.norm(RAY)
# 面の上とみなす幅 [mm]（`assembly_check` の接触公差と同じ桁）
ON_EPS = 0.05
# 1 回に GPU へ載せる点数の上限（実際は三角形数から (n,M,3) が
# 2²⁴ 要素に収まるよう絞る）
CHUNK = 65536


def _tri_of(solid, tol: float = TOL):
    """ソリッド 1 個の三角形 (T,3,3) を返す。"""
    verts, faces = solid.tessellate(tolerance=tol)
    if not faces:
        return np.zeros((0, 3, 3), dtype=np.float32)
    v = np.array([(p.X, p.Y, p.Z) for p in verts], dtype=np.float32)
    f = np.array(faces, dtype=np.int64)
    return v[f]


class GpuInside:
    """部品 1 個（＝ソリッド何個か）の内外判定。

    `screw_place.Inside` と同じ呼び方ができるが、**まとめて聞くほど速い**。
    """

    __slots__ = ("tri", "lo", "hi", "n_tri", "strict", "_dev",
                 "v0", "e1", "e2", "h", "ok", "inv", "d")

    def __init__(self, solids, strict: bool = False, tol: float = TOL):
        if not isinstance(solids, (list, tuple)):
            solids = [solids]
        tris = [_tri_of(s, tol) for s in solids]
        tris = [t for t in tris if len(t)]
        tri = (np.concatenate(tris, axis=0) if tris
               else np.zeros((0, 3, 3), dtype=np.float32))
        self.n_tri = len(tri)
        self.strict = strict
        self._dev = DEVICE
        if self.n_tri:
            self.lo = tri.reshape(-1, 3).min(axis=0)
            self.hi = tri.reshape(-1, 3).max(axis=0)
        else:
            self.lo = np.zeros(3, dtype=np.float32)
            self.hi = np.zeros(3, dtype=np.float32)
        # ⚠ **三角形側の項は点に依らない。** 毎回の呼び出しで組み直すと、
        #   点が少ないときは三角形数（M）ぶんの計算が丸ごと支配的になる。
        #   ここで 1 度だけ作って GPU に置いておく。
        if HAVE_TORCH and self.n_tri:
            t = torch.as_tensor(tri, device=self._dev, dtype=torch.float32)
            self.tri = t
            self.v0 = t[:, 0]
            self.e1 = t[:, 1] - self.v0
            self.e2 = t[:, 2] - self.v0
            self.d = torch.as_tensor(RAY, device=self._dev,
                                     dtype=torch.float32)
            self.h = torch.cross(self.d.expand_as(self.e2), self.e2, dim=-1)
            a = (self.e1 * self.h).sum(-1)
            self.ok = a.abs() > 1e-9
            self.inv = torch.where(self.ok, 1.0 / torch.where(
                self.ok, a, torch.ones_like(a)), torch.zeros_like(a))
        else:
            self.tri = tri
            self.v0 = self.e1 = self.e2 = self.h = None
            self.ok = self.inv = self.d = None

    # -- 中核 --------------------------------------------------------------
    def many(self, pts) -> np.ndarray:
        """(N,3) の点が中にあるか。境界は「中」に寄せない（外）。"""
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        out = np.zeros(len(p), dtype=bool)
        if not self.n_tri or not len(p):
            return out
        # bbox の外は数えるまでもない（総当たりの N を減らす一番効く手）
        inbox = np.all((p >= self.lo - 1e-3) & (p <= self.hi + 1e-3), axis=1)
        idx = np.nonzero(inbox)[0]
        if not len(idx):
            return out
        if not HAVE_TORCH:
            out[idx] = _inside_numpy(self.tri, p[idx])
            return out
        d, v0, e1, h, ok, inv = (self.d, self.v0, self.e1, self.h,
                                 self.ok, self.inv)
        e2 = self.e2
        res = torch.zeros(len(idx), dtype=torch.bool, device=self._dev)
        pt_all = torch.as_tensor(p[idx], device=self._dev, dtype=torch.float32)
        # 中間テンソルは (n, M, 3)。三角形が多い部品ほど n を絞る。
        chunk = max(64, min(CHUNK, int((1 << 24) // max(self.n_tri, 1))))
        for s in range(0, len(idx), chunk):
            q = pt_all[s:s + chunk]                        # (n,3)
            sv = q[:, None, :] - v0[None, :, :]            # (n,M,3)
            u = (sv * h[None, :, :]).sum(-1) * inv[None, :]
            qv = torch.cross(sv, e1[None, :, :].expand_as(sv), dim=-1)
            v = (qv * d).sum(-1) * inv[None, :]
            t = (qv * e2[None, :, :]).sum(-1) * inv[None, :]
            hit = (ok[None, :] & (u >= 0) & (v >= 0) & (u + v <= 1.0)
                   & (t > 1e-5))
            # ⚠ **書き戻す幅は `chunk`（動的）であって `CHUNK`（上限）ではない。**
            #   ここを CHUNK にしていたら、残り全部を指すスライスへ chunk 個を
            #   代入することになり RuntimeError で落ちた。
            res[s:s + chunk] = (hit.sum(1) & 1).bool()
        out[idx] = res.cpu().numpy()
        return out

    def state_many(self, pts) -> np.ndarray:
        """0=外 / 1=面の上 / 2=中。面の上は **±ON_EPS で食い違う点**とする。"""
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        if not len(p):
            return np.zeros(0, dtype=np.int8)
        # 面の上かどうかは、レイ方向に ±ON_EPS ずらして結果が変わるかで見る。
        # ⚠ 3 回に分けて聞かない。**1 回にまとめる**ほど GPU は効く。
        off = (RAY * ON_EPS).astype(np.float32)
        n = len(p)
        r = self.many(np.concatenate([p, p + off, p - off], axis=0))
        st = np.where(r[:n], 2, 0).astype(np.int8)
        st[r[n:2 * n] != r[2 * n:]] = 1
        return st

    # -- 互換 API（1 点ずつ。遅いので新しいコードでは使わない）-------------
    def __call__(self, p) -> bool:
        r = self.state_many([p])[0] if self.strict else self.many([p])[0]
        return bool(r == 2) if self.strict else bool(r)

    def state(self, p) -> int:
        return int(self.state_many([p])[0])


def _inside_numpy(tri, p) -> np.ndarray:
    """torch が無いときの後退路（同じ式を numpy で）。"""
    v0, e1, e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    d = RAY.astype(np.float32)
    h = np.cross(np.broadcast_to(d, e2.shape), e2)
    a = (e1 * h).sum(-1)
    ok = np.abs(a) > 1e-9
    inv = np.zeros_like(a)
    inv[ok] = 1.0 / a[ok]
    out = np.zeros(len(p), dtype=bool)
    for s in range(0, len(p), 256):
        q = p[s:s + 256]
        sv = q[:, None, :] - v0[None, :, :]
        u = (sv * h[None, :, :]).sum(-1) * inv[None, :]
        qv = np.cross(sv, np.broadcast_to(e1[None, :, :], sv.shape))
        v = (qv * d).sum(-1) * inv[None, :]
        t = (qv * e2[None, :, :]).sum(-1) * inv[None, :]
        hit = ok[None, :] & (u >= 0) & (v >= 0) & (u + v <= 1.0) & (t > 1e-5)
        out[s:s + 256] = (hit.sum(1) % 2).astype(bool)
    return out


def info() -> str:
    if not HAVE_TORCH:
        return "torch なし（numpy の後退路で動く）"
    name = (torch.cuda.get_device_name(0) if DEVICE == "cuda" else "CPU")
    return f"torch {torch.__version__} / {DEVICE} / {name}"
