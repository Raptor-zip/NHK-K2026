"""標的3種(バケツ/机下棚/旗)の物理コライダー(バッチSDF接触)と「入った/掛かった」判定。

TS版 rapier-rag.ts の addBucket/addShelf/judgeBucket/judgeShelf を GPU バッチに移植し、
旗(ポール+横棒キャプセル・掛かり判定)を新規追加。
接触は「押し出し(位置射影) + 法線速度殺し(反発0) + クーロン摩擦」を各サブステップで適用。
"""
from __future__ import annotations

from dataclasses import dataclass

import torch

from .mesh import NODE_R

WALL_T = 0.02
MU = 0.7
MU_GROUND = 0.8


def apply_contact(pos: torch.Tensor, vel: torch.Tensor, pen: torch.Tensor, n: torch.Tensor, mu: float) -> None:
    """pen>0 のノードを法線 n で押し出し、法線速度を殺し、摩擦を掛ける(in-place)。
    pos/vel: [B,N,3], pen: [B,N], n: [B,N,3] (単位ベクトル)"""
    m = pen > 0
    if pen.dtype != pos.dtype:
        pen = pen.to(pos.dtype)
    pen_c = pen.clamp(min=0)
    pos += n * pen_c.unsqueeze(-1)
    vn = (vel * n).sum(-1)
    hit = m & (vn < 0)
    jn = torch.where(hit, -vn, torch.zeros_like(vn))
    vel += n * jn.unsqueeze(-1)  # 法線速度を0へ(反発0)
    vnl = (vel * n).sum(-1, keepdim=True)
    vt = vel - n * vnl
    vt_len = vt.norm(dim=-1)
    scale = (1 - mu * jn / (vt_len + 1e-9)).clamp(min=0)
    scale = torch.where(hit, scale, torch.ones_like(scale))
    vel += vt * (scale - 1).unsqueeze(-1)


def contact_ground(pos: torch.Tensor, vel: torch.Tensor) -> None:
    """床(y=0 平面)"""
    pen = NODE_R - pos[..., 1]
    n = torch.zeros_like(pos)
    n[..., 1] = 1.0
    apply_contact(pos, vel, pen, n, MU_GROUND)


def _pick_min(escs: list[torch.Tensor], normals: list[torch.Tensor]) -> tuple[torch.Tensor, torch.Tensor]:
    """複数の脱出方向から最小距離のものを選ぶ。全て >0 のとき内部(接触)。"""
    E = torch.stack(escs, dim=-1)  # [B,N,K]
    Nv = torch.stack(normals, dim=-2)  # [B,N,K,3]
    inside = (E > 0).all(dim=-1)
    pen, k = E.min(dim=-1)
    n = torch.gather(Nv, -2, k.unsqueeze(-1).unsqueeze(-1).expand(*k.shape, 1, 3)).squeeze(-2)
    pen = torch.where(inside, pen, torch.zeros_like(pen) - 1)
    return pen, n


@dataclass
class BucketTarget:
    """上開き円筒バケツ。center=(x,z), rim_y=リム上端, depth, radius(内半径)"""
    cx: float
    cz: float
    rim_y: float
    depth: float
    radius: float

    kind = "bucket"

    @property
    def floor_y(self) -> float:
        return self.rim_y - self.depth

    def contact(self, pos: torch.Tensor, vel: torch.Tensor) -> None:
        r = NODE_R
        dx = pos[..., 0] - self.cx
        dz = pos[..., 2] - self.cz
        rho = torch.sqrt(dx * dx + dz * dz) + 1e-9
        rad = torch.stack([dx / rho, torch.zeros_like(rho), dz / rho], dim=-1)
        up = torch.zeros_like(pos)
        up[..., 1] = 1.0
        y = pos[..., 1]
        Ri, Ro = self.radius, self.radius + WALL_T
        # 側壁シェル (rho∈[Ri,Ro], y∈[floorY,rimY])
        pen, n = _pick_min(
            [rho - (Ri - r), (Ro + r) - rho, (self.rim_y + r) - y, y - (self.floor_y - r)],
            [-rad, rad, up, -up],
        )
        apply_contact(pos, vel, pen, n, MU)
        # 底(半径Ro の円盤, 上面=floorY, 厚み0.04)
        pen2, n2 = _pick_min(
            [(self.floor_y + r) - y, (Ro + r) - rho, y - (self.floor_y - 0.05 - r)],
            [up, rad, -up],
        )
        apply_contact(pos, vel, pen2, n2, MU)

    def judge(self, pos: torch.Tensor) -> torch.Tensor:
        """静定後、雑巾が実際にバケツ内へ収まったか [B]"""
        dx = pos[..., 0] - self.cx
        dz = pos[..., 2] - self.cz
        rho = torch.sqrt(dx * dx + dz * dz)
        y = pos[..., 1]
        inside = (rho < self.radius * 1.03) & (y > self.floor_y - 0.03) & (y < self.rim_y + 0.03)
        frac = inside.float().mean(dim=-1)
        c = pos.mean(dim=1)
        crho = torch.sqrt((c[:, 0] - self.cx) ** 2 + (c[:, 2] - self.cz) ** 2)
        return (frac >= 0.5) & (crho < self.radius)

    # 散布ドット: リム面(高さ rim_y)を降下通過した点
    def arrival_plane(self) -> tuple[str, float]:
        return ("y_down", self.rim_y)

    def radial_of(self, a: torch.Tensor) -> torch.Tensor:
        """狙い中心からの面内ズレ [B] (a: 到達点 [B,3])"""
        return torch.sqrt((a[:, 0] - self.cx) ** 2 + (a[:, 2] - self.cz) ** 2)


@dataclass
class ShelfTarget:
    """机の下棚(手前=-z に開口を持つ箱)。前方 +z へ射出する前提。"""
    cx: float
    cz: float
    width: float
    depth: float
    top_y: float
    desk_top_y: float

    kind = "shelf"

    def __post_init__(self) -> None:
        hw, hd, tw = self.width / 2, self.depth / 2, WALL_T
        cx, cz, ty = self.cx, self.cz, self.top_y
        # AABB [xmin,xmax,ymin,ymax,zmin,zmax] (addShelf 移植)
        self.boxes = [
            (cx - hw, cx + hw, ty, self.desk_top_y, cz - hd, cz + hd),  # 天板(塞がった塊)
            (cx - hw, cx + hw, 0.0, ty, cz + hd, cz + hd + tw),  # 背板
            (cx - hw - tw, cx - hw, 0.0, ty, cz - hd, cz + hd),  # 左側板
            (cx + hw, cx + hw + tw, 0.0, ty, cz - hd, cz + hd),  # 右側板
        ]

    def contact(self, pos: torch.Tensor, vel: torch.Tensor) -> None:
        r = NODE_R
        for (x0, x1, y0, y1, z0, z1) in self.boxes:
            x, y, z = pos[..., 0], pos[..., 1], pos[..., 2]
            ex = [x - (x0 - r), (x1 + r) - x, y - (y0 - r), (y1 + r) - y, z - (z0 - r), (z1 + r) - z]
            zeros = torch.zeros_like(pos)
            ns = []
            for axis, sign in ((0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)):
                n = zeros.clone()
                n[..., axis] = float(sign)
                ns.append(n)
            pen, n = _pick_min(ex, ns)
            apply_contact(pos, vel, pen, n, MU)

    def judge(self, pos: torch.Tensor) -> torch.Tensor:
        hw, hd = self.width / 2, self.depth / 2
        x, y, z = pos[..., 0], pos[..., 1], pos[..., 2]
        inside = (
            ((x - self.cx).abs() < hw + 0.03)
            & (z > self.cz - hd - 0.03)
            & (z < self.cz + hd + 0.03)
            & (y > -0.03)
            & (y < self.top_y + 0.03)
        )
        frac = inside.float().mean(dim=-1)
        c = pos.mean(dim=1)
        cin = (
            ((c[:, 0] - self.cx).abs() < hw)
            & (c[:, 2] > self.cz - hd)
            & (c[:, 2] < self.cz + hd)
            & (c[:, 1] < self.top_y)
        )
        return (frac >= 0.5) & cin

    def arrival_plane(self) -> tuple[str, float]:
        return ("z_fwd", self.cz - self.depth / 2)  # 開口(前面)通過

    def radial_of(self, a: torch.Tensor) -> torch.Tensor:
        return torch.sqrt((a[:, 0] - self.cx) ** 2 + (a[:, 1] - self.top_y / 2) ** 2)


@dataclass
class FlagTarget:
    """旗: 鉛直ポール + 水平横棒(x方向キャプセル)。雑巾が横棒に「掛かって残った」ら成功。
    field.ts: 横棒 y=3.0, 半幅0.3。横棒はポールから片持ち(狙点=布方向の中点)。"""
    bar_y: float  # 3.0
    bar_z: float  # 標的距離 D
    bar_x0: float  # 横棒の x 範囲(狙い線が x=0)
    bar_x1: float
    bar_r: float = 0.02
    pole_r: float = 0.03

    kind = "flag"

    @property
    def pole_x(self) -> float:
        return self.bar_x0 - 0.01

    def _segments(self) -> list[tuple[tuple, tuple, float]]:
        return [
            ((self.bar_x0, self.bar_y, self.bar_z), (self.bar_x1, self.bar_y, self.bar_z), self.bar_r),
            ((self.pole_x, 0.0, self.bar_z), (self.pole_x, self.bar_y + 0.08, self.bar_z), self.pole_r),
        ]

    def warmup(self, device, dtype=torch.float32) -> None:
        """定数テンソルを事前生成(CUDAグラフ記録中に作られてグラフ内部バッファ化するのを防ぐ)"""
        import math
        self._cap_cache = {}
        for p0, p1, _ in self._segments():
            L = math.dist(p0, p1)
            P0 = torch.tensor(p0, dtype=dtype, device=device)
            u = (torch.tensor(p1, dtype=dtype, device=device) - P0) / L
            self._cap_cache[(p0, p1)] = (P0, u, L)

    def _capsule(self, pos: torch.Tensor, vel: torch.Tensor, p0: tuple[float, float, float], p1: tuple[float, float, float], cap_r: float) -> None:
        if not hasattr(self, "_cap_cache"):
            self.warmup(pos.device, pos.dtype)
        P0, u, L = self._cap_cache[(p0, p1)]
        t = ((pos - P0) * u).sum(-1).clamp(0.0, L)
        q = P0 + u * t.unsqueeze(-1)
        d = pos - q
        dist = d.norm(dim=-1) + 1e-9
        pen = (NODE_R + cap_r) - dist
        n = d / dist.unsqueeze(-1)
        apply_contact(pos, vel, pen, n, MU)

    def contact(self, pos: torch.Tensor, vel: torch.Tensor) -> None:
        self._capsule(pos, vel, (self.bar_x0, self.bar_y, self.bar_z), (self.bar_x1, self.bar_y, self.bar_z), self.bar_r)
        self._capsule(pos, vel, (self.pole_x, 0.0, self.bar_z), (self.pole_x, self.bar_y + 0.08, self.bar_z), self.pole_r)

    def judge(self, pos: torch.Tensor) -> torch.Tensor:
        """横棒を跨いで掛かり、落ちずに残っているか"""
        z = pos[..., 2]
        front = (z < self.bar_z - 0.005).float().mean(dim=-1)
        back = (z > self.bar_z + 0.005).float().mean(dim=-1)
        c = pos.mean(dim=1)
        return (
            (front > 0.10)
            & (back > 0.10)
            & (c[:, 1] > self.bar_y - 0.45)
            & (c[:, 1] < self.bar_y + 0.15)
            & (c[:, 0] > self.bar_x0 - 0.06)
            & (c[:, 0] < self.bar_x1 + 0.06)
            & ((c[:, 2] - self.bar_z).abs() < 0.30)
        )

    def arrival_plane(self) -> tuple[str, float]:
        return ("z_fwd", self.bar_z)  # 横棒の鉛直面を前進通過した点(x,y)

    def radial_of(self, a: torch.Tensor) -> torch.Tensor:
        aim_x = (self.bar_x0 + self.bar_x1) / 2
        return torch.sqrt((a[:, 0] - aim_x) ** 2 + (a[:, 1] - self.bar_y) ** 2)
