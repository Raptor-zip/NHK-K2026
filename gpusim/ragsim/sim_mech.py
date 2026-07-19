"""遠心スリング(pitch/yaw)・投石機の CUDA バッチ移植 (rapier-rag.ts simulateSling/simulateTrebuchet)。

BeltSim を継承し、装填形状・巻き上げ(把持ばね+キネマティック腕)・離脱条件だけ差し替える。
自由飛行・空力・静定・判定・収録は共通。ローラーは BeltSim のパラメータ違い(nip 0.18 / spin 0.6)。
eager 実行(巻き上げ中の腕運動が時間依存のため compile 対象外)。
"""
from __future__ import annotations

import numpy as np
import torch

from . import mesh
from .sim import DT, DTH, SUB, BeltSim
from .targets import contact_ground

ATTACH_KS = 1400.0
ATTACH_KD = 1.5  # TS版は5だが陽的サブステップの安定限界(c·dt/m<2)を超えるため低減
ARM_LEN = 0.5
TETHER = 0.12
CW_ARM_SHORT = 0.18
TREB_ARM_LONG = 0.6
TREB_SLING = 0.35


def _edge_indices() -> np.ndarray:
    """把持する iy=0 の辺(上下層×全ix)のノード index [26]"""
    out = []
    for layer in range(2):
        for ix in range(mesh.NODES_X):
            out.append(mesh.idx(ix, 0, layer))
    return np.asarray(out, dtype=np.int64)


class _MechSim(BeltSim):
    """腕+把持ばね系の共通部。サブクラスが _tip_now(腕先端の運動) と離脱条件を定義する。"""

    engine = "mech"  # torch.compile 無効(腕が時間依存)

    def _init_mech(self, grip0: np.ndarray, hang_dir: np.ndarray, seeds: np.ndarray, dirs: np.ndarray) -> None:
        """装填: grip0[B,3] から hang_dir[B,3] 方向へ垂らした布を作り直し、把持ばねを張る"""
        B, N = self.B, mesh.N_NODES
        dev = self.device
        pos0 = np.zeros((B, N, 3), dtype=np.float32)
        for b in range(B):
            rng = np.random.default_rng(int(seeds[b]))
            e1 = dirs[b] / np.linalg.norm(dirs[b])
            lateral = np.cross(e1, [0.0, 1.0, 0.0])
            lateral /= np.linalg.norm(lateral)
            hang = hang_dir[b] / np.linalg.norm(hang_dir[b])
            normal = np.cross(hang, lateral)
            nl = np.linalg.norm(normal)
            normal = normal / nl if nl > 1e-9 else np.array([0.0, 1.0, 0.0])
            pos0[b] = mesh.place_hanging(grip0[b], hang, lateral, normal, rng)
        self.pos = torch.as_tensor(pos0, device=dev)
        self.vel = torch.zeros_like(self.pos)
        d0 = self.pos[:, self.sj] - self.pos[:, self.si]
        self.rest = d0.norm(dim=-1)

        self.edge = torch.as_tensor(_edge_indices(), device=dev)
        # 把持ばねの静止長 = アンカー(腕先端など)から辺ノードへの初期距離 (attachEdge 移植)
        anchor0 = torch.as_tensor(self._anchor0, dtype=torch.float32, device=dev)  # [B,3]
        self.attach_rest = (self.pos[:, self.edge] - anchor0.unsqueeze(1)).norm(dim=-1)  # [B,26]
        self._tip_hist: list[torch.Tensor] = []

    def _attach_force(self, F: torch.Tensor, pos: torch.Tensor, vel: torch.Tensor,
                      anchor: torch.Tensor, anchor_v: torch.Tensor, feeding: torch.Tensor) -> torch.Tensor:
        """辺ノード→アンカーの把持ばね。反力の合計 [B,3] を返す(投石機のグリップ動力学用)"""
        d = anchor.unsqueeze(1) - pos[:, self.edge]  # [B,26,3]
        L = d.norm(dim=-1) + 1e-9
        u = d / L.unsqueeze(-1)
        rv = ((anchor_v.unsqueeze(1) - vel[:, self.edge]) * u).sum(-1)
        s = (ATTACH_KS * (L - self.attach_rest) + ATTACH_KD * rv) * feeding.to(F.dtype).unsqueeze(-1)
        fe = u * s.unsqueeze(-1)
        F.index_add_(1, self.edge, fe)
        return -fe.sum(dim=1)

    def _windup_substeps(self, aeroF: torch.Tensor, feeding: torch.Tensor, active_f: torch.Tensor,
                         anchor: torch.Tensor, anchor_v: torch.Tensor) -> None:
        """巻き上げ中の 1 outer step(把持ばね込み, in-place)"""
        for _ in range(SUB):
            F = aeroF.clone()
            self._spring_force(F, self.pos, self.vel)
            self._attach_force(F, self.pos, self.vel, anchor, anchor_v, feeding)
            self.vel += DTH * (F / self.node_m + self.gvec)
            self.vel *= active_f
            self.pos += DTH * self.vel
            contact_ground(self.pos, self.vel)
            self.target.contact(self.pos, self.vel)

    def _extra_result(self) -> dict:
        tips = torch.stack(self._tip_hist).cpu().numpy() if self._tip_hist else None
        return {"arm_tips": tips}  # [n_outer_done, 3] (バッチ内で同一スケジュール前提: 代表=item0)


class SlingSim(_MechSim):
    """遠心式スリング。axis='pitch'(オーバースロー) / 'yaw'(ハンマー投げ, tiltDeg=回転面の傾き)"""

    def __init__(self, target, omega_rel, angle_deg, seeds, axis: str = "pitch",
                 dir_vec: np.ndarray | None = None, **kwargs) -> None:
        B = len(omega_rel)
        super().__init__(target, speed=np.asarray(omega_rel, dtype=np.float64),
                         elev_deg=np.zeros(B), seeds=seeds, **kwargs)
        dev = self.device
        self.axis = axis
        om = torch.as_tensor(np.asarray(omega_rel, dtype=np.float64), dtype=torch.float32, device=dev)
        ang = torch.as_tensor(np.asarray(angle_deg, dtype=np.float64) * np.pi / 180, dtype=torch.float32, device=dev)
        dirs = np.tile(np.array([0.0, 0.0, 1.0]), (B, 1)) if dir_vec is None else np.asarray(dir_vec, dtype=np.float64)
        e1 = torch.as_tensor(dirs, dtype=torch.float32, device=dev)  # [B,3] 水平射出方向
        up = torch.tensor([0.0, 1.0, 0.0], device=dev).expand(B, 3)
        lateral = torch.cross(e1, up, dim=-1)
        lateral = lateral / lateral.norm(dim=-1, keepdim=True)

        if axis == "yaw":
            tilt = ang
            self.pu = e1 * torch.cos(tilt).unsqueeze(-1) + up * torch.sin(tilt).unsqueeze(-1)
            self.pv = lateral
            wish = self.pu
        else:
            self.pu = e1
            self.pv = up.clone()
            wish = e1 * torch.cos(ang).unsqueeze(-1) + up * torch.sin(ang).unsqueeze(-1)
        self.phi_rel = torch.atan2((wish * self.pu).sum(-1), -(wish * self.pv).sum(-1))
        self.phi = self.phi_rel + np.pi
        self.omega = torch.zeros(B, device=dev)
        self.omega_rel = om
        self.alpha = om * om / (2 * np.pi)

        tip0 = self.pivot + (self.pu * torch.cos(self.phi).unsqueeze(-1) + self.pv * torch.sin(self.phi).unsqueeze(-1)) * ARM_LEN
        self._anchor0 = tip0.cpu().numpy()
        grip0 = self._anchor0 + np.array([0.0, -TETHER, 0.0])  # 布は先端から tether 下に垂らす
        self._init_mech(grip0, np.tile(np.array([0.0, -1.0, 0.0]), (B, 1)), seeds, dirs)
        self._tip = tip0
        self._tip_prev = tip0.clone()

    def _release_mask(self, fed: torch.Tensor, step: int) -> torch.Tensor:
        return (~fed) & (self.phi <= self.phi_rel + 1e-6)

    def _advance_eager(self, aeroF, feeding, active_f, all_fed):
        if not all_fed:
            # 腕を1 outer step ぶん回す(離脱済み item は phi_rel で停止)
            self.omega = torch.minimum(self.omega_rel, self.omega + self.alpha * DT)
            self.phi = torch.maximum(self.phi - self.omega * DT, self.phi_rel)
            self._tip_prev = self._tip
            self._tip = self.pivot + (self.pu * torch.cos(self.phi).unsqueeze(-1) + self.pv * torch.sin(self.phi).unsqueeze(-1)) * ARM_LEN
            tip_v = (self._tip - self._tip_prev) / DT
            self._windup_substeps(aeroF, feeding, active_f, self._tip, tip_v)
        else:
            self.pos, self.vel = self._substep_block(self.pos, self.vel, aeroF, feeding, active_f, False)
        self._tip_hist.append(self._tip[0].detach().clone())
        return None, None


class TrebuchetSim(_MechSim):
    """投石機: CW重力トルクの振り子ODEで腕をキネマティック駆動、スリング(ロープ)+グリップは動力学"""

    feed_max_t = 2.6  # 巻き上げ安全上限 (TS: maxWind 2.5s)

    def __init__(self, target, cw_kg, release_deg, seeds, **kwargs) -> None:
        B = len(cw_kg)
        super().__init__(target, speed=np.asarray(cw_kg, dtype=np.float64),
                         elev_deg=np.zeros(B), seeds=seeds, **kwargs)
        dev = self.device
        cw = torch.as_tensor(np.asarray(cw_kg, dtype=np.float64), dtype=torch.float32, device=dev)
        self.release_deg = torch.as_tensor(np.asarray(release_deg, dtype=np.float64), dtype=torch.float32, device=dev)
        m_arm, rag_m = 0.3, 0.048
        self.I = (cw * CW_ARM_SHORT ** 2 + (m_arm / 3) * (TREB_ARM_LONG ** 2 + CW_ARM_SHORT ** 2)
                  + rag_m * (TREB_ARM_LONG + TREB_SLING) ** 2)
        self.cw = cw
        self.phi = torch.full((B,), np.radians(-160.0), device=dev)
        self.omega = torch.zeros(B, device=dev)
        self.e1 = torch.tensor([0.0, 0.0, 1.0], device=dev).expand(B, 3)
        self.up = torch.tensor([0.0, 1.0, 0.0], device=dev).expand(B, 3)

        tip0 = self.pivot + (self.e1 * torch.cos(self.phi).unsqueeze(-1) + self.up * torch.sin(self.phi).unsqueeze(-1)) * TREB_ARM_LONG
        grip0 = tip0 + torch.tensor([0.0, -TREB_SLING, 0.0], device=dev)
        self._anchor0 = grip0.cpu().numpy()
        self._init_mech(self._anchor0, np.tile(np.array([0.0, -1.0, 0.0]), (len(cw_kg), 1)),
                        seeds, np.tile(np.array([0.0, 0.0, 1.0]), (len(cw_kg), 1)))
        self.grip = grip0.clone()  # 動的グリップ(質量0.02, ロープで先端に拘束)
        self.grip_v = torch.zeros_like(self.grip)
        self.grip_m = 0.02
        self._tip = tip0
        self._tip_prev = tip0.clone()

    def _release_mask(self, fed: torch.Tensor, step: int) -> torch.Tensor:
        V = self.vel.mean(dim=1)
        fwd = (V * self.e1).sum(-1)
        speed = V.norm(dim=-1)
        elev = torch.rad2deg(torch.atan2(V[:, 1], fwd.abs() + 1e-9))
        return (~fed) & (fwd > 0) & (speed > 4) & (elev <= self.release_deg)

    def _advance_eager(self, aeroF, feeding, active_f, all_fed):
        if not all_fed:
            # 振り子ODE(半陰的オイラー)
            alpha = self.cw * 9.81 * CW_ARM_SHORT * torch.cos(self.phi) / self.I
            self.omega = self.omega + alpha * DT
            self.phi = self.phi + self.omega * DT
            self._tip_prev = self._tip
            self._tip = self.pivot + (self.e1 * torch.cos(self.phi).unsqueeze(-1) + self.up * torch.sin(self.phi).unsqueeze(-1)) * TREB_ARM_LONG
            tip_v = (self._tip - self._tip_prev) / DT
            feed_f = feeding.to(self.pos.dtype)
            for _ in range(SUB):
                F = aeroF.clone()
                self._spring_force(F, self.pos, self.vel)
                f_grip = self._attach_force(F, self.pos, self.vel, self.grip, self.grip_v, feeding)
                # グリップ動力学 + ロープ拘束(先端から slingLen 以内)
                self.grip_v += DTH * (f_grip / self.grip_m + self.gvec) * feed_f.unsqueeze(-1)
                self.grip += DTH * self.grip_v * feed_f.unsqueeze(-1)
                d = self.grip - self._tip
                L = d.norm(dim=-1, keepdim=True) + 1e-9
                over = (L > TREB_SLING).to(L.dtype)
                u = d / L
                self.grip -= u * (L - TREB_SLING) * over
                vr = ((self.grip_v - tip_v) * u).sum(-1, keepdim=True)
                self.grip_v -= u * vr.clamp(min=0) * over  # 張り方向の離反速度を殺す
                self.vel += DTH * (F / self.node_m + self.gvec)
                self.vel *= active_f
                self.pos += DTH * self.vel
                contact_ground(self.pos, self.vel)
                self.target.contact(self.pos, self.vel)
        else:
            self.pos, self.vel = self._substep_block(self.pos, self.vel, aeroF, feeding, active_f, False)
        self._tip_hist.append(self._tip[0].detach().clone())
        return None, None


def rotate_yaw_dir(d: np.ndarray, off: float) -> np.ndarray:
    c, s = np.cos(off), np.sin(off)
    return np.array([d[0] * c + d[2] * s, d[1], -d[0] * s + d[2] * c])


def measure_yaw_azimuth(omega_rel: float, tilt_deg: float, n: int = 6) -> float:
    """yaw(ハンマー投げ)の系統的な方位ズレを実測(rotate_yaw_dir(dir, az) を機構 dir に使う)"""
    from .targets import BucketTarget

    far = BucketTarget(cx=0.0, cz=100.0, rim_y=0.55, depth=0.255, radius=0.137)  # 実質「床のみ」
    sim = SlingSim(far, omega_rel=np.full(n, omega_rel), angle_deg=np.full(n, tilt_deg),
                   seeds=np.arange(777, 777 + n), axis="yaw", max_time=3.0)
    sim.run()
    c = sim.pos.mean(dim=1).cpu().numpy()  # 静定後の重心
    piv = sim.pivot.cpu().numpy()
    fwd = (c[:, 2] - piv[2]).mean()
    lat = -(c[:, 0] - piv[0]).mean()  # lateral = (-1,0,0)
    return float(np.arctan2(lat, fwd))
