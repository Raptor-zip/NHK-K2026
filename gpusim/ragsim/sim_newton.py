"""NVIDIA Newton (旧 warp.sim 後継, Warp/CUDA) の XPBD ソルバを使うエンジン。

BeltSim と同一インターフェース。ばね拘束・粒子↔形状接触(位置ベース+クーロン摩擦)を
ライブラリの SolverXPBD に任せ、空力・ベルト送り・判定・収録は BeltSim の実装を流用する
(self.pos / self.vel を Newton の状態バッファへの torch ビューに差し替えるだけ)。

Newton 利用上のハマりどころ(実測):
- 粒子間接触はワールドをまたいでも効く+布内部でばねと干渉して爆発
  → model.particle_max_radius=0 & particle_grid=None で丸ごと無効化(自己衝突なし=旧実装同等)
- Rapier 向けのばね減衰 kd は XPBD のヤコビ反復と非互換で爆発 → kd≈0 に(空力が減衰を担う)
- 全インスタンスは同一シーンなので単一ワールドに重ねる(接触ペア数を O(B²) にしない)
"""
from __future__ import annotations

import numpy as np
import torch
import warp as wp

from . import mesh
from .sim import DT, BeltSim
from .targets import BucketTarget, FlagTarget, ShelfTarget

NEWTON_SUB = 8  # XPBD サブステップ(dt≈5.2e-4)。陽的 64 分割より粗くても無条件安定
NEWTON_ITERS = 10
KD_SCALE = 0.0  # ばね減衰は XPBD 反復で発散するため無効(空力+接触摩擦が減衰源)


def _add_target_shapes(builder, target, cfg_mu=0.7) -> None:
    import newton

    cfg = newton.ModelBuilder.ShapeConfig(mu=cfg_mu, restitution=0.0)
    if isinstance(target, BucketTarget):
        R, wall_t = target.radius, 0.02
        floor_y, depth = target.floor_y, target.depth
        cx, cz = target.cx, target.cz
        rmid = R + wall_t / 2
        seg = 32
        hw = rmid * np.tan(np.pi / seg) * 1.2
        for k in range(seg):
            a = 2 * np.pi * k / seg
            q = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(-a))
            builder.add_shape_box(
                body=-1,
                xform=wp.transform(wp.vec3(cx + rmid * np.cos(a), floor_y + depth / 2, cz + rmid * np.sin(a)), q),
                hx=wall_t / 2, hy=depth / 2, hz=float(hw), cfg=cfg,
            )
        # 底円盤(シリンダ軸は局所Z → X軸まわり90°回してY軸向きに)
        qx = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), float(np.pi / 2))
        builder.add_shape_cylinder(
            body=-1, xform=wp.transform(wp.vec3(cx, floor_y - 0.02, cz), qx),
            radius=R + wall_t, half_height=0.02, cfg=cfg,
        )
    elif isinstance(target, ShelfTarget):
        for (x0, x1, y0, y1, z0, z1) in target.boxes:
            builder.add_shape_box(
                body=-1,
                xform=wp.transform(wp.vec3((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), wp.quat_identity()),
                hx=(x1 - x0) / 2, hy=(y1 - y0) / 2, hz=(z1 - z0) / 2, cfg=cfg,
            )
    elif isinstance(target, FlagTarget):
        # 横棒: 局所Z軸 → Y軸まわり90°でX向き
        qy = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(np.pi / 2))
        builder.add_shape_capsule(
            body=-1,
            xform=wp.transform(wp.vec3((target.bar_x0 + target.bar_x1) / 2, target.bar_y, target.bar_z), qy),
            radius=target.bar_r, half_height=(target.bar_x1 - target.bar_x0) / 2, cfg=cfg,
        )
        # ポール: 局所Z軸 → X軸まわり-90°でY向き
        qx = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), float(-np.pi / 2))
        h = (target.bar_y + 0.08) / 2
        builder.add_shape_capsule(
            body=-1, xform=wp.transform(wp.vec3(target.pole_x, h, target.bar_z), qx),
            radius=target.pole_r, half_height=h, cfg=cfg,
        )
    else:
        raise ValueError(f"unknown target: {target}")
    builder.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=0.9, restitution=0.0))


class NewtonBeltSim(BeltSim):
    engine = "newton"

    def __init__(self, *args, mu: float = 0.7, soft_mu: float | None = None, newton_sub: int = NEWTON_SUB, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        import newton
        self.newton_sub = newton_sub

        soft_mu = mu if soft_mu is None else soft_mu

        B, N = self.B, mesh.N_NODES
        node_m = self.node_m

        # 布1枚ぶんのサブビルダー(位置は仮 — finalize 後に per-seed 初期状態で上書き)
        sub = newton.ModelBuilder(up_axis=newton.Axis.Y, gravity=-9.81)
        pos0 = self.pos[0].cpu().numpy()
        for p in pos0:
            sub.add_particle(pos=(float(p[0]), float(p[1]), float(p[2])), vel=(0.0, 0.0, 0.0),
                             mass=node_m, radius=mesh.NODE_R)
        ii, jj, ks, kd = mesh.build_springs()
        for a, b, e, d in zip(ii, jj, ks, kd):
            sub.add_spring(int(a), int(b), ke=float(e), kd=float(d) * KD_SCALE, control=0.0)

        main = newton.ModelBuilder(up_axis=newton.Axis.Y, gravity=-9.81)
        for _ in range(B):  # 全インスタンスを単一ワールドへ重ねる(標的形状は共有)
            main.add_builder(sub)
        _add_target_shapes(main, self.target, cfg_mu=mu)

        model = main.finalize(str(self.device))
        model.particle_max_radius = 0.0  # 粒子間接触を無効化
        model.particle_grid = None
        model.soft_contact_mu = soft_mu

        self.model = model
        self.solver = newton.solvers.SolverXPBD(model, iterations=NEWTON_ITERS)
        self.pipeline = newton.CollisionPipeline(model, broad_phase="nxn", soft_contact_margin=0.02)
        self.s0, self.s1 = model.state(), model.state()
        self.ctrl = model.control()
        self.contacts = self.pipeline.contacts()

        # per-seed の初期状態・静止長を上書き
        q0 = wp.to_torch(self.s0.particle_q).view(B, N, 3)
        q0.copy_(self.pos)
        wp.to_torch(self.s1.particle_q).view(B, N, 3).copy_(self.pos)
        wp.to_torch(model.spring_rest_length).copy_(self.rest.reshape(-1))

        # 以後 BeltSim のロジック(空力/ベルト/判定/収録)は Newton の状態ビューを直接読む
        self.pos = q0
        self.vel = wp.to_torch(self.s0.particle_qd).view(B, N, 3)
        self._f0 = wp.to_torch(self.s0.particle_f).view(B, N, 3)
        self._f1 = wp.to_torch(self.s1.particle_f).view(B, N, 3)

    def _advance_eager(self, aeroF, feeding, active_f, all_fed):
        """1 outer step: 外力(空力+ベルト)を state に書き、XPBD サブステップを回す"""
        if not all_fed:
            self._belt_force(aeroF, self.pos, self.vel, feeding)
        self._f0.copy_(aeroF)
        self._f1.copy_(aeroF)
        nsub = self.newton_sub
        dth = DT / nsub
        for _ in range(nsub):
            self.pipeline.collide(self.s0, self.contacts)  # contacts を in-place 更新
            self.solver.step(self.s0, self.s1, self.ctrl, self.contacts, dth)
            self.s0, self.s1 = self.s1, self.s0
        # newton_sub は偶数 → self.pos/self.vel のビューは常に s0 を指す
        return None, None
