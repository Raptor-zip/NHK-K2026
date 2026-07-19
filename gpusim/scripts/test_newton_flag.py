"""旗横棒キャプセルへの接触が効くか、静置ドロップで軌跡を追う。"""
import sys
from pathlib import Path

import numpy as np
import torch
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim import mesh
from ragsim.sim_newton import NewtonBeltSim
from ragsim.targets import FlagTarget

f = FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3)
B = 2
sim = NewtonBeltSim(f, speed=np.zeros(B), elev_deg=np.zeros(B), seeds=np.arange(B), max_time=1.5)
print("shapes:", sim.model.shape_count, "particles:", sim.model.particle_count)

# 布を棒の 3cm 上に水平静置(棒中心に重心)
pos = np.zeros((B, mesh.N_NODES, 3), dtype=np.float32)
for b in range(B):
    rng = np.random.default_rng(b)
    pos[b] = mesh.place_hanging(
        np.array([0.0, f.bar_y + 0.03, f.bar_z - mesh.RAG_L / 2]),
        np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), rng)
pt = torch.as_tensor(pos, device=sim.pos.device)
sim.pos.copy_(pt)
wp.to_torch(sim.s1.particle_q).view(B, -1, 3).copy_(pt)
sim.rest.copy_((pt[:, sim.sj] - pt[:, sim.si]).norm(dim=-1))
wp.to_torch(sim.model.spring_rest_length).copy_(sim.rest.reshape(-1))

DT = 1 / 240
from ragsim.sim_newton import NEWTON_SUB
dth = DT / NEWTON_SUB
for step in range(int(0.8 / DT)):
    sim._f0.zero_()
    sim._f1.zero_()
    # 重力のみ(空力なし)で棒に当たるか
    sim.pos  # view
    for _ in range(NEWTON_SUB):
        sim.pipeline.collide(sim.s0, sim.contacts)
        sim.solver.step(sim.s0, sim.s1, sim.ctrl, sim.contacts, dth)
        sim.s0, sim.s1 = sim.s1, sim.s0
    if step % 40 == 0:
        q = sim.pos[0].detach().cpu().numpy()
        cy = q[:, 1].mean()
        # 棒軸(y=3, z=3.92, x自由)への最短距離
        d = np.sqrt((q[:, 1] - f.bar_y) ** 2 + (q[:, 2] - f.bar_z) ** 2)
        print(f"step {step:3d}: c.y={cy:.3f} min_dist_to_bar={d.min():.3f} nc={sim.contacts.soft_contact_count.numpy()[0]}")
