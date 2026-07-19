"""NewtonBeltSim の妥当性テスト: ベルト投擲→バケツ、旗ドロップ(掛かり)。"""
import time
import sys
from pathlib import Path

import numpy as np
import torch
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim import mesh
from ragsim.sim_newton import NewtonBeltSim
from ragsim.targets import BucketTarget, FlagTarget


def test_bucket_throw():
    B = 16
    t = BucketTarget(cx=0.0, cz=2.4, rim_y=0.55, depth=0.255, radius=0.137)
    sim = NewtonBeltSim(t, speed=np.full(B, 4.25), elev_deg=np.full(B, 45.0),
                        seeds=np.arange(1000, 1000 + B), max_time=4.0)
    t0 = time.time()
    r = sim.run()
    print(f"投擲→バケツ: {time.time()-t0:.0f}s hit {r['hit'].sum()}/{B} "
          f"radial={r['radial'].mean():.3f} broke={r['broke'].sum()} "
          f"relV={r['release_speed'][:3].round(2)} ang={r['release_ang_deg'][:3].round(1)}")


def test_flag_drop():
    f = FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3)
    B = 8
    dz = np.linspace(-0.05, 0.05, B)
    sim = NewtonBeltSim(f, speed=np.zeros(B), elev_deg=np.zeros(B), seeds=np.arange(B), max_time=3.0)
    pos = np.zeros((B, mesh.N_NODES, 3), dtype=np.float32)
    for b in range(B):
        rng = np.random.default_rng(100 + b)
        pos[b] = mesh.place_hanging(
            np.array([0.0, f.bar_y + 0.05, f.bar_z - mesh.RAG_L / 2 + dz[b]]),
            np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), rng)
    pt = torch.as_tensor(pos, device=sim.pos.device)
    sim.pos.copy_(pt)
    wp.to_torch(sim.s1.particle_q).view(B, -1, 3).copy_(pt)
    sim.rest.copy_((pt[:, sim.sj] - pt[:, sim.si]).norm(dim=-1))
    wp.to_torch(sim.model.spring_rest_length).copy_(sim.rest.reshape(-1))
    sim.nip_len = -1.0  # 即fed
    r = sim.run()
    c = sim.pos.mean(dim=1).cpu().numpy()
    print("旗ドロップ dz:", dz.round(3))
    print("  hung:", r["hit"], " broke:", r["broke"].sum())
    print("  final c.y:", c[:, 1].round(2))


if __name__ == "__main__":
    test_bucket_throw()
    test_flag_drop()
