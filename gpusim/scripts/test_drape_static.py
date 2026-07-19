"""決定的実験: 完全均衡のU字ドレープ(初速0)が棒に留まるか。実物なら100%留まる。
両エンジンで同一初期条件。オフセット(不均衡)も振って「どこまで耐えるか」を測る。"""
import sys
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim import mesh
from ragsim.targets import FlagTarget

def drape_positions(f, offset, rng):
    """棒に長さ方向でU字に掛けた雑巾。offset=均衡からのズレ(m, +で奥側が長い)"""
    r_eff = f.bar_r + mesh.NODE_R + 0.002  # 棒表面+ノード半径+微小隙間
    pos = np.zeros((mesh.N_NODES, 3), dtype=np.float32)
    half = mesh.THICK / 2
    for layer in range(2):
        off_n = half if layer == 0 else -half
        for iy in range(mesh.NODES_Y):
            for ix in range(mesh.NODES_X):
                s = (iy / mesh.NY) * mesh.RAG_L  # 0..0.3 長さ方向
                x = (ix / mesh.NX - 0.5) * mesh.RAG_W
                s_top = mesh.RAG_L / 2 + offset  # 折り返し点
                d = s - s_top
                arc = r_eff * np.pi / 2  # 頂部の1/4円弧ぶん(近似)
                if abs(d) < arc:  # 頂部: 半円弧に沿わせる
                    th = d / r_eff  # -π/2..π/2
                    y = f.bar_y + (r_eff + off_n) * np.cos(th)
                    z = f.bar_z + (r_eff + off_n) * np.sin(th)
                else:  # 垂れ下がり(手前/奥)
                    sgn = np.sign(d)
                    hang = abs(d) - arc
                    y = f.bar_y - hang
                    z = f.bar_z + sgn * (r_eff + (off_n if sgn < 0 else -off_n))
                j = rng.standard_normal(3) * 0.001
                pos[mesh.idx(ix, iy, layer)] = (x + j[0], y + j[1], z + j[2])
    return pos

def run(engine, offsets):
    if engine == "newton":
        from ragsim.sim_newton import NewtonBeltSim as E
    else:
        from ragsim.sim import BeltSim as E
    f = FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3)
    B = len(offsets)
    sim = E(f, speed=np.zeros(B), elev_deg=np.zeros(B), seeds=np.arange(B), max_time=4.0)
    pos = np.stack([drape_positions(f, o, np.random.default_rng(k)) for k, o in enumerate(offsets)])
    pt = torch.as_tensor(pos, device=sim.pos.device)
    sim.pos.copy_(pt)
    if engine == "newton":
        import warp as wp
        wp.to_torch(sim.s1.particle_q).view(B, -1, 3).copy_(pt)
    sim.rest.copy_((pt[:, sim.sj] - pt[:, sim.si]).norm(dim=-1))
    if engine == "newton":
        import warp as wp
        wp.to_torch(sim.model.spring_rest_length).copy_(sim.rest.reshape(-1))
    sim.vel.zero_()
    sim.nip_len = -1.0  # 即fed扱い
    r = sim.run()
    c = sim.pos.mean(dim=1).cpu().numpy()
    print(f"[{engine}] offset(m):", [f"{o:+.3f}" for o in offsets])
    print(f"  掛かったまま: {list(r['hit'])}")
    print(f"  最終重心y:  ", [f"{y:.2f}" for y in c[:, 1]])

offsets = [0.0, 0.01, 0.02, 0.04, 0.06, 0.08]
run("newton", offsets)
run("torch", offsets)
