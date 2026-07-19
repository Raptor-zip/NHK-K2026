import time, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim.sim_newton import NewtonBeltSim
from ragsim.sim import BeltSim
from ragsim.targets import FlagTarget, ShelfTarget

# 旗投擲 v8/62° を Newton と torch で比較
for Engine, name in ((NewtonBeltSim, "Newton"), (BeltSim, "torch")):
    B = 32
    f = FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3)
    sim = Engine(f, speed=np.full(B, 8.0), elev_deg=np.full(B, 62.0), seeds=np.arange(3000, 3000+B), max_time=5.0)
    t0=time.time(); r = sim.run()
    print(f"旗 {name}: {time.time()-t0:.0f}s 掛かり {r['hit'].sum()}/{B}={r['hit'].mean()*100:.0f}% radial={r['radial'].mean():.3f} broke={r['broke'].sum()}")

# 机も Newton で
for Engine, name in ((NewtonBeltSim, "Newton"),):
    B = 32
    t = ShelfTarget(cx=0.0, cz=4.8, width=0.65, depth=0.45, top_y=0.5, desk_top_y=0.76)
    sim = Engine(t, speed=np.full(B, 12.5), elev_deg=np.full(B, 4.0), seeds=np.arange(2000, 2000+B), max_time=4.0)
    t0=time.time(); r = sim.run()
    print(f"机 {name}: {time.time()-t0:.0f}s 命中 {r['hit'].sum()}/{B}={r['hit'].mean()*100:.0f}% radial={r['radial'].mean():.3f} broke={r['broke'].sum()}")
