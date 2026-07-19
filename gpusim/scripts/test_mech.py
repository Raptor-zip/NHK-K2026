import sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim.sim_mech import SlingSim, TrebuchetSim, measure_yaw_azimuth, rotate_yaw_dir
from ragsim.sim import BeltSim
from ragsim.targets import BucketTarget

B = 12
t = BucketTarget(cx=0.0, cz=2.4, rim_y=0.55, depth=0.255, radius=0.137)

# pitch スリング (CPU最適 ω9/β45)
s = SlingSim(t, omega_rel=np.full(B,9.0), angle_deg=np.full(B,45.0), seeds=np.arange(1000,1000+B), max_time=4.0)
t0=time.time(); r = s.run()
print(f"sling(pitch) ω9/β45: {time.time()-t0:.0f}s hit {r['hit'].sum()}/{B} relV={r['release_speed'][:3].round(2)} ang={r['release_ang_deg'][:3].round(0)} radial={r['radial'].mean():.2f} broke={r['broke'].sum()}")

# 投石機 (CPU最適 CW0.9/離脱35)
s = TrebuchetSim(t, cw_kg=np.full(B,0.9), release_deg=np.full(B,35.0), seeds=np.arange(1000,1000+B), max_time=4.0)
t0=time.time(); r = s.run()
print(f"trebuchet CW0.9/35: {time.time()-t0:.0f}s hit {r['hit'].sum()}/{B} relV={r['release_speed'][:3].round(2)} ang={r['release_ang_deg'][:3].round(0)} radial={r['radial'].mean():.2f} broke={r['broke'].sum()}")

# yaw (CPU最適 ω6/tilt20 + 方位補正)
t0=time.time()
az = measure_yaw_azimuth(6.0, 20.0)
print(f"yaw方位ズレ実測: {np.degrees(az):.0f}° ({time.time()-t0:.0f}s)")
d = rotate_yaw_dir(np.array([0.0,0,1.0]), az)
s = SlingSim(t, omega_rel=np.full(B,6.0), angle_deg=np.full(B,20.0), seeds=np.arange(1000,1000+B),
             axis="yaw", dir_vec=np.tile(d,(B,1)), max_time=4.0)
r = s.run()
print(f"yaw ω6/tilt20(補正後): hit {r['hit'].sum()}/{B} relV={r['release_speed'][:3].round(2)} radial={r['radial'].mean():.2f} broke={r['broke'].sum()}")
