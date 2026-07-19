"""機構別(yaw/sling/trebuchet/roller)のパラメータ総当たり最適化 (torch エンジン)。

    .venv/bin/python scripts/optimize_mech.py <kind> <bucket|desk|flag> [trials]

kind: yaw|sling|trebuchet|roller。出力 out/opt-<target>-<kind>.json (belt と同形式)。
yaw は組合せごとに方位ズレをバッチ実測して機構マウントを補正(CPU版と同じ流儀)。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim.sim import BeltSim
from ragsim.sim_mech import SlingSim, TrebuchetSim, rotate_yaw_dir
from ragsim.targets import BucketTarget, FlagTarget, ShelfTarget

TARGETS = {
    "bucket": {"make": lambda: BucketTarget(cx=0.0, cz=2.4, rim_y=0.55, depth=0.255, radius=0.137), "max_time": 4.0, "D": 2.4},
    "desk": {"make": lambda: ShelfTarget(cx=0.0, cz=4.8, width=0.65, depth=0.45, top_y=0.5, desk_top_y=0.76), "max_time": 4.0, "D": 4.8},
    "flag": {"make": lambda: FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3), "max_time": 6.0, "D": 3.92},
}

# グリッド: CPU版最適値(bucket-2.4.json/desk-4.8.json)を種に、旗は高ロブ域(下降当て含む)
GRIDS = {
    ("yaw", "bucket"): (np.arange(5.0, 8.1, 0.75), np.arange(14.0, 26.1, 3.0)),
    ("yaw", "desk"): (np.arange(15.0, 21.1, 1.5), np.arange(8.0, 16.1, 2.0)),
    ("yaw", "flag"): (np.arange(9.0, 15.1, 1.5), np.arange(40.0, 60.1, 5.0)),
    ("sling", "bucket"): (np.arange(6.0, 10.1, 1.0), np.arange(25.0, 50.1, 6.0)),
    ("sling", "desk"): (np.arange(18.0, 30.1, 3.0), np.arange(25.0, 50.1, 6.0)),
    ("sling", "flag"): (np.arange(9.0, 12.01, 0.75), np.arange(30.0, 50.1, 5.0)),
    ("trebuchet", "bucket"): (np.arange(0.7, 1.31, 0.15), np.arange(25.0, 45.1, 5.0)),
    ("trebuchet", "desk"): (np.arange(1.4, 2.21, 0.2), np.arange(10.0, 26.1, 4.0)),
    ("trebuchet", "flag"): (np.arange(1.0, 2.21, 0.3), np.arange(45.0, 65.1, 5.0)),
    ("roller", "bucket"): (np.arange(3.5, 5.01, 0.25), np.arange(50.0, 70.1, 5.0)),
    ("roller", "desk"): (np.arange(7.0, 9.51, 0.5), np.arange(4.0, 12.1, 2.0)),
    ("roller", "flag"): (np.arange(6.0, 8.01, 0.5), np.arange(44.0, 60.1, 4.0)),
}

ROLLER_NIP, ROLLER_SPIN = 0.18, 0.6


def yaw_azimuths(combos: list[tuple[float, float]], target_make, max_time: float, n: int = 6, iters: int = 3) -> dict:
    """全組合せの方位ズレを反復実測(補正→残差→加算)。
    雑巾は腕先端(ピボットから横0.5m)から放たれ弾道がピボットを通らないため、
    自由落下点ではなく **実標的の判定面通過点の横ズレ** で合わせる(バケツ=リム面/机・旗=前面)。"""
    om = np.repeat([a for a, _ in combos], n)
    tl = np.repeat([b for _, b in combos], n)
    seeds = np.tile(np.arange(777, 777 + n), len(combos))
    az = np.zeros(len(combos))
    for _ in range(iters):
        dirs = np.stack([rotate_yaw_dir(np.array([0.0, 0.0, 1.0]), az[k // n]) for k in range(len(om))])
        sim = SlingSim(target_make(), omega_rel=om, angle_deg=tl, seeds=seeds, axis="yaw",
                       dir_vec=dirs, max_time=max_time)
        r = sim.run()
        arr = r["arrival"].reshape(len(combos), n, 3)
        lat = -np.nanmean(arr[:, :, 0], axis=1)
        fwd = np.nanmean(arr[:, :, 2], axis=1).clip(min=0.5)
        az = az + np.arctan2(lat, fwd)  # 判定面通過点の方位残差を積む
    return {combos[k]: float(az[k]) for k in range(len(combos))}


def build_sim(kind, target, A, Bv, seeds, max_time, az_map=None):
    if kind == "roller":
        return BeltSim(target, speed=A, elev_deg=Bv, seeds=seeds, nip_len=ROLLER_NIP, spin_frac=ROLLER_SPIN, max_time=max_time)
    if kind == "sling":
        return SlingSim(target, omega_rel=A, angle_deg=Bv, seeds=seeds, axis="pitch", max_time=max_time)
    if kind == "yaw":
        dirs = np.stack([rotate_yaw_dir(np.array([0.0, 0.0, 1.0]), az_map[(a, b)]) for a, b in zip(A, Bv)])
        return SlingSim(target, omega_rel=A, angle_deg=Bv, seeds=seeds, axis="yaw", dir_vec=dirs, max_time=max_time)
    if kind == "trebuchet":
        return TrebuchetSim(target, cw_kg=A, release_deg=Bv, seeds=seeds, max_time=max_time)
    raise ValueError(kind)


def main() -> None:
    kind = sys.argv[1]
    tkey = sys.argv[2]
    trials = int(sys.argv[3]) if len(sys.argv) > 3 else 16
    tg = TARGETS[tkey]
    As, Bs = GRIDS[(kind, tkey)]
    combos = [(round(float(a), 3), round(float(b), 3)) for a in As for b in Bs]

    t0 = time.time()
    az_map = None
    if kind == "yaw":
        az_map = yaw_azimuths(combos, tg["make"], tg["max_time"])
        print(f"yaw方位補正 {len(combos)}组 実測済 ({time.time()-t0:.0f}s)")

    A = np.repeat([a for a, _ in combos], trials)
    Bv = np.repeat([b for _, b in combos], trials)
    seeds = np.tile(np.arange(1000, 1000 + trials), len(combos))
    if kind == "yaw":
        az_full = {(a, b): az_map[(a, b)] for a, b in combos}
        sim = build_sim(kind, tg["make"](), A, Bv, seeds, tg["max_time"], az_full)
    else:
        sim = build_sim(kind, tg["make"](), A, Bv, seeds, tg["max_time"])
    r = sim.run()
    hit = r["hit"].reshape(len(combos), trials)
    rad = r["radial"].reshape(len(combos), trials)

    rows = []
    for k, (a, b) in enumerate(combos):
        h = hit[k]
        mm = float(rad[k][h].mean()) if h.any() else float(rad[k].mean())
        rows.append({"a": a, "b": b, "spin": 0.0, "p": float(h.mean() * 100), "meanMiss": mm})
    best_p = max(r_["p"] for r_ in rows)
    band = sorted((r_ for r_ in rows if r_["p"] >= best_p - 6), key=lambda r_: r_["meanMiss"])
    best = band[0]
    if kind == "yaw":
        best["azimuthRad"] = az_map[(best["a"], best["b"])]
    rows.sort(key=lambda r_: (-r_["p"], r_["meanMiss"]))
    print(f"=== {kind}/{tkey} ベスト: a={best['a']} b={best['b']} → {best['p']:.0f}% miss={best['meanMiss']:.3f} ({time.time()-t0:.0f}s)")
    for r_ in rows[:6]:
        print(f"  a={r_['a']:6.2f} b={r_['b']:5.1f} → {r_['p']:5.1f}%  miss={r_['meanMiss']:.3f}")

    out = Path(__file__).resolve().parent.parent / "out"
    (out / f"opt-{tkey}-{kind}.json").write_text(json.dumps({
        "cfg": {"kind": kind, "target": tkey, "trials": trials, "D": tg["D"], "engine": "torch"},
        "best": {"belt": best},
        "grid": rows,
    }, indent=1, ensure_ascii=False))
    print(f"→ out/opt-{tkey}-{kind}.json")


if __name__ == "__main__":
    main()
