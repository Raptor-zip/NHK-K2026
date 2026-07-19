"""ベルト射出パラメータ(速度×仰角[×スピン])の GPU バッチ総当たり最適化。

    .venv/bin/python scripts/optimize.py <bucket|desk|flag> [trials] [spin] [engine=torch|newton]

engine=newton は Warp/Newton の XPBD 接触ソルバを使う(接触がライブラリ品質・高速)。
出力は out/opt-<target>[-newton].json。

全組合せ×試投を1バッチで同時にシミュレートする(CPU版 rag-optimize-bucket.ts の代替)。
ランキング: 命中率ベスト帯(6%以内)の中で狙い中心ズレ最小(CPU版と同じ流儀)。
出力: out/opt-<target>.json
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ragsim.targets import BucketTarget, FlagTarget, ShelfTarget


def get_engine(name: str):
    if name == "newton":
        from ragsim.sim_newton import NewtonBeltSim
        return NewtonBeltSim
    from ragsim.sim import BeltSim
    return BeltSim

BELT_NIP = 0.35

SCENES = {
    "bucket": {
        "target": lambda: BucketTarget(cx=0.0, cz=2.4, rim_y=0.55, depth=0.255, radius=0.137),
        "speeds": np.arange(3.75, 5.51, 0.25),
        "elevs": np.arange(42.0, 60.1, 3.0),
        "spins": [0.0],
        "max_time": 4.0,
        "D": 2.4,
    },
    "desk": {
        "target": lambda: ShelfTarget(cx=0.0, cz=4.8, width=0.65, depth=0.45, top_y=0.5, desk_top_y=0.76),
        "speeds": np.arange(10.0, 14.01, 0.5),
        "elevs": np.arange(4.0, 16.1, 2.0),
        "spins": [0.0],
        "max_time": 4.0,
        "D": 4.8,
    },
    "flag": {
        "target": lambda: FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3),
        "speeds": np.arange(7.0, 11.01, 0.5),
        "elevs": np.arange(44.0, 72.1, 3.0),
        "spins": [0.0, 0.4],
        "max_time": 5.0,
        "D": 3.92,
    },
}

# 旗の粗掃引ベスト(v8/60°/spin0 → 17%)周辺の細分化
SCENES["flag-fine"] = {
    **SCENES["flag"],
    "speeds": np.arange(7.5, 8.76, 0.25),
    "elevs": np.arange(56.0, 66.1, 2.0),
    "spins": [0.0, 0.25],
}


def main() -> None:
    kind = sys.argv[1] if len(sys.argv) > 1 else "flag"
    trials = int(sys.argv[2]) if len(sys.argv) > 2 else 24
    sc = SCENES[kind]
    engine = "torch"
    for a in sys.argv[3:]:  # 追加引数: 数値=スピン水準の絞り込み / torch|newton=エンジン
        if a in ("torch", "newton"):
            engine = a
        else:
            sc = {**sc, "spins": [float(a)]}
            kind = f"{kind}-s{a}"
    Engine = get_engine(engine)
    suffix = "-newton" if engine == "newton" else ""

    combos = [(float(s), float(e), float(sp)) for sp in sc["spins"] for s in sc["speeds"] for e in sc["elevs"]]
    rows = []
    t0 = time.time()
    for spin in sc["spins"]:
        sub = [(s, e) for s in sc["speeds"] for e in sc["elevs"]]
        B = len(sub) * trials
        speed = np.repeat([s for s, _ in sub], trials)
        elev = np.repeat([e for _, e in sub], trials)
        seeds = np.tile(np.arange(1000, 1000 + trials), len(sub))
        sim = Engine(
            sc["target"](), speed=speed, elev_deg=elev, seeds=seeds,
            nip_len=BELT_NIP, spin_frac=spin, max_time=sc["max_time"],
        )
        r = sim.run()
        hit = r["hit"].reshape(len(sub), trials)
        rad = r["radial"].reshape(len(sub), trials)
        for k, (s, e) in enumerate(sub):
            h = hit[k]
            # ズレは命中回の平均(全外しなら全回平均)
            mm = float(rad[k][h].mean()) if h.any() else float(rad[k].mean())
            rows.append({"a": float(s), "b": float(e), "spin": float(spin), "p": float(h.mean() * 100), "meanMiss": mm})
        print(f"spin={spin}: B={B} 済 ({time.time()-t0:.0f}s)")

    best_p = max(r["p"] for r in rows)
    band = [r for r in rows if r["p"] >= best_p - 6]
    band.sort(key=lambda r: r["meanMiss"])
    best = band[0]
    rows.sort(key=lambda r: (-r["p"], r["meanMiss"]))
    print(f"\n=== {kind} ベスト: v={best['a']} 仰角={best['b']}° spin={best['spin']} → {best['p']:.0f}% 中心ズレ{best['meanMiss']:.3f}m")
    for r in rows[:10]:
        print(f"  v={r['a']:5.2f} 仰={r['b']:4.1f}° spin={r['spin']:.1f} → {r['p']:5.1f}%  miss={r['meanMiss']:.3f}")

    out = Path(__file__).resolve().parent.parent / "out"
    out.mkdir(exist_ok=True)
    (out / f"opt-{kind}{suffix}.json").write_text(json.dumps({
        "cfg": {"kind": kind, "trials": trials, "D": sc["D"], "nipLen": BELT_NIP, "engine": engine},
        "best": {"belt": best},
        "grid": rows,
    }, indent=1, ensure_ascii=False))
    print(f"→ out/opt-{kind}{suffix}.json ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
