"""GPU の内外判定が OCC と同じ答えを出すか、どれだけ速いかを見る.

    python scripts/gpu_bench.py [--parts 40] [--pts 4000]

⚠ **速いだけでは使えない。** `screw_place` の結果（ねじの位置）は内外判定に
  直結するので、**答えが変わらないこと**を先に確かめる。ここでは組んだ実体
  から部品を選び、その bbox に一様な点をばらまいて 2 つの判定を突き合わせる。

見るもの:
  * 食い違い率（境界からの距離ごとに分けて出す。メッシュ化の弦公差ぶん、
    面のすぐそばは食い違って当たり前）
  * 1 点あたりの時間（OCC は 1 点ずつ、GPU はまとめて）
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parts", type=int, default=40)
    ap.add_argument("--pts", type=int, default=4000)
    ap.add_argument("--tol", type=float, default=None)
    args = ap.parse_args()

    import gpu_geom as G
    import screw_place as SP
    import tr_assembly as A
    import tr_fix as F
    import validate as V

    print(G.info())
    tol = args.tol if args.tol is not None else G.TOL

    A.AUTO_SCREWS = False
    t0 = time.time()
    shape = A.build(dict(yaw=0.0, pitch=0.0, grab=0.0, press=0.0, tilt=0.0))
    print(f"build {time.time() - t0:.1f}s")

    known = set(F.BOX)
    sol: dict[str, list] = {}
    for n, sd, _b in V.solids_with_bbox(shape):
        seg = [q.split("#")[0] for q in n.split("/")]
        nm = next((q for q in reversed(seg) if q in known), seg[-1])
        sol.setdefault(nm, []).append(sd)

    names = sorted(sol)
    step = max(1, len(names) // args.parts)
    picked = names[::step][:args.parts]
    print(f"部品 {len(sol)} 個のうち {len(picked)} 個を見る / 点 {args.pts} 個ずつ")

    rng = np.random.default_rng(12345)
    tot = mismatch = 0
    near = 0                      # 面のすぐそば（弦公差以内）での食い違い
    t_cpu = t_gpu = t_mesh = 0.0
    n_tri = 0
    for nm in picked:
        ss = sol[nm]
        t0 = time.time()
        gi = G.GpuInside(ss, tol=tol)
        t_mesh += time.time() - t0
        n_tri += gi.n_tri
        if not gi.n_tri:
            continue
        lo, hi = gi.lo - 1.0, gi.hi + 1.0
        pts = rng.uniform(lo, hi, size=(args.pts, 3)).astype(np.float32)

        t0 = time.time()
        g = gi.many(pts)
        t_gpu += time.time() - t0

        ci = SP.Inside(ss)
        t0 = time.time()
        c = np.array([ci(list(map(float, p))) for p in pts], dtype=bool)
        t_cpu += time.time() - t0

        bad = np.nonzero(g != c)[0]
        tot += len(pts)
        mismatch += len(bad)
        # 食い違った点が「面のすぐそば」かどうか … ±(tol*3) ずらして
        # OCC の答えが変わるなら境界の揺れとみなす
        for i in bad[:200]:
            p = pts[i]
            d = (G.RAY * max(tol * 3.0, 0.15)).astype(np.float32)
            if ci(list(map(float, p + d))) != ci(list(map(float, p - d))):
                near += 1

    print(f"\nメッシュ化   {t_mesh:6.2f}s（三角形 {n_tri:,}）")
    print(f"OCC   1点ずつ {t_cpu:6.2f}s  → {t_cpu / max(tot, 1) * 1e6:8.1f} µs/点")
    print(f"GPU   まとめて {t_gpu:6.2f}s  → {t_gpu / max(tot, 1) * 1e6:8.1f} µs/点")
    if t_gpu > 0:
        print(f"倍率 {t_cpu / t_gpu:.0f}x（メッシュ化込みで "
              f"{t_cpu / (t_gpu + t_mesh):.1f}x）")
    print(f"\n食い違い {mismatch} / {tot} 点 = {mismatch / max(tot, 1) * 100:.3f}%"
          f"（うち境界の揺れ {near}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
