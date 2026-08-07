"""浮いている部品を、**何mm浮いているか**まで含めて出す.

    python scripts/floating.py [--md] [--pose match|stowed|loading]

`consistency.py` は接触グラフの連結成分で「浮いている塊」を列挙するが、
それだけでは直せない。知りたいのは

    その塊は、**どの部品に、あと何mm 寄せれば載るのか**

だから、浮いている塊ごとに「本体側で最も近い部品」と「その距離」を出す。
0.6mm なら座標の丸め、40mm なら設計そのものが抜けている、と切り分けられる。

⚠ 「接触している」は「組める」の必要条件でしかない。
   ここが通っても、締結（ねじ・ブラケット）は `fasteners.py` が別に見る。
"""

from __future__ import annotations

import argparse
import itertools
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_assembly as A  # noqa: E402
import tr_params as P  # noqa: E402
import tr_relations as R  # noqa: E402
import validate as V  # noqa: E402
from consistency import CONTACT_TOL, bbox_gap, bbox_overlap, overlap_volume  # noqa: E402

POSES = {"match": lambda: P.POSE_MATCH,
         "stowed": lambda: P.POSE_STOWED,
         "loading": lambda: P.POSE_LOADING}

# 「近い相手」を探す打ち切り距離 [mm]。これより遠い相手は原因の候補にならない
SEARCH = 60.0


def analyze(pose_name):
    sol = V.solids_with_bbox(A.build(POSES[pose_name]()))
    n = len(sol)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    # --- パス1: 接触だけ見て連結成分を作る（すきま 0.5mm 以内に絞るので速い）---
    # ⚠ 最初から半径 60mm で全 68,000 組に distance_to を掛けたら終わらなかった。
    #   浮いているのは全体の一部なので、**浮いた塊だけ**を対象に探索し直す。
    for i, j in itertools.combinations(range(n), 2):
        _, sa, ba = sol[i]
        _, sb, bb = sol[j]
        if bbox_overlap(ba, bb) > 20.0:
            if overlap_volume(sa, sb) > 1.0:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[rj] = ri
                continue
            # ⚠ bbox が重なっていても実体は重なっていないことがある。
            #   ここで `continue` していたので、実体が 0.00mm で**接している**組が
            #   連結されず「浮き」として出ていた（turret ↔ turret#11 など）。
            #   bbox の重なりは距離が 0 であることを意味しない。落とさず測る。
        elif bbox_gap(ba, bb) > CONTACT_TOL:
            continue
        try:
            d = sa.distance_to(sb)
        except Exception:
            continue
        if d <= CONTACT_TOL:
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[rj] = ri

    comps = {}
    for i in range(n):
        comps.setdefault(find(i), []).append(i)
    order = sorted(comps.values(), key=len, reverse=True)
    body = order[0]

    # --- パス2: 浮いた塊ごとに、本体側の最近傍を探す ---
    rows = []
    for c in order[1:]:
        best = (float("inf"), None, None)
        for i in c:
            _, sa, ba = sol[i]
            for j in body:
                _, sb, bb = sol[j]
                if bbox_gap(ba, bb) > min(SEARCH, best[0]):
                    continue
                try:
                    d = sa.distance_to(sb)
                except Exception:
                    continue
                if d < best[0]:
                    best = (d, i, j)
        names = sorted({sol[i][0].split("/")[-1].split("#")[0] for i in c})
        d, i, j = best
        rows.append((d, len(c), names,
                     sol[i][0].split("/")[-1] if i is not None else "—",
                     sol[j][0].split("/")[-1] if j is not None else "—"))
    return n, len(body), sorted(rows, reverse=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="match", choices=list(POSES) + ["all"])
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    out = []
    for nm in (list(POSES) if args.pose == "all" else [args.pose]):
        t0 = time.time()
        out.append((nm, *analyze(nm), time.time() - t0))

    if args.md:
        print("浮いている部品と、**本体へあと何mm 寄せれば載るか**。\n")
        print(f"> 接触とみなすすきま {CONTACT_TOL} mm、原因の探索半径 {SEARCH} mm。"
              f"`∞` は探索半径内に本体側の部品が無い＝**置き場所ごと決まっていない**。\n")
        print("| 姿勢 | ソリッド | 本体 | **浮き** | 塊 | 時間 |")
        print("|---|---|---|---|---|---|")
        for nm, n, nb, rows, dt in out:
            print(f"| {nm} | {n} | {nb} | **{n - nb}** | {len(rows)} | {dt:.1f}s |")
        for nm, n, nb, rows, dt in out:
            if not rows:
                continue
            print(f"\n### {nm}\n")
            print("| すきま | 個数 | 浮いている部品 | 最も近い本体側の部品 |")
            print("|---|---|---|---|")
            for d, cnt, names, a, b in rows[:40]:
                gap = "∞" if d == float("inf") else f"{d:.2f} mm"
                s = "、".join(names[:5]) + ("…" if len(names) > 5 else "")
                print(f"| {gap} | {cnt} | {s} | {b} |")
            if len(rows) > 40:
                print(f"\n（他 {len(rows) - 40} 塊）")
    else:
        for nm, n, nb, rows, dt in out:
            print(f"[{nm}] ソリッド{n} 本体{nb} 浮き{n - nb}個 / {len(rows)}塊 ({dt:.1f}s)")
            for d, cnt, names, a, b in rows[:40]:
                gap = "  ∞  " if d == float("inf") else f"{d:6.2f}"
                print(f"  {gap}mm x{cnt:<3d} {'、'.join(names[:4]):46s} → {b}")
    return 1 if any(o[1] != o[2] for o in out) else 0


if __name__ == "__main__":
    raise SystemExit(main())
