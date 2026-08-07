"""組立の整合性チェック: **食い込み**と**浮き**.

    python scripts/consistency.py [--md] [--pose match|stowed|loading|...]

`interference_full.py` は「離れているべき組が近すぎないか」を見る。
だがそれだけでは、実機として成立しない状態を2つ見逃す。

## 1. 食い込み（実体の重なり）

`distance_to`（BRepExtrema）は**接触も食い込みも 0.0 を返す**。区別できない。
そのうえ従来は「接触してよい」関係（BOLTED 等）を**測らずに飛ばして**いた。
結果、2020 の押出材同士が交差部で 10mm 素通しに重なったまま 20 組通っていた。

    2294 mm3  deck_2 ↔ deck_3      台座リングプレートが横梁の中に埋まっている
    1885 mm3  base_frame_2 ↔ base_frame_5   井桁の横梁が内桁を貫通
     942 mm3  mast_0 ↔ mast_1

実機ではアルミフレームは**突き合わせてブラケットで留める**か、片方を切り欠く。
素通しはあり得ない。重なりは**ブーリアンで体積を測る**しかない。

## 2. 浮き（どこにも留まっていない部品）

CAD では座標を書けば部品は空中に置ける。組み立てられるかは別。
全ソリッドを「接触しているか」で結んだグラフを作り、**連結成分**を見る。
本体成分に入らないソリッドは、実機では**手で支えないと落ちる**。

なお「接触している ≠ 締結されている」なので、これは必要条件の検査。
ボルトの有無は `fasteners.py` が別に見る。
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

POSES = {
    "match": lambda: P.POSE_MATCH,
    "stowed": lambda: P.POSE_STOWED,
    "loading": lambda: P.POSE_LOADING,
}

# 接触とみなす最大すきま [mm]。
# 3Dプリント部品の公差・板金の曲げ戻りを考えると 0.2mm 程度は実機でも埋まる。
CONTACT_TOL = 0.5

# ブーリアンを走らせる bbox 重なりの下限 [mm³]。
# bbox の重なりは実体の重なりの**上限**なので、これ未満なら測るまでもない。
BOOL_MIN = 20.0


def bbox_overlap(ba, bb):
    v = 1.0
    for lo_a, hi_a, lo_b, hi_b in (
            (ba.min.X, ba.max.X, bb.min.X, bb.max.X),
            (ba.min.Y, ba.max.Y, bb.min.Y, bb.max.Y),
            (ba.min.Z, ba.max.Z, bb.min.Z, bb.max.Z)):
        v *= max(0.0, min(hi_a, hi_b) - max(lo_a, lo_b))
    return v


def overlap_volume(sa, sb):
    """実体の重なり体積 [mm³]。

    ⚠ build123d の `&` は**交差が空だと None を返す**。`.volume` を直に
    呼ぶと AttributeError になる。`except Exception: v = 0.0` で握り潰すと
    「本当に交差が無い」と「ブーリアンが落ちた」が同じ 0 になり、
    検査が黙って効かなくなる。空は空として扱い、失敗は失敗として上げる。
    """
    r = sa & sb
    if r is None:
        return 0.0
    return r.volume


def bbox_gap(ba, bb):
    dx = max(0.0, ba.min.X - bb.max.X, bb.min.X - ba.max.X)
    dy = max(0.0, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y)
    dz = max(0.0, ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def analyze(pose_name):
    sol = V.solids_with_bbox(A.build(POSES[pose_name]()))
    n = len(sol)
    idx = {i: sol[i][0] for i in range(n)}

    # --- 接触グラフ（Union-Find）と食い込みを一度の走査で ---
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    dig = []          # 食い込み
    stats = {"total": 0, "bool": 0, "dist": 0}
    for i, j in itertools.combinations(range(n), 2):
        na, sa, ba = sol[i]
        nb, sb, bb = sol[j]
        stats["total"] += 1
        ov = bbox_overlap(ba, bb)
        if ov > BOOL_MIN:
            ga, gb = R.group_of(na), R.group_of(nb)
            kind, _need, _touch, ov_ok = R.expected(ga, gb)
            v = overlap_volume(sa, sb)
            stats["bool"] += 1
            if v > 1.0:                     # 実体が重なっている＝当然つながっている
                union(i, j)
                if v > ov_ok:
                    dig.append((v, kind, ga, gb,
                                na.split("/")[-1], nb.split("/")[-1]))
                continue
            # ⚠ bbox が重なっていても実体は重なっていないことがある。ここで
            #   `continue` すると、実体が 0.00mm で**接している**組が連結されず
            #   「浮き」に出る。bbox の重なりは距離 0 を意味しない。落とさず測る。
        elif bbox_gap(ba, bb) > CONTACT_TOL:
            continue
        try:
            d = sa.distance_to(sb)
        except Exception:
            continue
        stats["dist"] += 1
        if d <= CONTACT_TOL:
            union(i, j)

    # --- 連結成分 ---
    comps = {}
    for i in range(n):
        comps.setdefault(find(i), []).append(i)
    order = sorted(comps.values(), key=len, reverse=True)
    body, floating = order[0], order[1:]
    return n, stats, dig, body, floating, idx


def report_md(rows):
    print("組立の整合性チェック。`interference_full.py` が見ない2点を見る。\n")
    print("- **食い込み** … 実体の重なり体積をブーリアンで測る。"
          "`distance_to` は接触と食い込みをどちらも 0.0 と返すので区別できない\n"
          "- **浮き** … 接触グラフの連結成分。本体につながらない部品は実機で落ちる\n")
    print(f"接触とみなすすきま: **{CONTACT_TOL} mm**\n")
    print("| 姿勢 | ソリッド | ブーリアン実行 | **食い込み** | **浮き** | 時間 |")
    print("|---|---|---|---|---|---|")
    for nm, n, st, dig, body, floating, idx, dt in rows:
        nf = sum(len(c) for c in floating)
        print(f"| {nm} | {n} | {st['bool']:,} | **{len(dig)}** | **{nf}** | {dt:.1f}s |")
    tot = sum(len(r[3]) + sum(len(c) for c in r[5]) for r in rows)
    print()
    if tot == 0:
        print("**問題なし** ✅")
    for nm, n, st, dig, body, floating, idx, dt in rows:
        if dig:
            print(f"\n### {nm}: 食い込み {len(dig)} 件\n")
            print("| 重なり | 関係 | 許容 | 部品 A | 部品 B |")
            print("|---|---|---|---|---|")
            for v, kind, ga, gb, la, lb in sorted(dig, reverse=True)[:40]:
                ok = R.KINDS[kind][2]
                print(f"| {v:,.0f} mm³ | {kind} ({ga}↔{gb}) | {ok:,.0f} mm³ | {la} | {lb} |")
            if len(dig) > 40:
                print(f"\n（他 {len(dig) - 40} 件）")
        if floating:
            nf = sum(len(c) for c in floating)
            print(f"\n### {nm}: 浮き {nf} ソリッド / {len(floating)} 塊"
                  f"（本体は {len(body)} ソリッド）\n")
            print("| 塊 | ソリッド数 | 中身 |")
            print("|---|---|---|")
            for c in sorted(floating, key=len, reverse=True)[:25]:
                names = sorted({idx[i].split("/")[-1].split("#")[0] for i in c})
                s = "、".join(names[:6]) + ("…" if len(names) > 6 else "")
                print(f"| {len(c)} 個 | {len(c)} | {s} |")
            if len(floating) > 25:
                print(f"\n（他 {len(floating) - 25} 塊）")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="all", choices=["all", *POSES])
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    rows = []
    for nm in (list(POSES) if args.pose == "all" else [args.pose]):
        t0 = time.time()
        rows.append((nm, *analyze(nm), time.time() - t0))

    if args.md:
        report_md(rows)
    else:
        for nm, n, st, dig, body, floating, idx, dt in rows:
            nf = sum(len(c) for c in floating)
            print(f"[{nm:8s}] ソリッド{n} ブーリアン{st['bool']:,} "
                  f"→ 食い込み {len(dig)} 件 / 浮き {nf} 個 ({dt:.1f}s)")
            for v, kind, ga, gb, la, lb in sorted(dig, reverse=True)[:12]:
                print(f"    食い込み {v:8,.0f}mm³ {kind:9s} {la} ↔ {lb}")
            for c in sorted(floating, key=len, reverse=True)[:12]:
                names = sorted({idx[i].split('/')[-1].split('#')[0] for i in c})
                print(f"    浮き {len(c):3d}個  {'、'.join(names[:5])}")
    bad = sum(len(r[3]) + sum(len(c) for c in r[5]) for r in rows)
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
