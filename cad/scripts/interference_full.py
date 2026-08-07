"""全ソリッド総当たりの干渉チェック（関係性ベース）.

    python scripts/interference_full.py [--md] [--pose match|stowed|loading|yaw+30|yaw-30]

従来の `validate.py` は**手で列挙した17ペア**しか見ていなかった。
283 ソリッドの総当たりは 39,903 組あり、列挙外の組み合わせは
**一度も検査されていなかった**。

ここでは全組を見る。ただし 355 組は実体が接触していて、その大半は
**接触していて正しい**（ボルトが板を留める、車輪が軸に付く…）。
そこで `tr_relations.py` の**関係宣言**から期待すきまを決める。

    宣言がある   → その関係が期待する状態か（BOLTED なら接触、FREE なら 3mm 離れる）
    宣言が無い   → **FREE 扱い**。3mm 離れていなければ NG

つまり**関係の書き忘れは「素通り」ではなく「NG」として出る**。
これが 17ペア方式との決定的な違い。
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
    "yaw+30": lambda: dict(P.POSE_MATCH, yaw=P.YAW_LIMIT, pitch=60.0),
    "yaw-30": lambda: dict(P.POSE_MATCH, yaw=-P.YAW_LIMIT, pitch=P.PITCH_MIN),
}


def bbox_gap(ba, bb):
    """bbox 同士の距離。実体の距離の下限になる（安全側の粗ふるい）。"""
    dx = max(0.0, ba.min.X - bb.max.X, bb.min.X - ba.max.X)
    dy = max(0.0, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y)
    dz = max(0.0, ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def check(pose_name):
    sol = V.solids_with_bbox(A.build(POSES[pose_name]()))
    n = len(sol)
    bad, stats = [], {"total": 0, "screened": 0, "measured": 0}
    for (na, sa, ba), (nb, sb, bb) in itertools.combinations(sol, 2):
        stats["total"] += 1
        ga, gb = R.group_of(na), R.group_of(nb)
        kind, need, may_touch, ov_ok = R.expected(ga, gb)
        la, lb = na.split("/")[-1], nb.split("/")[-1]

        # --- ① 食い込み（実体の重なり）---
        # ⚠ `distance_to` は接触も食い込みも 0.0 を返す。**区別できない**。
        #   「接触してよい」関係を測らずに飛ばしていたので、押出材同士が
        #   10mm 食い込んでいても通っていた。重なりはブーリアンで測る。
        ov_bbox = 1.0
        for lo_a, hi_a, lo_b, hi_b in (
                (ba.min.X, ba.max.X, bb.min.X, bb.max.X),
                (ba.min.Y, ba.max.Y, bb.min.Y, bb.max.Y),
                (ba.min.Z, ba.max.Z, bb.min.Z, bb.max.Z)):
            ov_bbox *= max(0.0, min(hi_a, hi_b) - max(lo_a, lo_b))
        # bbox の重なりは実体の重なりの上限。許容以下なら測るまでもない
        if ov_bbox > max(ov_ok, 50.0):
            try:
                v = (sa & sb).volume
            except Exception:
                v = 0.0
            stats["measured"] += 1
            if v > ov_ok:
                bad.append((-v, kind, f"食い込み {v:.0f}mm³", ga, gb, la, lb))
                continue

        # --- ② すきま（離れているべき組）---
        if may_touch:
            stats["screened"] += 1
            continue
        if bbox_gap(ba, bb) >= need:
            stats["screened"] += 1
            continue
        try:
            d = sa.distance_to(sb)
        except Exception:
            continue
        stats["measured"] += 1
        if d < need:
            bad.append((d, kind, f"すきま {d:.2f}mm（要 {need:.1f}）", ga, gb, la, lb))
    return n, stats, bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="all", choices=["all", *POSES])
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    names = list(POSES) if args.pose == "all" else [args.pose]
    all_bad, lines = {}, []
    for nm in names:
        t0 = time.time()
        n, st, bad = check(nm)
        all_bad[nm] = bad
        lines.append((nm, n, st, len(bad), time.time() - t0))

    if args.md:
        print("**全ソリッド総当たり**の干渉チェック。"
              "`tr_relations.py` の関係宣言から期待すきまを決める。\n")
        print("> 関係が宣言されていない組は **FREE**（3mm 離れていること）として扱う。"
              "つまり**宣言の書き忘れは「素通り」ではなく NG になる**。\n")
        print("| 姿勢 | ソリッド | 総組数 | 粗ふるい通過 | 実測 | **NG** | 時間 |")
        print("|---|---|---|---|---|---|---|")
        for nm, n, st, nbad, dt in lines:
            print(f"| {nm} | {n} | {st['total']:,} | "
                  f"{st['total'] - st['screened']:,} | {st['measured']:,} | "
                  f"**{nbad}** | {dt:.1f}s |")
        tot = sum(len(b) for b in all_bad.values())
        print()
        if tot == 0:
            print("**NG なし** ✅")
        for nm, bad in all_bad.items():
            if not bad:
                continue
            print(f"\n### {nm}: {len(bad)} 件\n")
            print("| すきま | 関係 | 症状 | 部品 A | 部品 B |")
            print("|---|---|---|---|---|")
            for d, kind, sym, ga, gb, la, lb in sorted(bad)[:40]:
                print(f"| {d:.2f} mm | {kind} ({ga}↔{gb}) | {sym} | {la} | {lb} |")
            if len(bad) > 40:
                print(f"\n（他 {len(bad) - 40} 件）")
    else:
        for nm, n, st, nbad, dt in lines:
            print(f"[{nm:8s}] ソリッド{n} 総当たり{st['total']:,} "
                  f"実測{st['measured']:,} → NG {nbad} 件 ({dt:.1f}s)")
        for nm, bad in all_bad.items():
            for d, kind, sym, ga, gb, la, lb in sorted(bad)[:15]:
                print(f"    [{nm}] {d:6.2f}mm {kind:9s} {sym}  "
                      f"{la} ↔ {lb}  ({ga}↔{gb})")
    return 1 if any(all_bad.values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
