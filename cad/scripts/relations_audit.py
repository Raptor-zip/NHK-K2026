"""関係宣言の網羅性を監査する.

    python scripts/relations_audit.py [--md]

`tr_relations.py` は**宣言が無い組を FREE（3mm 離れていること）**として扱う。
これは「書き忘れが素通りしない」ための設計だが、裏を返すと
**未宣言の組が大量にあっても、たまたま離れていれば NG にならない**。

設計を変えて近づいた瞬間に初めて NG が出て、そこで「これは接触してよい
関係だった」と気づくことになる。それでも検出はできるので破綻はしないが、
どの組が「意図的に FREE」でどの組が「まだ考えていない」かは区別したい。

ここでは
  * 実在するグループの全対を列挙する
  * 宣言済み / 未宣言 に分ける
  * 未宣言の組について、全姿勢での**最小すきま**を測る
    → 近いのに未宣言なら、それは考えるべき組
    → 遠いなら FREE のままでよい
"""

from __future__ import annotations

import argparse
import itertools
import os
import sys

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
NEAR = 30.0    # これより近い未宣言の組は「考えるべき」として挙げる


def bbox_gap(ba, bb):
    dx = max(0.0, ba.min.X - bb.max.X, bb.min.X - ba.max.X)
    dy = max(0.0, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y)
    dz = max(0.0, ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    # 全姿勢を通した、グループ対ごとの最小すきま
    gap = {}
    groups = set()
    for pname, mk in POSES.items():
        sol = V.solids_with_bbox(A.build(mk()))
        for n, s, b in sol:
            groups.add(R.group_of(n))
        for (na, sa, ba), (nb, sb, bb) in itertools.combinations(sol, 2):
            ga, gb = R.group_of(na), R.group_of(nb)
            if ga == gb:
                key = (ga, gb)
            else:
                key = tuple(sorted((ga, gb)))
            lo = bbox_gap(ba, bb)
            if lo > NEAR:
                continue
            if lo >= gap.get(key, 1e9):
                continue
            try:
                d = sa.distance_to(sb)
            except Exception:
                continue
            if d < gap.get(key, 1e9):
                gap[key] = d

    gs = sorted(groups)
    declared = {tuple(sorted(k)) for k in R.RELATIONS}
    rows_near, rows_far, rows_dec = [], [], []
    for a, b in itertools.combinations_with_replacement(gs, 2):
        key = tuple(sorted((a, b)))
        d = gap.get(key)
        if key in declared:
            rows_dec.append((a, b, R.relation(a, b), d))
        elif d is not None and d < NEAR:
            rows_near.append((d, a, b))
        else:
            rows_far.append((a, b))
    rows_near.sort()

    if args.md:
        n_all = len(gs) * (len(gs) + 1) // 2
        print(f"グループ **{len(gs)}** 個 → 対は **{n_all}** 通り。"
              f"宣言済み **{len(rows_dec)}** / 未宣言 **{n_all - len(rows_dec)}**\n")
        print("> 未宣言の組は **FREE**（3mm 離れていること）として扱われる。"
              "遠く離れている組なら FREE のままでよい。\n")
        if rows_near:
            print(f"### 未宣言だが {NEAR:.0f}mm 以内に近づく組（**要検討**）\n")
            print("| 最小すきま | グループ A | グループ B | 判断 |")
            print("|---|---|---|---|")
            for d, a, b in rows_near:
                v = "⚠ **3mm 未満**。関係を宣言するか設計を直す" if d < 3.0 else "余裕あり"
                print(f"| {d:.2f} mm | {a} | {b} | {v} |")
        else:
            print(f"### 未宣言の組はすべて {NEAR:.0f}mm 以上離れている ✅\n")
        print(f"\n### 宣言済み {len(rows_dec)} 組\n")
        print("| グループ A | グループ B | 関係 | 全姿勢の最小すきま |")
        print("|---|---|---|---|")
        for a, b, k, d in sorted(rows_dec):
            ds = "—（30mm 超）" if d is None else f"{d:.2f} mm"
            print(f"| {a} | {b} | `{k}` | {ds} |")
    else:
        print(f"グループ {len(gs)} → 対 {len(gs) * (len(gs) + 1) // 2} "
              f"（宣言済み {len(rows_dec)} / 未宣言 {len(rows_near) + len(rows_far)}）")
        print(f"\n未宣言だが {NEAR:.0f}mm 以内: {len(rows_near)} 組")
        for d, a, b in rows_near:
            mark = "⚠" if d < 3.0 else " "
            print(f"  {mark} {d:7.2f}mm  {a} ↔ {b}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
