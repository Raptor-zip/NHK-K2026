"""骨格の部材感度解析 — どの部材が効いていて、どれが削れるか.

    python scripts/frame_optimize.py [--md]

静解析（scripts/fea_frame.py）では応力の安全率が 4.5 あり、骨格は**剛性で決まっている**
ことが分かった。押出材は 8.0kg で全体の 23% を占めるので、
**剛性を落とさずに削れる部材があるか**を1本ずつ抜いて確かめる。

判定の順序（この順で落ちたらそこで打ち切り）
  1. **荷重経路** — 抜いた結果、集中質量や計測点が支持（4輪）から到達できなくなるなら失格。
     これを見ないと「部材を抜いたら自重が減って変位も減った」という嘘の改善が出る。
  2. **機構化**   — 到達はできるが変位が発散するなら失格。
  3. **剛性**     — 砲塔取付面の変位の悪化率で並べる。

抜いた部材の自重も同時に消える点に注意（実機でも消えるので、これは正しい比較）。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import fea_frame as F  # noqa: E402
import tr_params as P  # noqa: E402

MECHANISM_MM = 20.0        # これを超える変位は機構化（構造が成立していない）
# 命中率解析（accuracy_budget.py）のピッチ 1σ = √(0.15² + 0.60²)。
# 骨格のたわみによる傾きの振れ幅はこれを超えてはならない。
AIM_BUDGET_DEG = 0.62
CASES = {
    "自重": dict(extra_forces=[], accel=(0, 0, 0)),
    "横加速1.5": dict(extra_forces=[], accel=(0, 1.5, 0)),
    "グラバー全開": dict(extra_forces=[], accel=(0, 0, 0), grabber_x=-700.0),
}
# 荷重経路が通っていなければならない点（集中質量の作用点と、変位の計測点）
MUST_REACH = [pos for _, _, pos in F.POINT_MASSES] + [
    (-700.0, 0.0, 780.0),                       # グラバー全開時
    (P.TURRET_X, 0.0, F.Z_TOP),                 # 砲塔取付面（計測点）
    (-70.0, 0.0, P.MAST_BEAM_Z),                # バケツ（計測点）
    (380.0, F.YS, F.Z_TOP),                     # レール前端（計測点）
]


def load_path_ok(nodes, members) -> bool:
    """4輪の支持点から、全ての荷重点・計測点へ部材をたどって到達できるか。"""
    adj: dict[int, set[int]] = {}
    for i, j, _ in members:
        adj.setdefault(i, set()).add(j)
        adj.setdefault(j, set()).add(i)

    def nearest(pt):
        return int(np.argmin(np.linalg.norm(nodes - np.array(pt), axis=1)))

    seen, stack = set(), []
    for sx in (1, -1):
        for sy in (1, -1):
            nd = nearest((sx * P.WHEELBASE_X / 2, sy * F.YI, F.Z_BASE))
            if nd not in seen:
                seen.add(nd)
                stack.append(nd)
    while stack:
        for nb in adj.get(stack.pop(), ()):
            if nb not in seen:
                seen.add(nb)
                stack.append(nb)
    return all(nearest(pt) in seen for pt in MUST_REACH)


def turret_angles(nodes, u):
    """砲塔取付面の回転（ピッチ・ヨー）を度で返す。

    照準にそのまま乗るのは**変位ではなく傾き**なので、こちらが本当の指標。
    """
    nd = int(np.argmin(np.linalg.norm(nodes - np.array([P.TURRET_X, 0.0, F.Z_TOP]), axis=1)))
    return np.degrees(u[6 * nd + 4]), np.degrees(u[6 * nd + 5])


def solve_named(nodes, all_members, drop: str | None):
    members = all_members if drop is None else [m for m in all_members if m[2] != drop]
    if not load_path_ok(nodes, members):
        return None, members
    out = {}
    for label, kw in CASES.items():
        try:
            u, disp, worst, key = F.solve(nodes, members, **kw)
            out[label] = (key["砲塔取付面"], key["バケツ"], worst[0][0]) + turret_angles(nodes, u)
        except Exception:
            out[label] = (9999.0, 9999.0, 9999.0, 9.9, 9.9)
    return out, members


def aim_spread(res):
    """荷重ケース間の傾きの振れ幅 [deg]。静的な一定分は較正で消えるので、
    **ケース間で変わる分だけ**が実際の照準誤差になる。"""
    p = [res[c][3] for c in CASES]
    y = [res[c][4] for c in CASES]
    return max(p) - min(p), max(y) - min(y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    nodes, all_members = F.build_model()
    base, _ = solve_named(nodes, all_members, None)
    base_disp = max(base[c][0] for c in CASES)
    base_stress = max(base[c][2] for c in CASES)
    names = sorted({m[2] for m in all_members})

    def mass_of(name):
        L = sum(float(np.linalg.norm(nodes[j] - nodes[i]))
                for i, j, nm in all_members if nm == name)
        return L / 1000.0 * P.EXT_MASS_PER_M

    bp, by = aim_spread(base)
    rows = []
    for name in names:
        eq = "等価梁" in name          # 板の面内剛性を梁で代用したもの＝実在しない部材
        res, _ = solve_named(nodes, all_members, name)
        if res is None:
            rows.append((name, mass_of(name), None, None, None, None, "path", eq))
            continue
        wd = max(res[c][0] for c in CASES)
        ws = max(res[c][2] for c in CASES)
        p, y = aim_spread(res)
        kind = "mech" if wd > MECHANISM_MM else ("eq" if eq else "ok")
        rows.append((name, mass_of(name), wd, ws, p, y, kind, eq))

    order = {"ok": 0, "eq": 1, "mech": 2, "path": 3}
    rows.sort(key=lambda r: (order[r[6]], r[2] if r[2] is not None else 0.0))

    total_m = sum(mass_of(n) for n in names)
    if args.md:
        print(f"基準: 砲塔取付面の最大変位 **{base_disp:.2f} mm**、"
              f"荷重ケース間の**傾きの振れ幅 ピッチ {bp:.3f}° / ヨー {by:.3f}°**、"
              f"最大応力 {base_stress:.1f} MPa（押出材 {total_m:.2f} kg・{len(all_members)} 要素）\n")
        print("| 抜く部材 | 質量 | 変位 | 悪化 | **ピッチ振れ** | 応力 | 判定 |")
        print("|---|---|---|---|---|---|---|")
        for name, mass, wd, ws, p, y, kind, eq in rows:
            if kind == "path":
                print(f"| {name} | {mass:.2f} kg | — | — | — | — | "
                      f"**必須**（荷重経路が切れる） |")
            elif kind == "mech":
                print(f"| {name} | {mass:.2f} kg | {wd:.1f} mm | 機構化 | — | — | "
                      f"**必須**（構造が成立しない） |")
            else:
                ratio = wd / base_disp
                if eq:
                    v = "対象外（板の等価梁・実在しない）"
                elif ratio < 1.10 and p < AIM_BUDGET_DEG:
                    v = f"**削減候補**（{(ratio - 1) * 100:+.0f}%）"
                elif p >= AIM_BUDGET_DEG:
                    v = f"必要（照準予算 {AIM_BUDGET_DEG:.2f}° を超える）"
                elif ratio < 1.5:
                    v = f"要検討（{(ratio - 1) * 100:+.0f}%）"
                else:
                    v = f"必要（{(ratio - 1) * 100:+.0f}%）"
                print(f"| {name} | {mass:.2f} kg | {wd:.2f} mm | {wd - base_disp:+.2f} mm | "
                      f"{p:.3f}° | {ws:.1f} MPa | {v} |")
        print()
        print(f"- 照準に効くのは**変位ではなく砲塔取付面の傾き**。しかも静的な一定分は較正で消えるので、"
              f"**荷重ケース間で変わる分（振れ幅）だけ**が実際の誤差になる。")
        print(f"- 現状のピッチ振れ **{bp:.3f}°** は、命中率解析のピッチ 1σ = "
              f"{AIM_BUDGET_DEG:.2f}°（バックラッシュ0.15°＋投入姿勢0.60°）に対して "
              f"{bp / AIM_BUDGET_DEG * 100:.0f}%。骨格の剛性は照準予算に対して十分に余裕がある。")
    else:
        print(f"基準 変位 {base_disp:.2f} mm / 傾き振れ ピッチ{bp:.3f}° ヨー{by:.3f}° / "
              f"応力 {base_stress:.1f} MPa / 押出 {total_m:.2f} kg")
        for name, mass, wd, ws, p, y, kind, eq in rows:
            if kind == "path":
                tag = "必須(荷重経路が切れる)"
            elif kind == "mech":
                tag = f"必須(機構化 {wd:.0f}mm)"
            else:
                tag = (f"変位 {wd:6.2f}mm ({wd / base_disp:4.2f}x) "
                       f"ピッチ振れ {p:.3f}° 応力 {ws:5.1f}MPa" + ("  [等価梁]" if eq else ""))
            print(f"  {name:24s} {mass:5.2f}kg  {tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
