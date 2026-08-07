"""装填の成功率をモンテカルロで見積もる（戦略書 8/29 Go/No-Go 判定の事前評価）.

    python sim/tr_montecarlo.py [--n 20] [--md]

実戦で必ず出るばらつきを入力して、山の一括回収が成立する割合を出す。

ばらつきの根拠
  * 山の置き位置 ±20mm / ±5° … 人間が場外から整える精度（FAQ 4.1 Q2 で整えは合法）
  * 机の設置    ±10mm       … 競技用品の設営誤差
  * 車体の停止  ±10mm       … LiDAR 自己位置推定 + ToF 併用時の目標精度

判定
  成功 = 10枚すべてが櫛歯の矩形内にあり把持できた かつ 机の移動が 5mm 未満
  （机を動かすと 4.1.4a で反則。机を単一剛体でモデル化したので基準走行での
    移動量は 0.0mm。閾値は数値誤差ぶんの 2mm とする）

注意
  雑巾は剛体プレート近似なので、この成功率は**機構配置の頑健性**の指標であって
  実機の回収成功率ではない。実測は 8/29 の実験で取ること。
"""

from __future__ import annotations

import argparse
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import pybullet as p  # noqa: E402

import tr_sim as S  # noqa: E402

RANGES = {
    "stack_dx": 0.020,
    "stack_dy": 0.020,
    "stack_yaw": 5.0,
    "desk_dx": 0.010,
    "desk_dy": 0.010,
    "base_err": 0.010,
}
DESK_TOL = 0.002


def trial(seed: int, t_end: float = 12.0, scale: float = 1.0):
    rng = random.Random(seed)
    pert = {k: rng.uniform(-v * scale, v * scale) for k, v in RANGES.items()}
    S.T_END = t_end
    sim = S.Sim(perturb=pert)
    sim.run(None)
    dpos, _ = p.getBasePositionAndOrientation(sim.desk)
    desk_move = ((dpos[0] - (S.DESK_X + pert["desk_dx"])) ** 2
                 + (dpos[1] - pert["desk_dy"]) ** 2) ** 0.5
    p.disconnect(sim.cid)
    ok = sim.grasped == S.STACK_N and desk_move < DESK_TOL
    if sim.aborted:
        mode = f"挿入中止・机は無傷（{sim.push_at_abort:.0f}mm で検知）"
    elif sim.grasped == 0:
        mode = (f"把持ゼロ（山を {sim.pushed_mm:.0f}mm 押した）" if sim.pushed_mm > 10
                else "把持ゼロ（櫛歯が届かなかった）")
    elif sim.grasped < S.STACK_N:
        mode = f"部分把持 {sim.grasped}/{S.STACK_N}（山が崩れた/ずれた）"
    elif desk_move >= DESK_TOL:
        mode = f"机が {desk_move * 1000:.0f}mm 動いた（反則）"
    else:
        mode = "成功"
    return ok, sim.grasped, desk_move, pert, mode, sim.pushed_mm, sim.aborted


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="ばらつきの倍率。頑健性の限界を探すのに使う")
    ap.add_argument("--sweep", action="store_true",
                    help="倍率 1/2/3/4 で掃引して破綻点を探す")
    args = ap.parse_args()

    if args.sweep:
        print("ばらつき倍率ごとの成功率（破綻点の探索）")
        for sc in (1.0, 2.0, 3.0, 4.0):
            res = [trial(2000 + j, scale=sc) for j in range(args.n)]
            push_max = max(r[5] for r in res)
            n_abort = sum(1 for r in res if r[6])
            desk_max = max(r[2] for r in res) * 1000
            ok_n = sum(1 for r in res if r[0])
            bad = {}
            for r in res:
                if not r[0]:
                    bad[r[4]] = bad.get(r[4], 0) + 1
            rng_txt = (f"山±{RANGES['stack_dx'] * sc * 1000:.0f}mm/±{RANGES['stack_yaw'] * sc:.0f}° "
                       f"停止±{RANGES['base_err'] * sc * 1000:.0f}mm")
            print(f"  ×{sc:.0f}  {rng_txt:34s} 成功 {ok_n}/{args.n} = {ok_n / args.n * 100:3.0f}%"
                  + f"  中止{n_abort}件 最大押し出し{push_max:4.0f}mm 机の移動最大{desk_max:4.1f}mm"
                  + ("" if not bad else "   " + ", ".join(f"{m}×{c}" for m, c in bad.items())))
        return 0

    rows = []
    for i in range(args.n):
        ok, got, dm, pert, mode, push, ab = trial(1000 + i, scale=args.scale)
        rows.append((i, ok, got, dm, pert, mode))
        if not args.md:
            print(f"  #{i:2d} {'OK ' if ok else 'NG '} 把持{got:2d}/10 机{dm * 1000:5.1f}mm "
                  f"山dx{pert['stack_dx'] * 1000:+5.1f} dy{pert['stack_dy'] * 1000:+5.1f} "
                  f"θ{pert['stack_yaw']:+4.1f}° 停止誤差{pert['base_err'] * 1000:+5.1f}mm  {mode}")

    n_ok = sum(1 for r in rows if r[1])
    modes: dict[str, int] = {}
    for r in rows:
        if not r[1]:
            modes[r[5]] = modes.get(r[5], 0) + 1

    if args.md:
        print(f"| 試行 | 判定 | 把持枚数 | 机の移動 [mm] | 山 dx/dy/θ | 停止誤差 [mm] |")
        print("|---|---|---|---|---|---|")
        for i, ok, got, dm, pert, mode in rows:
            print(f"| {i} | {'OK' if ok else 'NG'} | {got}/10 | {dm * 1000:.1f} | "
                  f"{pert['stack_dx'] * 1000:+.0f}/{pert['stack_dy'] * 1000:+.0f}/"
                  f"{pert['stack_yaw']:+.1f}° | {pert['base_err'] * 1000:+.1f} |")
        print()
        print(f"**成功率 {n_ok}/{args.n} = {n_ok / args.n * 100:.0f}%**"
              f"（山±20mm/±5°、机±10mm、停止±10mm のばらつき下）")
        if modes:
            print()
            print("| 失敗モード | 件数 |")
            print("|---|---|")
            for m, c in sorted(modes.items(), key=lambda kv: -kv[1]):
                print(f"| {m} | {c} |")
    else:
        print(f"\n  成功率 {n_ok}/{args.n} = {n_ok / args.n * 100:.0f}%")
        for m, c in sorted(modes.items(), key=lambda kv: -kv[1]):
            print(f"    失敗: {m} × {c}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
