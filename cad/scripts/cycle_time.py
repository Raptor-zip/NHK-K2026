"""射出サイクルタイムの計算（戦略書 §1.3「サイクル3.0秒以下」の検証）.

    python scripts/cycle_time.py [--md]

`tr_params.py` のアクチュエータ諸元と搬送距離だけから、1枚あたりの
射出サイクルを積み上げる。FAQ 4.1 Q14/Q15（1枚が機構を離れてから次を発射）を
守った上で、斜路にプリステージできる分だけ並列化する。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

MM = 0.001

FEED_SPEED = 0.30            # m/s ピックローラー / 搬送ベルトの送り速度（M2006 で余裕）
MUZZLE_V = 6.5               # m/s 常用初速の上限（布の揚力補正込み・scripts/target_ev.py）
ROLLER_RECOVER = 0.20        # s 射出でローラーが食われた回転数を戻す時間
AIM_SETTLE = 0.20            # s 砲塔ヨー微調整の整定
CONTROL_MARGIN = 0.30        # s 重送検知・判定・人間のコールぶんの余裕
STAGE_TO_NIP = 0.100         # m 斜路のプリステージ位置からニップまで


def ramp_len() -> float:
    return math.hypot(P.FEED_RAMP_X1 - P.FEED_RAMP_X0, P.FEED_RAMP_Z1 - P.FEED_RAMP_Z0) * MM


def rows():
    rag_x = P.RAG_X * MM
    out = [
        ("① 分離（ピックローラーが1枚を送り出す）", rag_x / FEED_SPEED, "並列可"),
        ("② 斜路搬送（ホッパー→プリステージ）", (ramp_len() - STAGE_TO_NIP) / FEED_SPEED, "並列可"),
        ("③ プリステージ→ニップ投入", STAGE_TO_NIP / FEED_SPEED, "直列"),
        ("④ ニップ通過（雑巾長 ÷ ローラー周速）", rag_x / MUZZLE_V, "直列"),
        ("⑤ ローラー回転数の回復", ROLLER_RECOVER, "直列"),
        ("⑥ 砲塔ヨー再照準の整定", AIM_SETTLE, "直列"),
        ("⑦ 重送検知・判定の余裕", CONTROL_MARGIN, "直列"),
    ]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    r = rows()
    serial = sum(t for _, t, kind in r if kind == "直列")
    parallel = sum(t for _, t, kind in r if kind == "並列可")
    cycle = max(serial, parallel)          # 並列側が律速になることもある
    supply = 180.0 / 32.0                  # 供給レート上限 5.6 s/枚（戦略書 §1.3）

    if args.md:
        print("| 工程 | 所要 [s] | 直列/並列 |")
        print("|---|---|---|")
        for name, t, kind in r:
            print(f"| {name} | {t:.2f} | {kind} |")
        print(f"| **直列の合計（＝サイクル）** | **{serial:.2f}** | |")
        print(f"| 並列側の合計（次弾の準備） | {parallel:.2f} | 直列側に隠れる |")
        print()
        print(f"- **サイクル {cycle:.2f} s/枚** ≤ 戦略書の目標 3.0 s "
              f"{'✅' if cycle <= 3.0 else '❌'}")
        print(f"- 供給レート上限は {supply:.1f} s/枚（32枚/180秒）なので、"
              f"射出機構は律速ではない。律速は補充往復と走行。")
        print(f"- 3.0秒の目標に対し {3.0 - cycle:.2f} s の余裕がある。"
              "この余裕は「照準の再確認」と「重送時の1回リトライ」に充てる。")
    else:
        for name, t, kind in r:
            print(f"  {name:<40} {t:5.2f} s  ({kind})")
        print(f"\n  直列合計（サイクル） {serial:.2f} s/枚   並列側 {parallel:.2f} s")
        print(f"  目標 3.0 s → {'OK' if cycle <= 3.0 else 'NG'}   "
              f"供給レート上限 {supply:.1f} s/枚")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
