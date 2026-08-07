#!/usr/bin/env python3
"""URDF が CAD と一致しているかを確かめる（生成し直して突き合わせる）。

⚠ **tr.urdf は「生成物」なのに、生成できない状態で放置されていた。**
  URDF 生成器はリンクごとに link_*() を直接呼ぶので、左右の車輪・上下の
  ローラーで同じ部品を 2 回 put する。組立側に同名 put の検査を入れた
  ときからここが例外で落ち、**CAD を変えても tr.urdf が更新されなかった**。
  そのあいだに溜まっていた食い違い:
      grabber_slide の上限   0.4246 → 0.316   （ストロークを変えたのに）
      grabber_press の上限   0.07   → 0.105
      turret_yaw の原点 z    0.838  → 0.842
      fork_tilt の原点 z     0.763  → 0.771
      質量合計              34.831 → 34.852 kg
  制御はこの URDF を見るので、ここが古いと**動かしてから気づく**類の
  食い違いになる。生成物は「生成し直して差が無いこと」を検査する。
"""
from __future__ import annotations

import difflib
import sys
from pathlib import Path

_R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_R / "src"))
sys.path.insert(0, str(_R / "urdf"))

import tr_urdf  # noqa: E402
from xml.etree import ElementTree as ET  # noqa: E402


def _normalize(x: str) -> list[str]:
    """タグごとに 1 行へ割る（差分を読める形にする）。"""
    return [ln for ln in x.replace("><", ">\n<").splitlines() if ln.strip()]


def main() -> int:
    path = _R / "urdf" / "tr.urdf"
    robot = tr_urdf.gen_urdf()
    order = {"material": 0, "link": 1, "joint": 2}
    ch = sorted(list(robot), key=lambda e: order.get(e.tag, 3))
    for e in list(robot):
        robot.remove(e)
    robot.extend(ch)
    fresh = ET.tostring(robot, encoding="unicode")
    have = path.read_text(encoding="utf-8")
    # ヘッダ行（宣言とコメント）を落として本体だけ比べる
    body = have.split("\n", 2)[-1].strip()
    a, b = _normalize(body), _normalize(fresh)
    if a == b:
        print(f"URDF は CAD と一致（link {len(robot.findall('link'))} / "
              f"joint {len(robot.findall('joint'))}）")
        return 0
    d = [ln for ln in difflib.unified_diff(a, b, "tr.urdf", "CAD から生成",
                                           n=0, lineterm="")]
    print(f"### URDF が CAD と食い違っている（差分 {len(d) - 2} 行）")
    for ln in d[:40]:
        print("  " + ln)
    if len(d) > 40:
        print(f"  （他 {len(d) - 40} 行）")
    print("\n  → `python urdf/tr_urdf.py` で生成し直すこと")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
