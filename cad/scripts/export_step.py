"""3 コンフィグの STEP を書き出す（機体まるごと 1 ファイル）。

    .venv/bin/python scripts/export_step.py            # 3 つとも
    .venv/bin/python scripts/export_step.py --only match

⚠ **部品ごとの STEP は `export_fab.py` の仕事**（`out/fab/step/`）。ここで
  出すのは「組んだ状態を人が見る / 外部 CAD へ渡す」ためのファイルで、
  `ASSEMBLY.md` が参照しているのもこちら（`out/tr_match.step`）。

⚠ ラベル付きの Compound をそのまま `export_step` に渡すと、build123d が
  アセンブリ文書の経路に入って**中身が空の STEP** を書くことがある
  （`export_fab.py` で実測。部品ごとの書き出しが 9 個そうなった）。
  ここでは形だけを集めた素の Compound を作り直してから渡す。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from build123d import Compound, Solid, export_step  # noqa: E402

import tr_assembly as A  # noqa: E402
import tr_params as P  # noqa: E402

CONFIGS = {
    "match": ("競技中（ヨー0・仰角既定）", lambda: P.POSE_MATCH),
    "stowed": ("スタート時（1000×1000×1200 に収める姿勢）",
               lambda: P.POSE_STOWED),
    "loading": ("装填中", lambda: P.POSE_LOADING),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None, help="match / stowed / loading")
    ap.add_argument("--out", default=os.path.join(ROOT, "out"))
    args = ap.parse_args()

    names = [args.only] if args.only else list(CONFIGS)
    os.makedirs(args.out, exist_ok=True)
    rc = 0
    for nm in names:
        if nm not in CONFIGS:
            print(f"知らないコンフィグ: {nm}（{'/'.join(CONFIGS)}）")
            return 2
        label, pose = CONFIGS[nm]
        t0 = time.time()
        comp = A.build(pose())
        # ⚠ ラベルと親子を落とす。付いたままだとアセンブリ経路に入る。
        flat = Compound(children=[Solid(s.wrapped) for s in comp.solids()])
        path = os.path.join(args.out, f"tr_{nm}.step")
        export_step(flat, path)
        mb = os.path.getsize(path) / 1e6
        n = len(comp.solids())
        print(f"{nm:8s} {label:34s} ソリッド {n:4d}  {mb:6.1f}MB  "
              f"{time.time() - t0:5.1f}s  → {os.path.relpath(path, ROOT)}")
        if mb < 1.0:
            print(f"  ⚠ {mb:.2f}MB は小さすぎる。中身が空の可能性がある")
            rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
