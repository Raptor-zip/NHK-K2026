#!/usr/bin/env python3
"""姿勢を変えても**部品の構成が変わらない**ことを、組み直して確かめる。

⚠ 連続掃引（sweep_fine.py）は「1 回組んだソリッドを変換すれば任意の中間姿勢に
  なる」ことを前提にしている。この前提が崩れるのは、姿勢によって**形が変わる**
  部品があるとき（ブーリアンが空を返して部品が消える、姿勢を引数に取る形状、
  など）。そこだけは組み直さないと分からない。

  組立の厳格チェック（assembly_check.py）を全姿勢で回せば分かるが、
  1 姿勢に O(N²) のブーリアンが要るので 17 姿勢で 50 分を超える（実測）。
  ここは**ブーリアンを一切走らせず**、組み直して
    ・ソリッド数と部品名の集合
    ・固定宣言の集合
  だけ突き合わせる。
"""
from __future__ import annotations

import sys
from pathlib import Path

_R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_R / "src"))
sys.path.insert(0, str(_R / "scripts"))

import assembly_check as AC  # noqa: E402
import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import validate as V  # noqa: E402


def main() -> int:
    poses = list(AC.ALL_POSES)
    base = None
    bad = []
    for nm in poses:
        sol = V.solids_with_bbox(A.build(AC.ALL_POSES[nm]()))
        parts = frozenset(AC.part_name(p) for p, _s, _b in sol)
        decl = frozenset((a, t, h) for a, lst in F.FIXINGS.items()
                         for t, h, _q, _n in lst)
        cur = (len(sol), parts, decl)
        line = (f"  {nm:12s} ソリッド {len(sol)} / 部品 {len(parts)}"
                f" / 宣言 {len(decl)}")
        if base is None:
            base = (nm, cur)
            print(line + "  ← 基準")
            continue
        b = base[1]
        d_sol = cur[0] - b[0]
        d_part = (parts - b[1], b[1] - parts)
        d_dec = (decl - b[2], b[2] - decl)
        ok = (d_sol == 0 and not d_part[0] and not d_part[1]
              and not d_dec[0] and not d_dec[1])
        print(line + ("" if ok else "  ← 基準と違う"))
        if not ok:
            bad.append((nm, d_sol, d_part, d_dec))
    print()
    if bad:
        print(f"### 姿勢で構成が変わる {len(bad)} 件（{base[0]} を基準に）")
        for nm, d_sol, d_part, d_dec in bad:
            print(f"  {nm}: ソリッド差 {d_sol:+d}")
            if d_part[0]:
                print(f"      増えた部品 {sorted(d_part[0])[:8]}")
            if d_part[1]:
                print(f"      消えた部品 {sorted(d_part[1])[:8]}")
            if d_dec[0] or d_dec[1]:
                print(f"      宣言の差 +{len(d_dec[0])} / -{len(d_dec[1])}")
        print("\n  → 連続掃引の前提（形は姿勢で変わらない）が崩れている。"
              "その部品は組み直しで見るしかない")
        return 1
    print(f"全 {len(poses)} 姿勢で構成は同一"
          "（連続掃引が変換だけで足りる根拠）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
