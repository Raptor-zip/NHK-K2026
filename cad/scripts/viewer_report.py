#!/usr/bin/env python3
"""ビューアが「干渉」と表示するものの正体を、組ごとに数えて出す。

⚠ **多くの CAD ビューアは「面が接している」ことを干渉として表示する。**
  ボルトで留めた面は接しているのが正しいので、正しい組立ほど「干渉」の
  件数が増える。ここを区別できないと「めっちゃ干渉してる」という見え方に
  なる（実際そうなっていた）。

  この表は 3 つを分けて数える:
    ① 実体が重なっている      … 宣言と許容で説明できるか / できないか
    ② 面が接している（0mm）   … 締結の宣言があるか / 無いか
    ③ 近い（0〜0.5mm）        … 組立公差の範囲
  ①の「説明できない」と②の「宣言が無い」だけが不良。
"""
from __future__ import annotations

import itertools
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
    pose = sys.argv[1] if len(sys.argv) > 1 else "match"
    sol = V.solids_with_bbox(A.build(AC.ALL_POSES[pose]()))
    names = [AC.part_name(n) for n, _s, _b in sol]
    cnt = {k: [0, 0.0] for k in ("lap_ok", "lap_bad", "touch_ok", "touch_bad",
                                 "near")}
    bad_rows = []
    for i, j in itertools.combinations(range(len(sol)), 2):
        na, nb = names[i], names[j]
        if na == nb:
            continue
        _p, sa, ba = sol[i]
        _q, sb, bb = sol[j]
        dec = F.declared(na, nb)
        if AC.bbox_overlap(ba, bb) > AC.BOOL_MIN:
            v = AC.overlap_volume(sa, sb)
            if v > 1.0:
                ok = dec[1] if dec else 0.0
                k = "lap_ok" if v <= ok else "lap_bad"
                cnt[k][0] += 1
                cnt[k][1] += v
                if k == "lap_bad":
                    bad_rows.append((v, na, nb, dec[0] if dec else "宣言なし"))
                continue
        elif AC.bbox_gap(ba, bb) > AC.CONTACT_TOL:
            continue
        try:
            d = sa.distance_to(sb)
        except Exception:                      # noqa: BLE001
            continue
        if d > AC.CONTACT_TOL:
            continue
        if d <= 1e-6:
            k = "touch_ok" if dec else "touch_bad"
        else:
            k = "near"
        cnt[k][0] += 1
        if k == "touch_bad":
            bad_rows.append((0.0, na, nb, "接触の宣言が無い"))

    print(f"[{pose}] ソリッド {len(sol)}\n")
    print("■ ビューアが「干渉」に見せるもの")
    print(f"  ② 面が接している（0mm）・締結の宣言あり  {cnt['touch_ok'][0]:4d} 組"
          "  ← **正しい組立。ボルト面・圧入・軸受**")
    print(f"  ③ 0〜0.5mm 離れている                  {cnt['near'][0]:4d} 組"
          "  ← 組立公差の範囲")
    print(f"  ① 実体が重なっていて宣言で説明できる    {cnt['lap_ok'][0]:4d} 組"
          f" 合計 {cnt['lap_ok'][1]:,.0f}mm³")
    print("       ↑ ねじがタップ穴に噛む体積・歯の噛み合い・束ねた配線")
    print("\n■ 不良（これだけが直す対象）")
    print(f"  ① 説明できない重なり  {cnt['lap_bad'][0]:4d} 組"
          f" 合計 {cnt['lap_bad'][1]:,.0f}mm³")
    print(f"  ② 宣言の無い接触      {cnt['touch_bad'][0]:4d} 組")
    for v, na, nb, how in sorted(bad_rows, reverse=True)[:20]:
        print(f"      {v:9,.0f}mm³  {na} ↔ {nb}  [{how}]")
    return 1 if (cnt["lap_bad"][0] or cnt["touch_bad"][0]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
