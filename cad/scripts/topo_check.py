"""トポロジー最適化した輪郭の検査 — 「古い」と「作れない」を落とす.

    python scripts/topo_check.py [--md]

`scripts/topo_opt.py` が出す `out/topo/<板名>.json` は**生成物**で、
`src/` を直せば古くなる。古い輪郭のまま STEP を作ると、相手を動かしたのに
板の形だけ前のまま、という状態になる。ここで 3 つ見る:

  A. 鮮度 … 輪郭を作ったときの**境界条件の指紋**が、いまと一致するか
  B. 網羅 … `topo_plate()` を呼んでいる板の輪郭が全部あるか
  C. 作れるか … 分断していないか・最小部材幅・座面が残っているか

⚠ A を**ファイルの更新時刻**で見ていたが外した。`src/` を 1 文字直しただけで
  全部の輪郭が「古い」になり、本当に古くなったときに気づけなくなる
  （実際 `tr_lib.py` に sys.path を 3 行足しただけで 9 枚とも警告が出た）。
  形に関わるのは境界条件だけなので、その指紋（`topo_opt.bc_hash`）で比べる。

⚠ C は `topo_opt.py` を走らせたときにも出るが、**そのとき見て直した**とは
  限らない。輪郭は JSON として残るので、あとから何度でも問い直せるように
  ここでも見る。「1 回通った」を根拠に使わないための二重化。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

TOPO = os.path.join(ROOT, "out", "topo")


def used_names() -> set[str]:
    """`L.topo_plate("...")` で呼ばれている板の名前。"""
    import re
    names = set()
    for path in glob.glob(os.path.join(ROOT, "src", "*.py")):
        with open(path, encoding="utf-8") as fp:
            for m in re.finditer(r'topo_plate\(\s*["\']([\w]+)["\']', fp.read()):
                names.add(m.group(1))
    return names


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(TOPO, "*.json")))
    files = [f for f in files if not os.path.basename(f).startswith("_")]

    bad: list[tuple[str, str]] = []
    rows: list[tuple] = []

    if not files:
        bad.append(("—", "輪郭が 1 つも無い。`python scripts/topo_opt.py` を回すこと"))

    import topo_opt as TO
    from topo import core, shape

    regions = {r.name: r for r in TO.collect()[0]}

    for path in files:
        nm = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding="utf-8") as fp:
            d = json.load(fp)
        out = shape.from_json(d)
        reg = regions.get(nm)
        # ⚠ 鮮度は**ファイルの更新時刻では見ない**。`src/` を 1 文字直しただけで
        #   全部の輪郭が「古い」になり、本当に古くなったときに気づけなくなる
        #   （実際 `tr_lib.py` に 3 行足しただけで 9 枚とも警告が出た）。
        #   境界条件そのものの指紋で比べる。
        if reg is not None:
            now = TO.bc_hash(reg)
            if d.get("bc") != now:
                bad.append((nm, f"境界条件が輪郭を作ったときと違う（"
                                f"{d.get('bc', 'なし')} → {now}）。"
                                f"topo_cache → topo_opt をやり直すこと"))
        msgs = shape.check(out, reg) if reg is not None else ["境界条件が作れない板になった"]
        # ⚠ **素の板（`plain`）には最適化の物差しを当てない（2026-08-05）。**
        #   最適化で使える形が出なかった板は、`topo_opt` が「設計領域＋逃げ穴」
        #   をそのまま輪郭として書く（消すと `topo_plate` が例外で落ちる）。
        #   その形は**設計者が描いた板そのもの**なので、「最小部材幅が
        #   RIB_MIN 未満」「連結成分が 2 個」は最適化の失敗ではない。
        #   実測 `yaw_side_L` の素の板は最小幅 1.72mm だが、それはねじの
        #   逃げ穴と板の縁のあいだの残肉で、元の設計にもとからある。
        #   ⚠ **消しているのではない。**板の残肉そのものは「板の荷重経路の
        #     細り」（`ligament.py`）が別に見ている。ここで二重に出すと、
        #     最適化の失敗と設計の細りが混ざって読めなくなる。
        #   座面・逃げ穴・境界条件の鮮度は素の板でも見る（見落とすと
        #   「軸が通らない板」を書いたまま気づけない）。
        plain = bool(d.get("plain"))
        if plain:
            msgs = [m for m in msgs
                    if "最小部材幅" not in m and "連結成分" not in m]
        for m in msgs:
            bad.append((nm, m))
        rows.append((nm, out.area, out.min_width, out.n_parts, len(out.holes),
                     ("素の板（最適化できず）" if plain else "OK") if not msgs
                     else " / ".join(msgs)))

    for nm in sorted(used_names() - {os.path.splitext(os.path.basename(f))[0]
                                     for f in files}):
        bad.append((nm, "`topo_plate` で呼んでいるのに輪郭が無い"))

    if args.md:
        print("# トポロジー最適化した輪郭の検査\n")
        print("| 板 | 面積 mm² | 最小幅 mm | 連結 | 穴 | 判定 |")
        print("|---|---:|---:|---:|---:|---|")
        for nm, a, wmin, n, nh, verdict in rows:
            # ⚠ 「素の板」は不良ではない（最適化を**しなかった**だけで、
            #   板そのものは切れるし組める）。`NG` を付けると、判定を
            #   終了コードではなく本文の語で見ている検査に拾われる。
            mark = ("OK" if verdict == "OK"
                    else verdict if verdict.startswith("素の板")
                    else f"NG {verdict}")
            print(f"| {nm} | {a:,.0f} | {wmin:.1f} | {n} | {nh} | {mark} |")
        if bad:
            print(f"\n**{len(bad)} 件の指摘**\n")
            for nm, m in bad:
                print(f"- `{nm}` … {m}")
        else:
            print(f"\n輪郭 {len(rows)} 枚。指摘なし（最小部材幅の下限 "
                  f"{core.RIB_MIN:.0f}mm）")
    else:
        for nm, a, wmin, n, nh, verdict in rows:
            print(f"{nm:<18} {a:9,.0f}mm² 最小幅{wmin:5.1f} 連結{n} 穴{nh}  {verdict}")
        for nm, m in bad:
            print(f"  ⚠ {nm}: {m}")
    # ⚠ 合否は**致命だけ**で決める（`topo_opt.FATAL` と同じ見方）。
    #   座面の角が丸めで 1mm² 落ちるのと、板が 2 つに割れているのを
    #   同じ扱いにすると、次に直すべき板が選べない。
    return 1 if any(TO.fatal([m]) or "境界条件" in m for _n, m in bad) else 0


if __name__ == "__main__":
    raise SystemExit(main())
