"""自動配置したねじの検査 — 「古い」と「足りない」を落とす.

    python scripts/screw_check.py [--md]

`scripts/screw_place.py` が出す `out/screws.json` は**生成物**で、部品を
動かせば古くなる。古い位置のままねじを置くと、相手は動いたのにねじだけ
前の場所に残る（それでも組立検査は「浮き」ではなく「離れ」として出るので、
原因がねじの凍結だと気づきにくい）。ここで 3 つ見る:

  A. 鮮度 … 位置を決めたときの**部品の bbox の指紋**が、いまと一致するか
  B. 網羅 … 宣言のうち実体が入った割合。減っていたら気づけるようにする
  C. 整合 … JSON の各行が、いまも実在する部品を指しているか

⚠ A は `topo_check.py` と同じ考え方（ファイルの更新時刻では見ない）。
  `src/` を 1 文字直しただけで「古い」になると、本当に古くなったときに
  気づけなくなる。位置が依存しているのは**部品の bbox だけ**なので、
  その指紋で比べる。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

JSON = os.path.join(ROOT, "out", "screws.json")

# 実体の入った締結の組が、これを下回ったら警告する。
# ⚠ 数字そのものより「**減った**」ことに意味がある。判定を厳しくするたびに
#   置ける本数は減るので、下げるときは理由を書いてここも下げること。
# 2026-08-07: 実体の入った組が 278 → 339 に増えたので、下限も上げる
# （下げるときは理由を書くこと、というのがこの定数の約束）。
MIN_JOINTS = 320


def _slot_screws(d, F, L):
    """溝ナットで留まるねじを (a, b, 位置, 向き, ねじ名) で並べる。

    ⚠ **自動配置と手置きの両方から拾う。** `out/screws.json` だけを見ていると、
      手で位置を決めたねじ（斜材のボルトのように、自動では芯が出せなくて
      組立側に移したもの）がまるごと検査の外に出る。移した先が見られて
      いないのでは、移した意味が無い。
    """
    for r in d["screws"]:
        if r.get("kind") != "RIVET":
            yield r["a"], r["b"], r["pos"], r["dir"], r["name"]
    auto = {r["name"] for r in d["screws"]}
    for nm, (_kind, _size, _ln, extras) in F.FASTENER.items():
        if nm in auto or not any(e[0] == "TNUT" for e in extras):
            continue
        tool = F.TOOL.get(nm)               # (頭 x,y,z, 工具を差す向き)
        if tool is None:
            continue
        tg = [t for t in F.targets_of(nm) if not L.is_screw_part(t)]
        for i, b in enumerate(tg):
            yield (tg[1 - i] if len(tg) == 2 else nm), b, \
                tuple(tool[:3]), tuple(tool[3:]), nm


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    import screw_place as SP
    import tr_assembly as A
    import tr_lib as L
    import tr_params as P

    bad: list[str] = []
    try:
        with open(JSON, encoding="utf-8") as fp:
            d = json.load(fp)
    except FileNotFoundError:
        print("ねじの位置が 1 つも無い。`python scripts/screw_place.py` を回すこと")
        return 1

    A.build(P.POSE_MATCH)
    joints, placed, missing, stray, _sm = L.fastener_joints()
    n_missing = sum(r[4] for r in missing)

    # A. 鮮度。⚠ 指紋はねじを**置かない**状態の bbox で取る（置いた状態で
    #    取ると、ねじ自身が指紋に入って毎回変わる）。
    A.AUTO_SCREWS = False
    A._STATIC = None                       # キャッシュを捨てて組み直す
    # ⚠ ソリッド数の見張りも解除する。ねじを外して組み直すのだから数は
    #   減って当たり前で、そのまま呼ぶと「木を壊した」と誤検出になる。
    A._BUILD_SOLIDS = None
    A.build(dict(yaw=0.0, pitch=0.0, grab=0.0, press=0.0, tilt=0.0))
    now = SP.bc_hash()
    A.AUTO_SCREWS = True
    A._STATIC = None
    A._BUILD_SOLIDS = None
    if d.get("bc") != now:
        bad.append(f"部品の位置が凍結したときと違う（{d.get('bc', 'なし')} → {now}）。"
                   f"`python scripts/screw_place.py` をやり直すこと")

    # B. 網羅
    if len(placed) < MIN_JOINTS:
        bad.append(f"実体の入った締結が {len(placed)} 組しかない"
                   f"（{MIN_JOINTS} 組は入るはず）")
    if stray:
        bad.append(f"宣言のどの組にも紐づかないねじ {len(stray)} 本")

    # C. 整合
    import tr_fix as F
    for r in d["screws"]:
        for nm in (r["a"], r["b"]):
            if nm not in F.BOX:
                bad.append(f"{r['name']} が指す {nm} は組立に無い")
                break

    # D. 溝の上に乗っているか
    # ⚠ **アルミフレームは面のどこにでも留められない。** HFS5-2020 は
    #   4 面それぞれに溝が 1 条だけ、面の中心線に通っている。そこから
    #   外れた場所は下穴の無いただの肌で、溝ナットは入らない。
    #   置き場所を「両方に材料がある所」だけで選んでいた頃は、押出材に
    #   留まる 84 本が**全部**溝から外れていた（最大 6mm ＝ 面の端）。
    #   リベットは溝を使わない（面に下穴を開ける）ので数えない。
    off = []
    for r in d["screws"]:
        b = r["b"]
        if (not L.is_extrusion(b) or r.get("link") != "base"
                or r.get("kind") == "RIVET" or b not in F.BOX):
            continue
        bb = F.BOX[b]
        ext = [bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z]
        la = max(range(3), key=lambda i: ext[i])
        if max(ext[i] for i in range(3) if i != la) > P.EXT_W + 1.5:
            continue                       # 斜材は下の D2 で見る（bbox では出せない）
        k = next((i for i, v in enumerate(r["dir"]) if v), None)
        if k is None or k == la:
            continue                       # 端面は中心の下穴なので別扱い
        w = next(i for i in range(3) if i not in (k, la))
        ctr = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2,
               (bb.min.Z + bb.max.Z) / 2)[w]
        if abs(r["pos"][w] - ctr) > 1.0:
            off.append((abs(r["pos"][w] - ctr), r["name"], r["a"], b))

    # D2. **斜めに寝た押出材**の溝の芯。
    # ⚠ D は bbox の中心線を溝の芯とみなすので、傾いた部材には使えない
    #   （bbox が断面より太い）。そこを「判定できない」として素通しにして
    #   いたぶん、斜材に留まる 4 本は軸線から 3.0〜6.1mm 外れたまま残り、
    #   1 本はガセットの外に浮いていた。芯は組立側が `tr_fix.SLOT_AXIS` に
    #   出しているので、そこからの**面内の横ずれ**で見る。
    # ⚠ 自動配置（screws.json）だけでなく、**手で置いたねじも見る**。
    #   斜材のボルトは手で置くようにしたので、自動配置しか見ていないと
    #   この検査は 0 本を見て「指摘なし」と言う。
    for a, b, pos, dvec, nm in _slot_screws(d, F, L):
        if b not in F.SLOT_AXIS:
            continue
        px, py, pz, ux, uy, uz = F.SLOT_AXIS[b]
        # 面内の横方向 = 軸 × ねじの向き。ここへの射影が溝からのずれ
        wx = uy * dvec[2] - uz * dvec[1]
        wy = uz * dvec[0] - ux * dvec[2]
        wz = ux * dvec[1] - uy * dvec[0]
        n = (wx * wx + wy * wy + wz * wz) ** 0.5
        if n < 1e-6:
            continue                       # ねじが軸と平行＝端面。溝の話ではない
        e = abs(((pos[0] - px) * wx + (pos[1] - py) * wy
                 + (pos[2] - pz) * wz) / n)
        if e > 1.0:
            off.append((e, nm, a, b))
    if off:
        off.sort(reverse=True)
        bad.append(f"溝の中心から外れた溝ナット締結が {len(off)} 本"
                   f"（最大 {off[0][0]:.1f}mm・{off[0][1]} {off[0][2]}→{off[0][3]}）")

    if args.md:
        print("# 自動配置したねじの検査\n")
        print(f"- 凍結した本数 {len(d['screws'])}")
        print(f"- 締結の宣言 {len(joints)} 組 / 実体が入った {len(placed)} 組")
        print(f"- 実体の無い締結 {n_missing} 本（{len(missing)} 組）")
        if bad:
            print(f"\n**{len(bad)} 件の指摘**\n")
            for m in bad:
                print(f"- {m}")
        else:
            print("\n指摘なし")
    else:
        print(f"凍結 {len(d['screws'])} 本 / 宣言 {len(joints)} 組 / "
              f"実体あり {len(placed)} 組 / 未実体 {n_missing} 本")
        for m in bad:
            print(f"  ⚠ {m}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
