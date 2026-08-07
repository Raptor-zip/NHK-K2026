"""凍結したねじが「頭の座面」と「ねじ込みの肉」を実体として持っているか.

    python scripts/screw_seat_check.py [--md]

なぜ要るか
-----------
`scripts/screw_place.py` は接触面の有効格子を頭の座面ぶん収縮（`_erode`）して
から置き場所を選ぶが、あの関数は**収縮しきったら 1 つ前を返す**。板厚 2mm の
端面のように「どう頑張っても座面が取れない面」でも、収縮前の縁がそのまま
候補として残る。実際、ニップ入口ガイド（PETG t2）の 3 分割の継ぎ目に
**t2 の断面へ M4×20 を 12mm ねじ込む** ねじが 8 本凍結されていた。

`edge_tap_check.py` は注記に「端面」「タップ」と書いてある締結しか見ない。
注記が `2-M4` のものは素通りするので、**宣言ではなく実体から**見る検査が要る。

見るもの
---------
  A. 頭の座面 … 座面から 0.5mm 中へ入った面で、頭の外周（頭半径の 85%）が
     頭を入れる側の材料の上に載っているか
  B. ねじ込みの肉 … かみ合い区間の 3 か所で、**下穴の外に 0.3d の壁**が
     残っているか（M4 なら半径 2 + 1.2 = 3.2mm の外周が材料の中）。
     `SCREW_IN` のみ。0.3d はタップの山が持つ最小の肉で、これを割ると
     締めたときに側面が裂ける。

どちらも円周 12 点で見て、**3/4 以上**を要求する。面取り・ザグリ・肉抜きの
縁で数点落ちるのは普通なので、そこは通す。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

N = 12                  # 円周の標本数
OK_RATIO = 0.75         # これを下回ったら不良


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    import export_fab as EF
    import screw_place as SP
    import tr_assembly as A
    import tr_fix as F
    import tr_lib as L
    import validate as V

    # ⚠ **ねじを置かない状態で組む。** 置いてから見ると、ねじ自身の実体が
    #   「座面の材料」として内点判定に引っかかる。
    A.AUTO_SCREWS = False
    # ⚠ **材質は `put()` の時点の色から取る。** `L.mat()` が材質ごとに色を
    #   付けているので色は事実上の材質タグだが、**組み上がった Solid では
    #   色が落ちている**（`.solids()` が畳むときに消える）。組立後に
    #   `sd.color` を見ていたときは材質が全部空になり、モーターのような
    #   買う部品を除外できていなかった（`plate_audit.build_parts` と同じ手）。
    rgb2mat = {tuple(round(c, 4) for c in v): k for k, v in L.MAT_COLOR.items()}
    mat_of: dict[str, str] = {}
    orig_put = A.put

    def rec_put(parts, shape_, name, to=None, how=None, note="", tool=None):
        col = getattr(shape_, "color", None)
        if col is None:
            for ch in getattr(shape_, "children", []) or []:
                col = getattr(ch, "color", None)
                if col is not None:
                    break
        if col is not None:
            m = rgb2mat.get(tuple(round(float(x), 4) for x in tuple(col)[:3]))
            if m is not None:
                mat_of[name] = m
        return orig_put(parts, shape_, name, to=to, how=how, note=note,
                        tool=tool)

    A.put = rec_put
    try:
        shape = A.build(dict(yaw=0.0, pitch=0.0, grab=0.0, press=0.0, tilt=0.0))
    finally:
        A.put = orig_put
    org = {k: tuple(v.position) for k, v in A.LINK_LOC.items()}

    known = set(F.BOX)
    sol: dict[str, list] = {}
    bb: dict[str, list] = {}
    for n, sd, _b in V.solids_with_bbox(shape):
        seg = [q.split("#")[0] for q in n.split("/")]
        nm = next((q for q in reversed(seg) if q in known), seg[-1])
        sol.setdefault(nm, []).append(sd)
        o = bb.get(nm)
        v = [_b.min.X, _b.min.Y, _b.min.Z, _b.max.X, _b.max.Y, _b.max.Z]
        bb[nm] = v if o is None else [min(o[i], v[i]) for i in range(3)] + \
            [max(o[i], v[i]) for i in range(3, 6)]

    def thick(nm: str) -> float:
        """外接箱の最小辺 ≒ 板厚。**必要な厚みとの差を出すために添える。**"""
        v = bb.get(nm)
        return min(v[3] - v[0], v[4] - v[1], v[5] - v[2]) if v else 0.0

    def is_bought(nm: str) -> bool:
        """買う部品か。**買う部品の穴は規格で決まっている。**

        ⚠ モーターや軸受やリングは、CAD では円柱や輪っかで描いてある。
          ねじ穴の周りにどれだけ肉があるかは、その簡略形からは読めない
          （実物にはちゃんとボスがある）。ここを見ると偽の指摘しか出ない。
        """
        return EF.classify(nm, mat_of.get(nm, ""), sol.get(nm, []))[0] == "BUY"
    # ⚠ 判定は `screw_place` と同じ実装を使う（GPU があれば GPU）。
    #   ここで別の判定を書くと、「置けた」と「座面がある」で答えが割れる。
    ins = {k: SP.inside_of(v) for k, v in sol.items()}

    with open(os.path.join(ROOT, "out", "screws.json"), encoding="utf-8") as fp:
        data = json.load(fp)

    def head_radius(kind: str, d: float) -> float:
        if kind == "RIVET":
            return max(2.0, d) / 2
        if kind == "RIVET_FLAT":
            return L.rivet_flat_head(d) / 2
        if kind == "FLAT":
            return L.FLAT_HEAD_DIA[int(d)] / 2
        return L.SCREW_HEAD[int(d)][0] / 2

    bad_seat, bad_tap = [], []
    for s in data["screws"]:
        a, b, d = s["a"], s["b"], s["size"]
        if a not in ins or b not in ins:
            continue                       # 実体の無い組は screw_check の担当
        hr = head_radius(s["kind"], d)
        # 凍結位置はリンクローカル。関節 0 ではどのリンクも並進だけなので足すだけ。
        o = org.get(s["link"], (0.0, 0.0, 0.0))
        p = [s["pos"][i] + o[i] for i in range(3)]
        dz, ux, uy = SP._basis(s["dir"])

        ang = [2 * math.pi * j / N for j in range(N)]

        # A. 頭の座面（`screw_place` は頭を入れる側を必ず a に入れて凍結する）
        c = [p[i] - dz[i] * 0.5 for i in range(3)]
        ring = [[c[i] + hr * 0.85 * (math.cos(t) * ux[i] + math.sin(t) * uy[i])
                 for i in range(3)] for t in ang]
        hit = int(np.asarray(ins[a].many(ring)).sum())
        if hit < N * OK_RATIO and not is_bought(a):
            bad_seat.append((s["name"], a, b, d, hit, N))

        # B. ねじ込みの肉。
        # ⚠ **壁は「呼び径 + 少し」では足りない。** 下穴の外に残る肉が
        #   0.3d 無いと、タップの山が持たずに側面が裂ける（M4 で 1.2mm）。
        #   0.6mm 固定で見ていた頃は、t6 の耳が「合格」になっていた
        #   （壁 0.75mm）。`hop_hanger_*` や `mount_ear_*` を t8 にしてある
        #   のと同じ基準に揃える。
        if s["how_ab"][1] == "SCREW_IN" and not is_bought(b):
            grip = s.get("grip", 0.0)
            eng = max(1.0, min(s["length"] - grip, 2.0 * d))
            wall = d / 2 + max(0.6, 0.3 * d)
            pts = []
            for step in (0.3, 0.6, 0.9):
                cc = [p[i] - dz[i] * (grip + eng * step) for i in range(3)]
                pts += [[cc[i] + wall * (math.cos(t) * ux[i]
                                         + math.sin(t) * uy[i])
                         for i in range(3)] for t in ang]
            hit2 = int(np.asarray(ins[b].many(pts)).sum())
            tot = len(pts)
            if hit2 < tot * OK_RATIO:
                bad_tap.append((s["name"], a, b, d, hit2, tot,
                                thick(b), 2 * wall))

    if args.md:
        out = ["凍結したねじの座面とねじ込みの肉を、実体の内点判定で見る。",
               f"円周 {N} 点のうち {OK_RATIO:.0%} 以上が材料であることを要求する。",
               "", f"- ねじ {len(data['screws'])} 本",
               f"- 座面が足りない {len(bad_seat)} 本",
               f"- ねじ込みの肉が足りない {len(bad_tap)} 本", ""]
        if bad_seat:
            out += [f"### 頭の座面が足りない {len(bad_seat)} 本", "",
                    "| ねじ | 径 | 座面の部品 | 相手 | 外周 |", "|---|---|---|---|---|"]
            for nm, a, b, d, h, n in bad_seat:
                out.append(f"| `{nm}` | M{d:g} | `{a}` | `{b}` | {h}/{n} |")
            out.append("")
        if bad_tap:
            out += [f"### ねじ込み側の肉が足りない {len(bad_tap)} 本", "",
                    "下穴の外に **0.3d** の壁が要る（M4 なら 1.2mm）。"
                    "「要 t」は端面へ立てる場合に最低限いる厚み。", "",
                    "| ねじ | 径 | 頭側 | ねじ込む先 | 周 | いまの t | 要 t |",
                    "|---|---|---|---|---|---|---|"]
            for nm, a, b, d, h, n, tk, need in bad_tap:
                out.append(f"| `{nm}` | M{d:g} | `{a}` | `{b}` | {h}/{n} | "
                           f"{tk:.1f} | {need:.1f} |")
            out.append("")
        if not bad_seat and not bad_tap:
            out.append("座面もねじ込みの肉も足りている。")
        # ⚠ **stdout に全文を出す。** `check_all` は stdout をそのまま
        #   `out/screw_seat.md` に書くので、ここで端折ると表が切れる。
        with open(os.path.join(ROOT, "out", "screw_seat.md"), "w",
                  encoding="utf-8") as fp:
            fp.write("\n".join(out) + "\n")
        print("\n".join(out))
    else:
        for nm, a, b, d, h, n in bad_seat:
            print(f"座面 NG {nm} M{d:g} {a} → {b} 外周 {h}/{n}")
        for nm, a, b, d, h, n, tk, need in bad_tap:
            print(f"肉   NG {nm} M{d:g} {a} → {b} 周 {h}/{n} "
                  f"（t{tk:.1f} → 要 t{need:.1f}）")
        print(f"ねじ {len(data['screws'])} 本 / 座面 {len(bad_seat)} 本 / "
              f"肉 {len(bad_tap)} 本")
    # ⚠ 指摘があれば 1 を返す（`check_all` は終了コードで合否を決める）。
    return 1 if (bad_seat or bad_tap) else 0


if __name__ == "__main__":
    raise SystemExit(main())
