"""組立から「板の幾何 + 固定宣言」だけ抜き出して JSON に落とす.

    python scripts/topo_cache.py            # out/topo/_parts.json を作り直す

なぜキャッシュするか
--------------------
`plate_audit.build_parts()` は組立を丸ごと作るので 2 分近くかかる。
境界条件の作り方（どれを固定と見るか、荷重をどう向けるか）は**何度も
試して直す**ところなので、そのたびに組立を作っていると 1 回の試行が 2 分になる。

⚠ このキャッシュは**組立を変えたら必ず作り直す**こと。古いキャッシュで
  最適化すると、動かした相手の位置に境界条件だけ取り残される。
  `topo_opt.py` はキャッシュが CAD より古ければ警告を出す。
"""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

PATH = os.path.join(ROOT, "out", "topo", "_parts.json")


def outline_of(solids, box, k: int, tol: float = 0.6):
    """板ソリッドを厚み軸 k に垂直な平面へ投影した**外形**（穴は埋める）。

    返り値は板ローカル座標（外接矩形の中心が原点）の (x, y) 列。取れなければ None。

    ⚠ **設計領域を外接矩形にすると、元が長方形でない板では最適化が
      改悪になる。** ガセット（直角三角形）は矩形の半分しか肉が無いので、
      矩形全体を設計領域にすると 62g → 105g と重くなった。元の外形の
      内側でしか材料を置けないようにすれば、少なくとも重くはならない。
    ⚠ 穴は**埋める**。肉抜き穴は作り直す対象なので、設計領域から除いて
      しまうと「前の肉抜きの位置」がそのまま残る。機能穴（軸の逃げ）は
      別途 `void` で宣言から作るので、ここで拾う必要はない。
    """
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    ax = [i for i in range(3) if i != k]
    ctr = [(box[2 * i] + box[2 * i + 1]) / 2 for i in ax]
    tris = []
    for sol in solids:
        try:
            verts, faces = sol.tessellate(tol)
        except Exception:
            return None
        for a, b, c in faces:
            p = [(verts[i].X, verts[i].Y, verts[i].Z) for i in (a, b, c)]
            q = [(v[ax[0]] - ctr[0], v[ax[1]] - ctr[1]) for v in p]
            # 厚み方向から見て潰れている三角形（側面）は投影に寄与しない
            a2 = abs((q[1][0] - q[0][0]) * (q[2][1] - q[0][1])
                     - (q[2][0] - q[0][0]) * (q[1][1] - q[0][1])) / 2
            if a2 > 1e-6:
                tris.append(Polygon(q))
    if not tris:
        return None
    u = unary_union(tris)
    if u.geom_type == "MultiPolygon":
        u = max(u.geoms, key=lambda g: g.area)
    # 外形だけ取る（穴は埋める）。⚠ そのあと簡略化しないと、面の分割ぶん
    #   だけ点が並んで JSON が数百 KB になる。
    ring = Polygon(u.exterior).simplify(0.3, preserve_topology=True)
    return [(round(x, 2), round(y, 2)) for x, y in ring.exterior.coords]


def build(pose: str = "flat") -> dict:
    import plate_audit as PA
    import assembly_check as AC
    import validate as V
    info, F, _u, _c = PA.build_parts(pose)
    sub = PA.subtree_mass(info, F)

    # 板候補のソリッドを集め直して、平面投影の外形を取る。
    # ⚠ `build_parts` は bbox しか残さないので、組立をもう一度作ることになる。
    #   ここだけのために 2 分かける価値はある（設計領域を間違えると、
    #   最適化の結果がまるごと使えない）。
    import tr_assembly as A
    shape = A.build(PA.POSES[pose])
    by_name: dict[str, list] = {}
    for path, sol, _bx in V.solids_with_bbox(shape):
        by_name.setdefault(AC.part_name(path), []).append(sol)

    # ⚠ **最適化した板は、測り直すと基準そのものが動く。**
    #   `L.topo_plate` に置き換えた板をここで測ると
    #     ・設計領域が**最適化後の形**になり、次に解くとそこから更に削られる
    #       （何度も回すたびに痩せて、最後は座面だけの骨になる）
    #     ・bbox の中心＝輪郭の原点が動くので、次に出る輪郭を組立に入れると
    #       板が**丸ごとずれる**
    #     ・元の質量が失われて「何 g 減ったか」が出せなくなる
    #   なので**最適化前の幾何を別ファイルに凍結する**。`_base.json` は
    #   一度書いた板は二度と上書きしない（`_parts.json` を作り直しても残る）。
    base = load_base()
    frozen = set(base)
    if frozen:
        print(f"基準を凍結してある板 {len(frozen)} 枚: {', '.join(sorted(frozen))}")

    outlines: dict[str, list] = {nm: base[nm]["outline"] for nm in frozen
                                 if base[nm].get("outline")}
    for nm, d in info.items():
        if nm in frozen:
            continue
        b = d["box"]
        ext = [b[1] - b[0], b[3] - b[2], b[5] - b[4]]
        k = min(range(3), key=lambda i: ext[i])
        if ext[k] > PA.PLATE_T_MAX or nm not in by_name:
            continue
        side = sorted(ext)[1:]
        if side[0] < PA.PLATE_MIN_SIDE or side[0] * side[1] < PA.PLATE_AREA_MIN:
            continue
        ring = outline_of(by_name[nm], b, k)
        if ring:
            outlines[nm] = ring

    parts = {nm: {"box": list(d["box"]), "vol": d["vol"], "mat": d["mat"],
                  "rho": d["rho"], "mass": d["mass"],
                  "sub": sub.get(nm, d["mass"])}
             for nm, d in info.items()}
    for nm in frozen:
        if nm in parts:
            parts[nm] = dict(base[nm]["part"])

    return {
        "pose": pose,
        "parts": parts,
        "outlines": outlines,
        # 「自分が留まっている先」。向きが荷重の上流／下流を決める
        "fixings": {nm: [[t, h, q] for t, h, q, _n in lst]
                    for nm, lst in F.FIXINGS.items()},
        # **bbox が実体を表していない部材**（斜めに寝た押出材）。
        # ⚠ 境界条件は相手の bbox から作っている。斜材のように傾いた部材は
        #   bbox が実体よりずっと大きいので、そのまま座面にすると板の
        #   6 割が「相手の座面」になり、削れる場所が残らない。ここに
        #   載っている相手は `topo_opt` が別扱いにする。
        "slot_axis": {nm: list(v) for nm, v in F.SLOT_AXIS.items()},
        # ねじ 1 本ごとの呼び径。板を貫くねじの**逃げ穴の大きさ**に使う。
        # ⚠ 実体の bbox で穴を開けてはいけない。溝ナット付きのねじは
        #   bbox が φ11 になるが、板を貫くのは軸（M5 ならバカ穴 φ5.5）で、
        #   ナットは相手の溝の中＝板の反対側にいる。
        "fastener": {nm: [k, float(s)] for nm, (k, s, _l, _e) in F.FASTENER.items()},
        "roots": sorted(F.ROOTS),
    }


BASE = os.path.join(ROOT, "out", "topo", "_base.json")


def load_base() -> dict:
    """最適化前の板の幾何（`{板名: {"part": …, "outline": […]}}`）。

    ⚠ **これは生成物ではなく基準**。一度書いた板は上書きしない。組立を
      変えたら `_parts.json` は作り直すが、ここは残す。板の形を本当に
      作り直したい（板を大きくした、締結を足した）ときだけ、その板の
      エントリを手で消すこと。
    """
    if not os.path.exists(BASE):
        return {}
    with open(BASE, encoding="utf-8") as fp:
        return json.load(fp)


def freeze(cache: dict, names) -> None:
    """まだ凍結していない板の幾何を `_base.json` に足す。"""
    base = load_base()
    added = []
    for nm in names:
        if nm in base or nm not in cache["parts"]:
            continue
        base[nm] = {"part": cache["parts"][nm],
                    "outline": cache.get("outlines", {}).get(nm)}
        added.append(nm)
    if added:
        os.makedirs(os.path.dirname(BASE), exist_ok=True)
        with open(BASE, "w", encoding="utf-8") as fp:
            json.dump(base, fp, ensure_ascii=False)
        print(f"基準を凍結した板 {len(added)} 枚: {', '.join(sorted(added))}")


def load() -> dict:
    if not os.path.exists(PATH):
        raise SystemExit(
            f"キャッシュが無い: {PATH}\n"
            f"  `python scripts/topo_cache.py` で作ること")
    src = os.path.join(ROOT, "src")
    newest = max(os.path.getmtime(os.path.join(src, f))
                 for f in os.listdir(src) if f.endswith(".py"))
    if os.path.getmtime(PATH) < newest:
        print("⚠ キャッシュが CAD より古い。`python scripts/topo_cache.py` "
              "で作り直すこと", file=sys.stderr)
    with open(PATH, encoding="utf-8") as fp:
        return json.load(fp)


if __name__ == "__main__":
    os.makedirs(os.path.dirname(PATH), exist_ok=True)
    d = build()
    with open(PATH, "w", encoding="utf-8") as fp:
        json.dump(d, fp, ensure_ascii=False)
    print(f"部品 {len(d['parts'])} 個 / 宣言 {len(d['fixings'])} 件 / "
          f"板の投影外形 {len(d['outlines'])} 枚 → {PATH}")
