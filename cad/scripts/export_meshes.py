"""URDF 用リンクメッシュ (STL) をエクスポートする.

各リンクを「そのリンクのローカル座標系」で STL 化して cad/urdf/meshes/ に置く。
STL の単位は mm。URDF 側で scale 0.001 を掛けて m に変換する。

    python scripts/export_meshes.py

⚠ **STL は色を持てない。** 材質の色（`tr_lib.MAT_COLOR`）は組立の中では
  ソリッドごとに付いているので、**材質ごとに STL を分けて**書き出し、
  どの STL が何色かを `meshes/materials.json` に残す。URDF 側
  (`urdf/tr_urdf.py`) はそれを読んで 1 リンクに複数の <visual> を出す。
  1 枚にまとめると、ビューアではアルミもモーターもプラダンも同じ灰色になる。
"""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from build123d import Compound, Location, export_stl  # noqa: E402
from OCP.TopoDS import TopoDS_Builder, TopoDS_Compound  # noqa: E402

import tr_assembly as A  # noqa: E402
import tr_lib as L  # noqa: E402

OUT = os.path.abspath(os.path.join(HERE, "..", "urdf", "meshes"))
TOLERANCE = 0.4          # mm 弦誤差（表示用途に十分・ファイルサイズ優先）
ANGULAR_TOL = 0.3


# ⚠ リンクごとに link_*() を直接呼ぶので、同じ名前の部品を 2 回 put する
#   （左右の車輪・上下のローラー）。組立側の同名検査を外さないと**メッシュを
#   書き出せない**。実際この検査を入れた日から meshes/*.stl が 1 日以上
#   古いまま残り、URDF の見た目と当たり判定が CAD と食い違っていた。
A.PUT_ALLOW_DUP = True

LINKS = {
    "base_link": lambda: A.link_base(),
    "wheel_left": lambda: A.link_wheel(1),
    "wheel_right": lambda: A.link_wheel(-1),
    "turret_yaw": lambda: A.link_turret_yaw(),
    "shooter_pitch": lambda: A.link_pitch(),
    "roller": lambda: A.link_roller(),
    "singulator": lambda: A.link_singulator(),
    "grabber_slide": lambda: A.link_carriage(),
    "grabber_press": lambda: A.link_press(),
    "fork": lambda: A.link_fork(),
}


# 色の付いていないソリッドの既定（アルミフレームのアルマイト）。
# ⚠ ここに落ちる部品が多いなら、`tr_lib.mat()` を通っていない部品がある印。
DEFAULT_RGB = L.MAT_COLOR["A6005C"]

# 材質名を逆引きする（色 → 名前）。ファイル名に使うので読める名前にする
RGB_NAME = {tuple(round(v, 4) for v in rgb): nm for nm, rgb in L.MAT_COLOR.items()}


def _leaves(obj, acc=None):
    """Compound を葉まで降り、`(葉, その葉の絶対姿勢)` を返す。

    ⚠ **葉をそのまま取り出してはいけない。** `location` は親から見た相対値で、
      `Pos(0, y, 0) * Rot(90, 0, 0) * shooter_roller()` のように**親側に
      回転が乗っている**部品がある。裸の葉を別の Compound に詰め直すと、
      その回転が消えて**部品の向きが変わる**。
      実際これで射出ローラーが軸を縦にして描かれ（bbox 90×602×90 →
      90×602×44.2）、Y 軸まわりに回るはずのローラーが首を振っていた。
      分割前は 1 リンク 1 メッシュで、木をたどらないので起きなかった。

    姿勢は返すだけで、ここでは動かさない（下の `_compound` を使うこと）。
    """
    loc = obj.location if getattr(obj, "location", None) is not None else Location()
    cur = loc if acc is None else acc * loc
    ch = list(getattr(obj, "children", []) or [])
    if not ch:
        yield obj, cur
        return
    for c in ch:
        yield from _leaves(c, cur)


def _compound(placed, label=""):
    """`(葉, 絶対姿勢)` の並びを 1 つの Compound にまとめる。

    ⚠ **`Shape.located()` / `.moved()` を使ってはいけない。** どちらも中で
      `BRepBuilderAPI_Copy` を呼び、**形状を丸ごと複製する**。base_link は
      葉が数千個あるので、これだけで 14GB 使って 3 分たっても 1 リンクも
      書き出せなかった。OCCT の `TopoDS_Shape.Located()` は TShape（幾何の
      実体）を共有したまま姿勢だけ差し替えるので、ただの参照コピーで済む。
    """
    comp = TopoDS_Compound()
    builder = TopoDS_Builder()
    builder.MakeCompound(comp)
    for leaf, loc in placed:
        w = getattr(leaf, "wrapped", None)
        if w is None:
            continue
        builder.Add(comp, w.Located(loc.wrapped))
    return Compound(comp, label=label)


def _rgb_of(obj):
    c = getattr(obj, "color", None)
    if c is None:
        return DEFAULT_RGB
    try:
        return (float(c.red), float(c.green), float(c.blue))
    except Exception:
        try:
            return tuple(float(v) for v in list(c)[:3])
        except Exception:
            return DEFAULT_RGB


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    # 既に置いてある STL は消す（材質分割で名前が変わるので、古いものが残ると
    # URDF に無いファイルが混ざったままになる）
    for f in os.listdir(OUT):
        if f.lower().endswith(".stl"):
            os.remove(os.path.join(OUT, f))

    manifest: dict[str, list[dict]] = {}
    bad = []
    for name, fn in LINKS.items():
        shape = fn()
        groups: dict[tuple, list] = {}
        leaves = list(_leaves(shape))
        for leaf, loc in leaves:
            key = tuple(round(v, 4) for v in _rgb_of(leaf))
            groups.setdefault(key, []).append((leaf, loc))

        # ⚠ 詰め直したものが**元と同じ形**か確かめる。材質で分けるために葉を別の
        #   Compound へ移す以上、姿勢が落ちれば形が変わる。見た目は「それらしく」
        #   出てしまい、動かして初めて気づく（ローラーが軸を縦にして首を振った）。
        b0, b1 = shape.bounding_box(), _compound(leaves).bounding_box()
        gap = max(abs(getattr(b0.size, k) - getattr(b1.size, k)) for k in "XYZ")
        if gap > 0.5:
            bad.append(f"{name}: bbox が {b0.size.X:.1f}x{b0.size.Y:.1f}x{b0.size.Z:.1f}"
                       f" → {b1.size.X:.1f}x{b1.size.Y:.1f}x{b1.size.Z:.1f} に変わった")

        parts = []
        total = 0
        for rgb, solids in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            mat_name = RGB_NAME.get(rgb, "OTHER")
            fname = f"{name}__{mat_name}.stl"
            path = os.path.join(OUT, fname)
            export_stl(_compound(solids, label=f"{name}:{mat_name}"),
                       path, tolerance=TOLERANCE, angular_tolerance=ANGULAR_TOL)
            kb = os.path.getsize(path) / 1024
            total += kb
            parts.append({"file": fname, "material": mat_name,
                          "rgba": [round(v, 4) for v in rgb] + [1.0],
                          "solids": len(solids)})
        manifest[name] = parts
        bb = shape.bounding_box()
        print(f"{name:16s} {len(parts):2d} 材質 {total:8.0f} KB  "
              f"bbox {bb.size.X:7.1f} x {bb.size.Y:7.1f} x {bb.size.Z:7.1f}")
        for p in parts:
            print(f"    {p['material']:14s} {p['solids']:5d} 個  {p['file']}")

    if bad:
        print("\n✘ 材質分割で形が変わったリンクがある（姿勢が落ちている）:")
        for m in bad:
            print("   " + m)
        return 1

    with open(os.path.join(OUT, "materials.json"), "w", encoding="utf-8") as fp:
        json.dump(manifest, fp, ensure_ascii=False, indent=1)
    print(f"\n材質の割付 → {os.path.join(OUT, 'materials.json')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
