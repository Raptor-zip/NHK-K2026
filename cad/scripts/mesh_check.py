#!/usr/bin/env python3
"""URDF が参照する STL が CAD と一致しているかを確かめる（書き出し直して比べる）。

⚠ **meshes/*.stl も「生成物」なのに、生成できない状態で放置されていた。**
  書き出しはリンクごとに link_*() を直接呼ぶので、左右の車輪・上下のローラーで
  同じ部品を 2 回 put する。組立側に同名 put の検査を入れた日から例外で落ち、
  **CAD を変えてもメッシュが更新されなかった**。どれくらい古かったか:
      grabber_press.stl   1,284 → 180,284 バイト（25 三角形＝ただの箱だった）
      grabber_slide.stl  55,284 → 491,284
      turret_yaw.stl    290,284 → 705,684
  シミュレーション（sim/tr_sim.py）はこれを見るので、古いままだと
  **画面では動いているのに実物と違う**という食い違いになる。

⚠ **書き出しが材質ごとにファイルを分けた日から、この検査は空回りしていた。**
  `export_meshes` が `base_link.stl` を `base_link__A5052.stl` … に変えたのに、
  ここは `<リンク名>.stl` を探し続けたので、**10 リンク全部が「STL が無い」**
  になった。全部落ちる検査は「直すもの」ではなく「無視するもの」になるので、
  そのあいだ STL の鮮度は誰も見ていない。
  → 分割の規則は書き写さず、`export_meshes` の関数をそのまま呼ぶ。

⚠ 判定に **bbox は使えない**。三角形分割は曲面の内側に入るので、円筒は
  半径 45 で 0.126mm 小さく出る（食い違いではない）。逆に古い
  grabber_press.stl は 25 三角形の箱なのに bbox は正しかった。
⚠ **1 バイトずつ比べるのも駄目だった。** 同じ形状を 2 回書き出すと
  base_link は 4,636,634 バイト中 14,505 バイトが違う（三角形分割の
  浮動小数点が実行ごとにわずかに揺れる）。三角形数は 92,731 で安定。
  → 判定は **三角形数（完全一致）+ bbox（0.3mm 以内）**。
    三角形数は形が変われば必ず変わるので、古い箱（25 三角形）は確実に出る。
    mtime では見ない（git の checkout で変わる）。
"""
from __future__ import annotations

import json
import struct
import sys
import tempfile
from pathlib import Path

_R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_R / "src"))
sys.path.insert(0, str(_R / "scripts"))

from build123d import export_stl  # noqa: E402

import tr_assembly as A  # noqa: E402

A.PUT_ALLOW_DUP = True

import export_meshes as EM  # noqa: E402


BBOX_TOL = 0.3      # mm。曲面の三角形分割は内側に入る（半径 45 で 0.126mm）


def stl_stat(path: Path):
    """(三角形数, bbox) を返す。"""
    b = path.read_bytes()
    n = struct.unpack_from("<I", b, 80)[0]
    lo = [1e18] * 3
    hi = [-1e18] * 3
    for k in range(n):
        off = 84 + k * 50 + 12          # 法線 12 バイトを飛ばす
        for v in range(3):
            for a in range(3):
                x = struct.unpack_from("<f", b, off + v * 12 + a * 4)[0]
                lo[a] = min(lo[a], x)
                hi[a] = max(hi[a], x)
    return n, lo, hi


def fresh_export(name, fn, td):
    """1 リンクを**書き出しと同じ材質分割で**temp へ出し、{ファイル名: 統計} を返す。

    ⚠ 分割の仕方をここに書き写さないこと。`export_meshes` の `_leaves` /
      `_rgb_of` / `_compound` をそのまま呼ぶ。書き写すと、分割の規則が
      変わったときに**検査だけ古い規則のまま通る**（それがこの検査を
      1 件も見なくした原因そのもの）。
    """
    groups: dict[tuple, list] = {}
    for leaf, loc in EM._leaves(fn()):
        groups.setdefault(tuple(round(v, 4) for v in EM._rgb_of(leaf)), []).append(
            (leaf, loc))
    out = {}
    for rgb, solids in groups.items():
        mat_name = EM.RGB_NAME.get(rgb, "OTHER")
        p = Path(td) / f"{name}__{mat_name}.stl"
        export_stl(EM._compound(solids, label=f"{name}:{mat_name}"), str(p),
                   tolerance=EM.TOLERANCE, angular_tolerance=EM.ANGULAR_TOL)
        out[p.name] = stl_stat(p)
    return out


def main() -> int:
    have_dir = _R / "urdf" / "meshes"
    bad = []
    # ⚠ **manifest（materials.json）も生成物**。URDF はこれを読んで
    #   <visual> を並べるので、STL が合っていても manifest が古いと
    #   ビューアの色と実体がずれる。ファイルの集合として突き合わせる。
    man_path = have_dir / "materials.json"
    manifest = {}
    if man_path.exists():
        try:
            manifest = json.loads(man_path.read_text(encoding="utf-8"))
        except Exception as e:
            bad.append(("materials.json", f"読めない: {e}"))
    else:
        bad.append(("materials.json", "無い"))

    listed = {f.name for f in have_dir.glob("*.stl")}
    expected = set()
    with tempfile.TemporaryDirectory() as td:
        for name, fn in EM.LINKS.items():
            stats = fresh_export(name, fn, td)
            expected |= set(stats)
            man_files = {p["file"] for p in manifest.get(name, [])}
            if manifest and man_files != set(stats):
                for f in sorted(set(stats) - man_files):
                    bad.append((name, f"materials.json に {f} が無い"))
                for f in sorted(man_files - set(stats)):
                    bad.append((name, f"materials.json の {f} はもう作られない"))
            tri = 0
            for fname, (n2, lo2, hi2) in sorted(stats.items()):
                have = have_dir / fname
                if not have.exists():
                    bad.append((name, f"{fname} が無い"))
                    continue
                n1, lo1, hi1 = stl_stat(have)
                d = max(max(abs(a - b) for a, b in zip(lo1, lo2)),
                        max(abs(a - b) for a, b in zip(hi1, hi2)))
                tri += n1
                if not (n1 == n2 and d <= BBOX_TOL):
                    bad.append((name, f"{fname}: 三角形 {n1:,d} ↔ {n2:,d} / "
                                      f"bbox 差 {d:.3f}mm"))
            print(f"  {name:16s} {len(stats):2d} 材質  三角形 {tri:8,d}")

    # 書き出しは毎回 *.stl を消してから作るので、余分なファイルは
    # 「手で置いた」か「もう作られない材質」のどちらか。どちらも消す対象。
    for f in sorted(listed - expected):
        bad.append(("(余分)", f"{f} は書き出しが作らないファイル"))

    print()
    if bad:
        print(f"### STL が CAD と食い違っている {len(bad)} 件")
        for nm, why in bad:
            print(f"  {nm}: {why}")
        print("\n  → `python scripts/export_meshes.py` で書き出し直すこと")
        return 1
    print(f"STL {len(expected)} 枚は CAD と一致"
          "（URDF の見た目・シミュレーションは最新）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
