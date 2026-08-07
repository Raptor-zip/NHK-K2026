"""端面タップの成立性を見る。

⚠ 2026-08-05 に曲げ品を全部「平板 2 枚 + 端面タップ」へ置き換えた。
  端面（板の小口）へのタップは自校でもできる加工だが、**板厚が
  `tr_lib.TAP_MIN` を満たしていないとねじ山が立たない**。
  M4 なら 6mm、M5 なら 7.5mm 要る。t3 の小口に M4 と書いても、
  山が 1.5 山しか掛からず、締めた瞬間になめる。

  図面には出ない（穴は開く）ので、**目視では絶対に見つからない**。
  分割を機械的にやった結果として大量に混ざるたぐいの不良なので、
  宣言（`tr_fix.FIXINGS` の注記）から機械的に潰す。

判定:
  注記に「端面」と「タップ」の両方が入っている締結について、
  組の**どちらか一方**でも `TAP_MIN[径]` 以上の板厚を持っていれば合格。
  （どちらにタップを立てるかは注記の日本語からは確実に読めないので、
   「厚いほうへ立てられる」ことだけを要求する。両方薄ければ確実に不良。）

⚠ 板厚は外接箱の最小辺で取る。肉抜き穴やテーパがあっても、板材から
  切り出す部品なら最小辺＝板厚になる。押出材・購入品は対象外
  （`export_fab.classify` が CUT と判定したものだけ見る）。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402
from assembly_check import part_name  # noqa: E402

SIZE = re.compile(r"M(\d+(?:\.\d+)?)")


def main() -> int:
    md = "--md" in sys.argv
    bb: dict[str, list] = {}
    for path, _s, b in V.solids_with_bbox(A.build(P.POSE_MATCH)):
        bb.setdefault(part_name(path), []).append(b)

    def thick(nm: str) -> float | None:
        bs = bb.get(nm)
        if not bs:
            return None
        x = max(b.max.X for b in bs) - min(b.min.X for b in bs)
        y = max(b.max.Y for b in bs) - min(b.min.Y for b in bs)
        z = max(b.max.Z for b in bs) - min(b.min.Z for b in bs)
        return min(x, y, z)

    rows, bad = [], []
    # ⚠ **皿もみの深さも見る（2026-08-05）。** 皿ねじの頭は板に沈むので、
    #   板がその深さ + 1mm ないと**穴の縁が抜ける**（頭が座らない）。
    #   実際 `car_brk` を t3 のまま「2-M4 皿」にすると残肉 0.7mm だった。
    #   ISO 10642 の頭の高さ: M3 1.65 / M4 2.2 / M5 2.75 / M6 3.3
    CSK_MIN = {3: 2.65, 4: 3.2, 5: 3.75, 6: 4.3}
    for src, lst in F.FIXINGS.items():
        for dst, how, _qty, note in lst:
            if note and "皿" in note and how in ("BOLT", "SCREW_IN"):
                m2 = SIZE.search(note)
                ta2, tb2 = thick(src), thick(dst)
                if m2 and ta2 is not None and tb2 is not None:
                    d2 = int(float(m2.group(1)))
                    nd = CSK_MIN.get(d2)
                    if nd is not None and max(ta2, tb2) < nd - 1e-6:
                        bad.append((src, dst, note,
                                    f"M{d2} の皿もみに t{nd:g} 以上が要るが、"
                                    f"{src} は t{ta2:.1f}、{dst} は t{tb2:.1f} "
                                    f"しかない。頭が座らないので板を厚くすること"))
            if not note or "端面" not in note or "タップ" not in note:
                continue
            m = SIZE.search(note)
            if not m:
                bad.append((src, dst, note, "注記にねじ径（M…）が無い"))
                continue
            d = float(m.group(1))
            need = L.TAP_MIN.get(int(d))
            if need is None:
                bad.append((src, dst, note, f"M{d:g} の下限が `TAP_MIN` に無い"))
                continue
            ta, tb = thick(src), thick(dst)
            if ta is None or tb is None:
                bad.append((src, dst, note,
                            f"実体が無い（{src}:{ta} {dst}:{tb}）"))
                continue
            ok = max(ta, tb) >= need - 1e-6
            rows.append((src, dst, f"M{d:g}", need, ta, tb, ok))
            if not ok:
                bad.append((src, dst, note,
                            f"M{d:g} の端面タップに t{need:g} 以上が要るが、"
                            f"{src} は t{ta:.1f}、{dst} は t{tb:.1f} しかない。"
                            f"厚いほうを t{need:g} にするか、"
                            f"リベット／貫通ボルト + ナットに変えること"))

    if md:
        out = ["端面タップの成立性。**どちらか一方**が `TAP_MIN` を満たせば合格。", ""]
        out.append("| 部品 | 固定先 | ねじ | 要 | 部品 t | 固定先 t | 判定 |")
        out.append("|---|---|---|---|---|---|---|")
        for src, dst, sz, need, ta, tb, ok in sorted(rows):
            out.append(f"| `{src}` | `{dst}` | {sz} | {need:g} | "
                       f"{ta:.1f} | {tb:.1f} | {'OK' if ok else '**NG**'} |")
        out.append("")
        if bad:
            out.append(f"### 立たない端面タップ {len(bad)} 件")
            out.append("")
            for src, dst, note, why in bad:
                out.append(f"- `{src}` → `{dst}`（{note}）… {why}")
        else:
            out.append("立たない端面タップは無い。")
        (ROOT / "out" / "edge_tap.md").write_text(
            "\n".join(out) + "\n", encoding="utf-8")
        print("\n".join(out[-30:]))
    else:
        for src, dst, note, why in bad:
            print(f"NG {src} → {dst}: {why}")
        print(f"端面タップ {len(rows)} 件 / 不良 {len(bad)} 件")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
