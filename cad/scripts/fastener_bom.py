"""締結具の**員数表** — 宣言と実体を突き合わせて「何本買うか」を出す.

    python scripts/fastener_bom.py [--md] [--limit N]

なぜ要るか
-----------
`src/tr_fix.py` の固定宣言のうち、締結具を消費するものは 478 組ある。
どの組にも "4-M3 PCD26" "2-M5" といった **note が付いているのに、ねじの
実体は 56 本しか無かった**。「留まっている」と書いてある箇所のほとんどに、
ねじが描かれていない。

これは 2 つの意味で問題だった。

  1. **買う数が分からない。** 発注は note を人が数えるしかなく、
     宣言を 1 つ増やしても員数表は誰も直さない。
  2. **質量が合わない。** 質量台帳は「ボルト・ナット・スペーサ類 700g」
     という 1 行の概算だった。35kg 規定の余裕が 160g しか無い設計で、
     この 1 行が最大の不確かさだった。

ここでやること
---------------
  A. 固定宣言（と note の本数・呼び径）を読み、**対応する実体のねじが
     置かれているか**を照合する → 「未実体」の一覧
  B. 実体のあるねじは `F.FASTENER`（`put_screw()` が控える規格）から、
     未実体の締結は宣言と**相手の板厚**から、それぞれ員数を出す
  C. 種類 × 呼び径 × 長さ の員数表、購入単位（袋）での必要数、質量合計

数え方そのものは `src/tr_lib.py` に置いてある。
**質量台帳（tr_assembly の LEDGER）が同じ関数を呼ぶ**ので、
図の本数・買う本数・質量の 3 つがずれようがない。ここはその表示係。

読み方の規約
-------------
  * note の "4-M4" は**その組 1 つに要る本数**と読む。
  * `BRACKET` の宣言は金具自身の TSLOT 宣言と二重なので数えない。
  * ねじ自身（`scr_*` / `nip_screw_*`）が主語・目的語の宣言は数えない
    （それはねじが**留まっている先**の話）。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402


def size_str(kind: str, d: float) -> str:
    # ⚠ リベットの呼びは**穴径 φ**でねじの M ではない。`kind == "RIVET"` と
    #   完全一致で見ていたので、皿頭（`RIVET_FLAT`）だけ「M3」と出ていた。
    #   発注書に M3 と書いてあれば、届くのはねじになる。
    return f"φ{d:g}" if kind.startswith("RIVET") else f"M{d:.0f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    # ⚠ 既定を大きく取る。未実体の一覧は**発注のための表**なので、
    #   途中で切れていると「残りは誰かが数える」ことになって元に戻る。
    ap.add_argument("--limit", type=int, default=400)
    args = ap.parse_args()

    A.build(P.POSE_MATCH)
    joints, placed, missing, stray, summary = L.fastener_joints()
    cnt, src = L.fastener_tally(missing)
    total = sum(cnt.values())
    mass = sum(n * L.fastener_mass(*k) for k, n in cnt.items())
    n_missing = sum(r[4] for r in missing)
    # 質量台帳の「締結具」グループと突き合わせる（同じ関数から積んでいる）
    ledger = sum(i.qty * i.unit_kg for i in L.LEDGER.items if i.group == "締結具")

    bad = []
    if stray:
        bad.append(f"宣言のどの組にも紐づかないねじ {len(stray)} 本")
    if abs(ledger - mass) > 0.0005:
        bad.append(f"質量台帳 {ledger * 1000:.0f}g と員数表 {mass * 1000:.0f}g が違う")
    # ⚠ **買えない長さが表に載っていた。** `L.screw_len()` は流通する呼び長さへ
    #   切り上げるが、`put_screw(..., length=26.0)` のように**手で書いた**ものは
    #   そこを通らない。実際 M4×26 が 12 本載っていて、25 と 30 のあいだの
    #   26mm は棚に無い。発注書に無い長さがあると、現場で別の長さに置き換わり、
    #   その時点で図面と実機がずれる（`screw_len` の註と同じ理由）。
    for k, n in sorted(cnt.items()):
        kind, _d, ln = k
        if kind in ("CAP", "FLAT") and ln and ln not in L.SCREW_LENGTHS:
            near = min(L.SCREW_LENGTHS, key=lambda v: abs(v - ln))
            bad.append(f"{L.KIND_JA[kind]} {size_str(kind, k[1])}×{ln:.0f} は"
                       f"流通する呼び長さに無い（{n} 本。いちばん近いのは {near}）")

    if args.md:
        print("固定宣言（`src/tr_fix.py` の FIXINGS）と、実体として置いたねじ"
              "（`put_screw()` が控える規格）を突き合わせ、**買う数**を出す。\n")
        print("| 項目 | 値 |")
        print("|---|---|")
        print(f"| 締結の宣言（組） | {len(joints)} |")
        print(f"| うち実体のねじが入っている組 | {len(placed)} |")
        print(f"| 実体のねじ | {len(F.FASTENER)} 本 |")
        print(f"| 未実体（宣言だけ） | {n_missing} 本ぶん・{len(missing)} 組 |")
        print(f"| 締結具の総数 | **{total}** 個 |")
        print(f"| 締結具の総質量 | **{mass * 1000:.0f} g** |")
        print(f"| 質量台帳（グループ「締結具」） | {ledger * 1000:.0f} g |")
        for b in bad:
            print(f"| NG | {b} |")
        print()
        print("### 員数表（種類 × 呼び径 × 長さ）\n")
        print("| 種類 | 呼び径 | 長さ [mm] | 個数 | 1袋 | 袋数 | 質量 [g] |")
        print("|---|---|---|---|---|---|---|")
        for kind in L.KIND_ORDER:
            for k in sorted((k for k in cnt if k[0] == kind),
                            key=lambda k: (k[1], k[2])):
                n, pk = cnt[k], L.PACK[kind]
                print(f"| {L.KIND_JA[kind]} | {size_str(kind, k[1])} | "
                      f"{f'{k[2]:.0f}' if k[2] else '—'} | **{n}** | {pk} | "
                      f"{-(-n // pk)} | {n * L.fastener_mass(*k) * 1000:.0f} |")
        print(f"| **合計** |  |  | **{total}** |  |  | **{mass * 1000:.0f}** |")
        print()
        print(f"> 内訳: 実体から数えた {src['実体']} 個 / "
              f"宣言から見積もった {src['推定']} 個。質量は規格外形"
              "（ISO 4762 / 4032 / 7089）からの体積 × "
              f"{L.RHO_FASTENER * 1000:.2f} g/cm³（A2-70 ステンレス）。\n")

        if missing:
            print(f"### 実体のねじが足りない締結 {len(missing)} 組"
                  f"（{n_missing} 本ぶん）\n")
            print("> 宣言はあるが図にねじが無い。長さは**相手の板厚からの"
                  "推定**なので、実体を置いたときに変わりうる。\n")
            print("| 部品 A | 部品 B | 方法 | note | 不足 | 見積もり |")
            print("|---|---|---|---|---|---|")
            for a, b, how, note, n, size, rivet, flush in missing[:args.limit]:
                kind, d, ln, ex = L.estimate_fastener(a, b, how, size, rivet, flush)
                ex_s = "".join(f" + {L.KIND_JA[k]}" for k, _ in ex)
                print(f"| `{a}` | `{b}` | {how} | {note or '—'} | {n} | "
                      f"{size_str(kind, d)}×{ln:.0f}{ex_s} |")
            if len(missing) > args.limit:
                print(f"\n（他 {len(missing) - args.limit} 組）")
        if stray:
            print(f"\n### 宣言に紐づかないねじ {len(stray)} 本\n")
            for nm in stray[:args.limit]:
                print(f"- `{nm}` → {'、'.join(F.targets_of(nm))}")
        print()
        print(f"> `BRACKET` の宣言 {len(summary)} 組は、金具自身の TSLOT 宣言と"
              "同じ締結を指すので数えていない（数えると 2 倍になる）。")
    else:
        print(f"宣言 {len(joints)} 組 / 実体 {len(F.FASTENER)} 本 / "
              f"未実体 {n_missing} 本（{len(missing)} 組）")
        for kind in L.KIND_ORDER:
            for k in sorted((k for k in cnt if k[0] == kind),
                            key=lambda k: (k[1], k[2])):
                n = cnt[k]
                ln = f"×{k[2]:.0f}" if k[2] else ""
                print(f"  {L.KIND_JA[kind]:<24} {size_str(kind, k[1]):<5}{ln:<5}"
                      f"{n:5d} 個  {-(-n // L.PACK[kind]):3d} 袋"
                      f"  {n * L.fastener_mass(*k) * 1000:7.0f} g")
        print(f"  {'合計':<24} {'':<10}{total:5d} 個"
              f"           {mass * 1000:7.0f} g")
        print(f"  実体から {src['実体']} 個 / 宣言から見積もり {src['推定']} 個")
        print(f"  質量台帳（締結具）= {ledger * 1000:.0f} g")
        for a, b, how, note, n, size, rivet in missing[:args.limit]:
            kind, d, ln, _ex = L.estimate_fastener(a, b, how, size, rivet)
            print(f"    未実体 {n} 本  {a} ↔ {b} [{how}] "
                  f"note={note or '—'} → {size_str(kind, d)}×{ln:.0f}")
        if len(missing) > args.limit:
            print(f"    （他 {len(missing) - args.limit} 組）")
        for nm in stray[:args.limit]:
            print(f"    紐づかず {nm} → {'、'.join(F.targets_of(nm))}")
        for b in bad:
            print(f"    [NG] {b}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
