"""手順書が CAD と食い違っていないかを見る（ASSEMBLY.md / DESIGN.md / BOM.md）.

    python scripts/doc_check.py [--md]

なぜ要るか
----------
`BOM.md` は `scripts/bom.py` が **CAD から生成する**が、`ASSEMBLY.md` は
**手で書いた手順書**で、部材の長さも本数も人が書き写している。書き写した数字は
必ず腐る。実際こうなっていた:

    ASSEMBLY.md 工程0「BOM.md の切断リストと現物を突き合わせる（22本 / 15.97m）」
    BOM.md      「合計 28 本 / 15.46 m」

手順書のとおりに受け入れ検査をすると、**6 本足りないのに「合っている」**になる。
しかも手順書は「BOM を見ろ」と書いてあるので、読む人は両方見て混乱する。

見るもの
--------
1. 手順書に出てくる `L<長さ>` が、生成した切断リストに実在するか
2. 「N本 / X m」の合計が切断リストと合っているか
3. **同じ文書の中で矛盾していないか**。「⚠ φ26 ではない」と書いてある文書の
   別の場所に「インロー φ26 が座に入っていることを確認」と書いてあった。
   どちらを信じるかは読む人によって変わる。
4. 型番（SRX3616 など）が `tr_params.py` の値と一致しているか

⚠ **数字を手順書から消す方向で直さないこと。** 「BOM を見ろ」だけの手順書は
  現場で使えない（受け入れ検査は紙を見ながらやる）。数字は書いたうえで、
  この検査で腐りを落とす。
"""

from __future__ import annotations

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

import tr_params as P  # noqa: E402

# ⚠ **`BOM.md` は入れない。** あれは `scripts/bom.py` が毎回 CAD から作る
#   生成物なので、腐りようがない（腐るなら生成器のほう）。しかも
#   `check_all` は bom と doc_check を**並列に**回すので、doc_check が先に
#   読むと「まだ書き換わっていない BOM.md」と比べて落ちる。
# ⚠ **`DESIGN.md` も入れない。** あれは**設計の記録**で、「前は L1015 と
#   書いてあった」「1407 は取り残しだった」のように**過去の誤った値を
#   わざと引用する**文書。引用と指示を機械が区別する方法は無いので、
#   ここへ入れると「直した記録を書いた瞬間に検査が落ちる」ことになる。
#   （`check_all` の判定語の註にある「意図的に置いた対照行」と同じ話）
# 見るのは**現場が紙を見ながら従う手順書**だけ。
DOCS = ("ASSEMBLY.md",)

# 手順書に書いてある**確認寸法**と、その正解の出どころ。
#   (正規表現, 正解を返す関数, 許容差 mm)
# ⚠ ここは「現場が紙を見ながら測る値」。腐ると、**測って合わないのに
#   合っていることにする**か、正しい機体を作り直すかのどちらかになる。
# ⚠ 正解は `tr_params` から採る。`out/validation.md` から採ると、
#   validate を回していない環境で「どちらも同じ古い値」になって通ってしまう。
DIMS = [
    (re.compile(r"バケツ上面の高さ\s*\*\*(\d+)\s*mm"), lambda P: P.BUCKET_TOP_Z, 1.0),
    (re.compile(r"2L\s*目盛り（約\s*(\d+)\s*mm"), lambda P: P.BUCKET_2L_Z, 1.0),
    (re.compile(r"バケツ底面\s*(\d+)"), lambda P: P.BUCKET_SEAT_Z, 1.0),
    (re.compile(r"机上面\s*(\d+)"), lambda P: P.DESK_H, 0.5),
]

# 「⚠ **φ26 ではない。**」の形で否定された値を拾う
DENY_RE = re.compile(r"\*\*([φΦ]?[\d.]+(?:mm)?)\s*ではない")
# 部材長「L840」「L1015×2」
LEN_RE = re.compile(r"\bL(\d{2,4})\b")
# 合計「**22本 / 15.97m**」「**28 本 / 15.46 m**」
TOTAL_RE = re.compile(r"\*\*(\d+)\s*本\s*/\s*([\d.]+)\s*m\*\*")


def cut_list():
    """`scripts/bom.py` と同じ切断リスト（長さ → 用途の並び）。全型番をまとめる。

    ⚠ 数え方を書き写さないこと。同じ関数を呼ぶ。
    ⚠ **型番をまたいでまとめる。** 手順書は「L840」としか書かないので、
      2020 か 2010 かは長さからは分からない。突き合わせるのは長さの実在。
    """
    import bom
    out: dict[int, list[str]] = {}
    for series, cl in bom.cut_lists().items():
        for ln, names in cl.items():
            out.setdefault(ln, []).extend(f"{series} {n}" for n in names)
    return dict(sorted(out.items(), reverse=True))


def read(name):
    p = os.path.join(ROOT, name)
    return p, (open(p, encoding="utf-8").read() if os.path.exists(p) else "")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    cuts = cut_list()
    n_cut = sum(len(v) for v in cuts.values())
    total_m = sum(k * len(v) for k, v in cuts.items()) / 1000.0
    bad = []

    for name in DOCS:
        path, text = read(name)
        if not text:
            bad.append((name, "ファイルが無い"))
            continue
        lines = text.splitlines()

        # 1. 部材長が切断リストに実在するか
        seen = {}
        for i, ln in enumerate(lines, 1):
            for m in LEN_RE.finditer(ln):
                seen.setdefault(int(m.group(1)), i)
        for v, i in sorted(seen.items()):
            if v not in cuts:
                near = min(cuts, key=lambda k: abs(k - v))
                bad.append((name, f"{i} 行目の L{v} は切断リストに無い"
                                  f"（いちばん近いのは L{near}）"))

        # 2. 合計本数・合計長さ
        for i, ln in enumerate(lines, 1):
            for m in TOTAL_RE.finditer(ln):
                n, mm = int(m.group(1)), float(m.group(2))
                if n != n_cut or abs(mm - total_m) > 0.02:
                    bad.append((name, f"{i} 行目の合計 {n}本/{mm:.2f}m は"
                                      f"切断リスト {n_cut}本/{total_m:.2f}m と違う"))

        # 3. 同じ文書の中の矛盾
        # ⚠ 否定している行そのものは除く。「φ26 ではない」の行に φ26 が
        #   出てくるのは当たり前で、それを矛盾と数えると必ず落ちる。
        deny = {}
        for i, ln in enumerate(lines, 1):
            for m in DENY_RE.finditer(ln):
                deny.setdefault(m.group(1).rstrip("mm"), []).append(i)
        for tok, deny_lines in deny.items():
            for i, ln in enumerate(lines, 1):
                if i in deny_lines or tok not in ln:
                    continue
                # 否定の理由を説明している行（同じ段落）は除く
                if "ではない" in ln or "ガバガバ" in ln or "内側" in ln:
                    continue
                bad.append((name, f"{i} 行目が {tok} を指示しているが、"
                                  f"{deny_lines[0]} 行目は「{tok} ではない」と書いている"))

        # 4. 型番
        if "SRX" in text and P.RAIL_MODEL not in text:
            bad.append((name, f"スライドレールの型番が {P.RAIL_MODEL} と違う"))

        # 5. 確認寸法
        for rx, fn, tol in DIMS:
            want = fn(P)
            for i, ln in enumerate(lines, 1):
                for m in rx.finditer(ln):
                    got = float(m.group(1))
                    if abs(got - want) > tol:
                        bad.append((name, f"{i} 行目の {m.group(0)} は "
                                          f"{want:.0f} と違う（差 {got - want:+.0f}mm）"))

    if args.md:
        print("# 手順書と CAD の突き合わせ\n")
        print(f"切断リスト: **{n_cut} 本 / {total_m:.2f} m**\n")
        if not bad:
            print("食い違いなし。")
        else:
            print(f"### 食い違い {len(bad)} 件\n")
            cur = None
            for doc, why in bad:
                if doc != cur:
                    print(f"\n**{doc}**\n")
                    cur = doc
                print(f"- {why}")
    else:
        print(f"切断リスト {n_cut} 本 / {total_m:.2f} m")
        for doc, why in bad:
            print(f"  ⚠ {doc}: {why}")
        if not bad:
            print("食い違いなし")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
