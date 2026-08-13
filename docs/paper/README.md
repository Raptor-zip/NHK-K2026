# TR CAD 設計論文（LaTeX）

TR（投擲ロボット）の CAD を構想から製作データ確定（`cad-v1.0`）まで持っていく過程を，
試行錯誤ごと研究論文形式でまとめたもの。外部公開を前提に書いてある。

- `tr_cad.tex` — 本体（日本語・luatexja / ltjsarticle・2 段組）
- `fig1.py` — Fig.1（系統色分けの投影図＋寸法注記）の生成器。
  組立から実測した値だけを描く（図に数値を手書きしない）。

  ```bash
  cd cad && .venv/bin/python ../docs/paper/fig1.py --rebuild   # 組立から作り直す（数分）
  cd cad && .venv/bin/python ../docs/paper/fig1.py             # 体裁だけ詰める（数秒）
  ```

  `--rebuild` は投影用の三角メッシュを `figs/.fig1_cache.pkl`（追跡しない）に取る。
  CAD を直したら必ず `--rebuild` から。
- `fig2.py` — Fig.2（位相最適化の入力と出力）の生成器。`topo_opt.collect()` の境界条件と
  `out/topo/<板>.json` の輪郭から描く（組立は作らないので数秒）。
- `figs/` — 図版。Fig.1・Fig.2 以外は `cad/out/` 以下の**自動生成物からのコピー**
  （matplotlib による投影図・トポロジー最適化の密度場・PyBullet の機構シム）。
  外部レンダラで焼いた絵は使っていない。
- `tr_cad.pdf` — 出力（9 ページ）
- `tr_cad_2p.tex` / `tr_cad_2p.pdf` — 2 ページ版（A4・2 段組）。
  図は 8 ページ版と同じものを使う。SNS などで画像として出す用途を想定している。
  体裁は講演会論文の一般的な形に寄せてあるが、特定の学会の様式ではない。

## ビルド

```bash
cd docs/paper
# ⚠ 相互参照（図表番号）が ?? のまま残るので必ず 2 回以上回す
lualatex tr_cad.tex
lualatex tr_cad.tex
```

TeX Live の `luatexja`（`ltjsarticle.cls`）と原ノ味フォントが要る。

## 図版を更新するとき

図は `cad/out/` の生成物なので，CAD を直したら元を作り直してからコピーし直す。

```bash
cd cad
.venv/bin/python scripts/render.py        # out/render_*.png
.venv/bin/python scripts/topo_opt.py --md # out/topo/*.png
cp out/render_match.png ... ../docs/paper/figs/
```

## 内容の出どころ

- `cad/DESIGN.md`（§0〜§45）
- 設計記録（非公開）
- git 履歴 161 コミット
