# NHK 高専ロボコン2026「高専杯 雑巾投擲選手権」

全国優勝を目指すための戦略・技術選定・スケジュール・試合シミュレーターのリポジトリ。

> このリポジトリの戦略書・設計資料・シミュレーターは、生成AI（Claude / OpenAI）を活用して作成・検証したものです。数値は物理シミュレーションによる相対比較であり、絶対値は実射データでの較正を前提としています。

## 構成

| パス | 内容 |
|---|---|
| `strategy.md` | 戦略書（ルール分析・技術選定の比較検討・機構/制御詳細・スケジュール・シム検証結果§11） |
| `simulator.md` | シミュレーターの設計・検証ループ運用・強化学習の見通し（未経験者向け解説） |
| `viewer/` | **試合シミュレーター** (Vite + Svelte 5 + TypeScript strict + three.js) |
| `gpusim/` | 雑巾（布）弾道のGPUバッチ物理シム (PyTorch / CUDA) |

## シミュレーター

デモ（公開URL）: <https://nhk-k2026-viewer.pages.dev>

### 開発・実行

```bash
cd viewer
npm install
npm run dev      # 開発サーバー (http://localhost:5173)
npm run check    # 型チェック (svelte-check, 0 errors 維持)
npm run build    # dist/ へビルド
npm run verify   # モンテカルロ検証実験 → 結果を stdout + verify-report.json
```

### Cloudflare Pages へのデプロイ（外部公開）

初回のみブラウザ認証が必要:

```bash
cd viewer
npx wrangler login   # ブラウザが開いて Cloudflare にログイン
npm run deploy       # build + wrangler pages deploy (プロジェクト名 nhk-k2026-viewer)
```

以後は `npm run deploy` だけで `https://nhk-k2026-viewer.pages.dev` に公開される。
※ 戦略情報を含むシミュレーターを公開URLに置くことになる点は留意（URLを知らない限り見えないが非公開ではない）。

### 機能

- 放送風HUD（左上=赤チーム得点+内訳 / 中央=残り時間タイマー / 右上=青チーム。フォントはAnton+Noto Sans JP 900+DSEG7）
- strategy.md §2.3 のプランを再生: ボーナス走→旗刈り→スマート切替→スーパー雑巾→デナイアル
- 相手アーキタイプ3種（手動装填/自動装填/旗ブロック）、自軍戦略4種を切替可能
- リアリティ: メカナム4輪の逆運動学回転・樽ローラー空転、射出ローラーのスピンアップ、
  スタックグラバー動作、抗力+バックスピン揚力の弾道、布のはためき、衝突判定+A*経路生成
- センサー可視化: 照準カメラPiP（相手車体検出BBox+バケツ推定）、2D LiDAR点群（スキャン面0.12m）+MCL粒子ミニマップ
- Web Audio合成の効果音（モーター/射出/ブザー/歓声）
- 決定論シード: 同条件なら同じ試合を再現（回帰テスト・将来のRL基盤）

## 重要日程（東海北陸地区）

- **8/24（月）** エントリー・最終アイデアシート締切（以降アイデア変更原則不可）
- **10/7（水）** チーム紹介シート等 提出締切
- **10/18（日）** 東海北陸地区大会
- **11/15（日）** 全国大会

## ライセンス

- 本リポジトリは [MIT License](LICENSE) の下で公開しています。
- 依存ライブラリはそれぞれのライセンス（MIT / Apache-2.0 等）に従います。
