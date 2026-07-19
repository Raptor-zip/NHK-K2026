# gpusim — 雑巾投擲の CUDA GPU バッチ物理シミュレータ

`viewer/src/sim/rapier-rag.ts`(Rapier/WASM, CPU)の GPU 移植。PyTorch CUDA のテンソル演算で
**数百〜数千投を1バッチで同時に**シミュレートする(RTX PRO 5000 Blackwell / cu130)。
CPU 版は1投3〜4秒(worker並列でも N=300 で数分〜)だったが、GPU 版は**約1300投を一括 ≈8分**
(バッチ数を増やしてもほぼ同時間 = カーネル起動律速)。

## 物理(CPU版の忠実移植)

- 雑巾 = 2層(上下面)スラブ 12×16 セル・442ノード。ばね5種
  (structural/shear/bend/thickness/interShear、解像度較正込み)— 数値は TS 版と同一
- 空力: セル中面の向え角依存 圧力抗力(CD_N=1.5)+表面摩擦(CD_T=0.02)、
  付加質量(円板近似 ≈18%)、マグヌス(全体ω剛体近似, C=0.6)
- 射出 = **ベルト式直動(対向ベルト)**: ニップ内ノードを摩擦サーボでベルト面速度へ引き込み
  (バックスピンは上下ベルト速度差 spinFrac から創発、人工注入なし)
- 命中判定 = **物理**: 標的コライダー+床に実際に落とし、静定(全ノード<0.28m/s×16step)後に
  中に収まったか/掛かったかで判定
- Rapier の陰的ばねと違い陽的(半陰的オイラー)なので 1/240 フレームを 64 サブステップに分割して安定化。
  接触は押し出し+反発0+クーロン摩擦(μ=0.7/床0.8)

## 標的3種

| 標的 | 距離 | 判定 |
|---|---|---|
| 固定バケツ②③ (内半径0.137/リム高0.55) | 2.4m | 静定後にバケツ内 |
| 机①下棚 (W0.65×D0.45, 開口高0.5/天板0.76) | 4.8m | 静定後に棚内 |
| **旗 (横棒 y=3.0m, 半幅0.3, ポール+棒キャプセル)** | 3.92m | **横棒を跨いで掛かり、落ちずに残る** |

## 2つの物理エンジン(コアだけ差し替え可能)

同一インターフェース(`BeltSim`)で 2 実装。空力・ばね配線・ベルト送り・判定・描画は共通。

1. **torch**(既定, `ragsim/sim.py`) — 自前バッチ物理。陽的(半陰的オイラー)+ 押し出し接触。
   `torch.compile`(CUDA Graphs)で高速化。
2. **newton**(`ragsim/sim_newton.py`) — **NVIDIA Newton(旧 warp.sim 後継)の XPBD ソルバ**で
   ばね拘束・粒子↔形状接触を解く「ライブラリ品質」の接触。`self.pos/vel` を Newton の状態バッファへの
   torch ビューに差し替えるだけで、空力等の共通ロジックをそのまま再利用。

**精度の突き合わせ**(命中率 / 中心ズレ):

| 標的 | torch | newton (XPBD) | 一致 |
|---|---|---|---|
| バケツ2.4m | 100% / 0.027m | 100% / 0.023m | ◎ |
| 机4.8m | 100% / 0.12m | 100% / 0.10m | ◎ |
| 旗3.9m(掛かり) | 12〜23% | **~0%** | ✗ |

バケツ・机は両エンジンで一致。**旗だけは Newton の 0% が非物理だと実証済み**
(`scripts/test_drape_static.py`: 完全均衡のU字ドレープ(初速0)ですら Newton は滑り落とす。
原因は XPBD の粒子接触摩擦が「摩擦予算∝貫入深さ」で静止接触の静摩擦がほぼ消える実装上の欠陥。
torch 版は不均衡+8cmまで保持=キャプスタン効果として妥当)。**旗の数値は torch 版が正**:
下降当て(頂点を棒の手前に置き上から被せる) v8.25/仰67° → **37%**(頂点前当て62°の23%から改善)。
バケツ/机は静摩擦非依存なので Newton は高速検証用として有効。

## 使い方

```bash
# パラメータ総当たり最適化(速度×仰角[×スピン差] × 試投, 全部1バッチ)
.venv/bin/python scripts/optimize.py bucket|desk|flag [trials] [torch|newton]
#   → out/opt-<target>[-newton].json

# 3パネル比較動画(N投の命中率 + 等倍/低速 × 成功例/失敗例 の4セグメント)
#   各パネル右上に「雑巾拡大ビュー(重心追従)」で姿勢・変形を表示
.venv/bin/python scripts/video.py [N] [torch|newton]
#   → out/gpu-belt-3targets[-newton].mp4
```

## 構成

- `ragsim/mesh.py` — メッシュトポロジー・ばね接続・装填形状(numpy)
- `ragsim/sim.py` — BeltSim: ベルト送り→自由飛行→静定→判定 のバッチループ(torch/CUDA)
- `ragsim/sim_newton.py` — NewtonBeltSim: 接触/ばねを Newton XPBD に委譲(BeltSim を継承)
- `ragsim/targets.py` — バケツ/机/旗コライダー(バッチSDF接触)+判定
- `ragsim/render.py` — 3D透視投影+厚みスラブ描画(PIL, rag-draw.ts 移植)
- `scripts/optimize.py` / `scripts/video.py` / `scripts/test_newton.py`(検証)

環境: `uv venv --python 3.12 && uv pip install torch numpy pillow newton --index-url https://download.pytorch.org/whl/cu130 --extra-index-url https://pypi.org/simple`
(newton は `uv pip install newton` で PyPI から。Warp 1.14 + Newton 1.3, Blackwell sm_120 で動作)

注意: CPU(Rapier陰的)とGPU(陽的)は積分器が違うため、高速平射(机)では布のばたつき方が変わり
最適パラメータが異なる。絶対値は8月の実射で較正する前提(CPU版と同じ)。
