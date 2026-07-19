/** シミュレーションパラメータ (strategy.md §4 の設計値と対応させる) */

export interface RagPhys {
  /** 質量 kg */
  m: number;
  /** 抗力係数まとめ k = 0.5·ρ·Cd·A_eff [kg/m] : a_drag = -(k/m)|v|v */
  k: number;
  /** バックスピン揚力係数 [1/s]: a_up = lift × (水平速度)。回転安定飛行の滑空成分 */
  lift: number;
}

export interface SimParams {
  drive: {
    vmax: number; // m/s
    acc: number; // m/s^2
  };
  /** 独立ステア(赤)の駆動性能。メカナムと出せる速度/加速度が違うため個別に持つ。 */
  swerveDrive: {
    vmax: number;
    acc: number;
  };
  /** 経路追従の制御ゲイン (ヘッドレス計測で調整する対象)。 */
  follow: {
    /** クロストラック(横ズレ)P ゲイン */
    crossP: number;
    /** クロストラック D ゲイン (経路へ向かう速度でダンピング) */
    crossD: number;
    /** コーナー通過に使える横加速度の割合 (× acc) */
    safeAccF: number;
    /** ルックアヘッド弧長 = clamp(0.28 + 速度×lookaheadK, 0.28, 0.85) */
    lookaheadK: number;
  };
  wheel: { r: number; k: number }; // メカナム: 半径, (a+b)/2
  turret: {
    muzzleY: number;
    yawRate: number; // rad/s
    settleYaw: number; // rad
  };
  shooter: {
    aim: number; // 照準+整定 s
    feed: number; // 送給 s
    recover: number; // s
    spinupRpmPerSec: number;
    rollerDia: number; // m
    /** 1射ごとの射出機構ジャム確率。表示シムのみで使う。 */
    jamProb: number;
  };
  pickup: { dur: number; superDur: number };
  probs: {
    flag: number;
    flagLob: number; // ブロック越しロブ
    bucketStill: number;
    bucketMove: number;
    desk: number;
    fixedB: number;
  };
  rag: RagPhys;
  superRag: RagPhys;
  bucketTopY: { blue: number; red: number };
  defense: {
    /** 任意の防御微動。通常は射撃中の姿勢を止めるため false。 */
    microMove: boolean;
  };
  /** 旗の目安容量 (ソルバが「この辺で価値が落ちる」と見なすソフト基準。ハード上限ではない)。 */
  flagCapacity: number;
  /**
   * 旗の雑巾が落ちる確率(1枚あたり毎秒)。実効落下率 = flagFallPerSec × 現在の枚数。
   * 枚数が多いほど落ちやすく、1枚でもまれに落ちる (ハード上限で3枚固定にはしない)。
   */
  flagFallPerSec: number;
  /** 床の雑巾を踏むとスリップ/絡まり(→リトライ)が起きるハザードを有効にするか。 */
  ragHazard: boolean;
  /** 経路生成で床の雑巾を避けるか (既定 false)。 */
  avoidFloorRags: boolean;
  /**
   * 投擲の角度ばらつき(度, 1σ)。距離依存の命中率に効く: 標的での横ズレ ≈ 距離×この角度。
   * 小さい標的(バケツ)ほど、遠いほど当たりにくくなる。
   */
  aimSpreadDeg: number;
}

export const DEFAULT_PARAMS: SimParams = {
  drive: { vmax: 2.0, acc: 1.6 },
  swerveDrive: { vmax: 2.4, acc: 1.3 },
  follow: { crossP: 22, crossD: 2, safeAccF: 0.6, lookaheadK: 0.16 },
  wheel: { r: 0.0635, k: 0.5 },
  turret: { muzzleY: 1.5, yawRate: 3.5, settleYaw: 0.02 },
  shooter: {
    aim: 1.5,
    feed: 0.9,
    recover: 0.6,
    spinupRpmPerSec: 2500,
    rollerDia: 0.1,
    jamProb: 0.1,
  },
  pickup: { dur: 8, superDur: 6 },
  probs: {
    flag: 0.72,
    flagLob: 0.5,
    bucketStill: 0.5,
    bucketMove: 0.15,
    desk: 0.95,
    fixedB: 0.92,
  },
  // 雑巾 200×300mm 48g: バックスピン安定時のエッジ寄り実効面積を仮定
  rag: { m: 0.048, k: 0.012, lift: 0.55 },
  // スーパー雑巾 400×600mm 140g
  superRag: { m: 0.14, k: 0.032, lift: 0.35 },
  bucketTopY: { blue: 2.0, red: 1.5 },
  defense: { microMove: false },
  flagCapacity: 3,
  flagFallPerSec: 0.02,
  ragHazard: true,
  avoidFloorRags: true,
  aimSpreadDeg: 2.8,
};

export type OppArchetype = 'manual' | 'auto' | 'blocker';
export type StrategyVariant = 'standard' | 'flagOnly' | 'bucketFocus' | 'fixedOnly' | 'optimal';

export const ARCHETYPE_LABEL: Record<OppArchetype, string> = {
  manual: '手動装填チーム',
  auto: '自動装填チーム(同型)',
  blocker: '旗ブロックチーム',
};

export function clonedParams(): SimParams {
  return JSON.parse(JSON.stringify(DEFAULT_PARAMS)) as SimParams;
}
