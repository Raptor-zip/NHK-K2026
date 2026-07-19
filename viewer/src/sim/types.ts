import type { GoalKey, TeamId, ThrowTargetKey, Vec2 } from '../config/field';

export type ControlMode = 'auto' | 'semi' | 'tps' | 'fps' | 'player' | 'crane' | 'multicam' | 'ragcam';

export type ControlCommand =
  | 'halt'
  | 'retry'
  | 'resup'
  | 'throwFlag'
  | 'throwB1'
  | 'throwB2'
  | 'throwB3'
  | 'throwDesk1'
  | 'throwDesk2'
  | 'throwMoving';

/** 行動キューの1要素。'smart' はプレーコール相当 (相手静止なら移動バケツ、通常は旗) */
export type ThrowGoal = ThrowTargetKey | 'smart';

export type Action =
  | { t: 'goto'; x: number; z: number; speed?: number; for?: ThrowTargetKey }
  | { t: 'wait'; dur: number; label?: string }
  | { t: 'waitUntil'; time: number; label?: string }
  | { t: 'pickup'; n: number; dur: number; superRag?: boolean }
  | { t: 'throw'; goal: ThrowGoal; superRag?: boolean; count?: number }
  | { t: 'status'; text: string }
  | { t: 'call'; text: string }
  | { t: 'power'; on: boolean }
  | { t: 'replan' }; // 最適戦略: ライブ状態で投擲計画を都度再計算し、投擲キューを生成する

export type RetryPhase = 'declared' | 'approach' | 'lift' | 'carry' | 'repair' | 'homing';

export interface RetryState {
  /** 競技用品接触などの強制リトライ (宣言後15秒停止) */
  forced?: boolean;
  phase: RetryPhase;
  t: number;
  from: { x: number; z: number; theta: number };
  /** 搬送経路 (A* で競技物品を避けてスタートゾーンへ運ぶ) と総弧長 */
  carryPath?: Vec2[];
  carryLen?: number;
}

export interface AimSpec {
  target: { x: number; y: number; z: number };
  goalKey: ThrowTargetKey;
  speed: number;
  angleRad: number;
  tof: number;
  prob: number;
  /** null = 物理判定弾 (推定座標狙いの移動バケツ・手動射出) */
  outcome: boolean | null;
  superRag: boolean;
  lob: boolean;
  fired: boolean;
  shotPose?: { x: number; z: number; cost: number; dist: number; prob: number };
}

export interface RobotState {
  team: TeamId;
  x: number;
  z: number;
  theta: number; // 前方 = (sinθ, cosθ)
  vx: number;
  vz: number;
  omega: number;
  thetaTarget: number;
  path: Vec2[] | null;
  pathIdx: number;
  turretYaw: number; // 車体相対
  turretPitch: number;
  rollerRpm: number;
  rollerTargetRpm: number;
  powered: boolean;
  ammo: number;
  superAmmo: number;
  status: string;
  cycleScale: number;
  probMul: number;
  driveMul: number;
  muzzleY: number;
  bucketTopY: number;
  liftY: number;
  /** 被弾などによるジャム率の恒久加算 (Q15) */
  jamBoost: number;
  /** 砲塔の躍度制御用の角速度状態 (Q20) */
  turretYawVel: number;
  turretPitchVel: number;
  queue: Action[];
  cur: Action | null;
  actT: number;
  throwsLeft: number;
  anim: {
    feed: number; // 0..1 送給中
    grab: number; // 0..1 グラバー動作中
    wheels: [number, number, number, number]; // 積算回転角 rad
    wheelOmega: [number, number, number, number];
    strafe: number;
  };
  aim: AimSpec | null;
  /** 独立ステア(スワーブ)か。true=モジュール方位を操舵レートで向けてから駆動 (赤) */
  swerve: boolean;
  /** スワーブのモジュール方位 (ワールド角, 前方=(sinθ,cosθ)規約) */
  moduleDir: number;
  /** 直近に射出した弾の狙い (自動/半自動モードの命中確率表示用) */
  lastShot: { key: ThrowTargetKey; prob: number; t: number } | null;
  collided: boolean;
  /** 競技用品への押し込み継続時間 (瞬間的な擦りで強制リトライしないためのデバウンス) */
  banContactT: number;
  /** 床の雑巾を踏み続けた時間 (絡まりハザード用) */
  ragDragT: number;
  /** 経路追従で正味の前進が無い時間 (スタック検出→再計画/タイムアウト用) */
  stallT: number;
  /** 現在の goto でゴールへ最も近づけた距離 (正味の前進判定用) */
  stallBestD: number;
  /** 到達時に機体を向ける方位 (射点で標的へ正対など)。null=進行方向を向く。 */
  arriveTheta: number | null;
  /** 経路の残弧長 (スタック判定の進捗指標。直線距離だと迂回路で誤検知するため) */
  pathRemain: number;
  retry: RetryState | null;
}

export interface Projectile {
  id: number;
  team: TeamId;
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  spin: number;
  superRag: boolean;
  /** null = 手動射出 (事前判定なし・物理判定で決まる) */
  outcome: boolean | null;
  goalKey: ThrowTargetKey;
  target: { x: number; y: number; z: number };
  t: number;
  tof: number;
  landed: boolean;
  /** 空力個体差 (抗力/揚力の倍率)。布の変形・回転ばらつきの簡易表現 */
  aeroK?: number;
  aeroL?: number;
  /** 事前計算した命中確率 (自動/半自動)。手動弾は未定義 */
  prob?: number;
}

export interface HungRag {
  id: number;
  x: number;
  y: number;
  z: number;
  attachedTo?: TeamId;
  localX?: number;
  localY?: number;
  localZ?: number;
  yaw: number;
  kind: 'bar' | 'floor' | 'shelf' | 'bucket';
  superRag: boolean;
  team: TeamId;
}

export interface MatchEvent {
  t: number;
  text: string;
  cls: 'hit' | 'miss' | 'ev' | 'opp' | 'sys';
}

export interface Breakdown {
  moving: number;
  flag: number;
  desk: number;
  fixed: number;
  superHits: number;
  bonus: boolean;
}

export function emptyBreakdown(): Breakdown {
  return { moving: 0, flag: 0, desk: 0, fixed: 0, superHits: 0, bonus: false };
}

export type ScoredMap = Record<GoalKey, number>;

export function emptyScored(): ScoredMap {
  return { flag: 0, b1: 0, b2: 0, b3: 0, desk1: 0, desk2: 0 };
}
