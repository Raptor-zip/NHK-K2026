import {
  POINTS,
  GOAL_LABEL,
  goalAimPoint,
  waypoints,
  FIELD,
  RED_SIDE,
  DIMS,
  mirror,
  FLAG_CLOTH_DIR,
  type GoalKey,
  type TeamId,
  type ThrowTargetKey,
  type Vec2,
} from '../config/field';
import {
  DEFAULT_PARAMS,
  type OppArchetype,
  type SimParams,
  type StrategyVariant,
} from '../config/params';
import { mulberry32, type Rng } from './rng';
import { solveShot } from './ballistics';
import { wheelOmegas, strafeSpeed } from './mecanum';
import { castScan, getUpperSegs, type LidarScan } from './lidar';
import { Localizer, OpponentTracker } from './estimator';
import { chassisObstacles, pointBlocked, resolveCircle, ROBOT_R } from './collision';
import { planPath } from './pathfind';
import { buildBlueScript, buildRedScript, initialPose } from './scripts';
import { planOptimal, type OptimalContext } from './optimal';
import {
  emptyBreakdown,
  emptyScored,
  type Action,
  type AimSpec,
  type Breakdown,
  type ControlCommand,
  type ControlMode,
  type HungRag,
  type MatchEvent,
  type Projectile,
  type RetryState,
  type RobotState,
  type ScoredMap,
} from './types';

export interface MatchOptions {
  seed?: number;
  archetype?: OppArchetype;
  variant?: StrategyVariant;
  redVariant?: StrategyVariant;
  headless?: boolean;
  params?: SimParams;
  /** 相手の腕前 0..1 */
  oppSkill?: number;
  blueCycleScale?: number;
  redCycleScale?: number;
  blueProbMul?: number;
  redProbMul?: number;
  blueDriveMul?: number;
  redDriveMul?: number;
  blueMuzzleY?: number;
  redMuzzleY?: number;
  /** 足回り: true=独立ステア(独ステ), false=メカナム。既定は青メカナム/赤独ステ */
  blueSwerve?: boolean;
  redSwerve?: boolean;
  /** auto=全自動, semi=ボタン指令のみ, tps/fps=直接操縦。auto以外は操作チームの自動スクリプトを無効化 */
  controlMode?: ControlMode;
  /** UI/手動操作/自己位置推定の対象チーム */
  playerTeam?: TeamId;
}

interface PhysGoal {
  key: ThrowTargetKey;
  /** ゴールが属する陣地 (このゴールに入れると相手陣側チームの得点) */
  side: TeamId;
  kind: 'flag' | 'bucket' | 'desk';
  x: number;
  z: number;
  topY: number;
  r: number;
}

const MATCH_LEN = 180;
const BUZZERS = [30, 60, 120, 150] as const;
const PROJ_DT = 1 / 120;

function wrapAngle(a: number): number {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

function teamLabel(team: TeamId): string {
  // ログ幅を節約するため1文字表記 (青/赤)
  return team === 'blue' ? '青' : '赤';
}

export class MatchSim {
  readonly par: SimParams;
  readonly rng: Rng;
  readonly archetype: OppArchetype;
  readonly variant: StrategyVariant;
  readonly redVariant: StrategyVariant | null;
  readonly headless: boolean;

  t = 0;
  over = false;
  score: Record<TeamId, number> = { blue: 0, red: 0 };
  breakdown: Record<TeamId, Breakdown> = { blue: emptyBreakdown(), red: emptyBreakdown() };
  scored: Record<TeamId, ScoredMap> = { blue: emptyScored(), red: emptyScored() };
  /** 補充スポット上の雑巾残数 (描画用) */
  spotStock: Record<TeamId, number> = { blue: 10, red: 10 };
  events: MatchEvent[] = [];
  hung: HungRag[] = [];
  projectiles: Projectile[] = [];
  blue: RobotState;
  red: RobotState;
  lastScan: LidarScan | null = null;
  /** 下段後ろ向きLiDAR (前後2台構成: 360°拘束でICPが縮退しない) */
  lastScanRear: LidarScan | null = null;
  /** 上段LiDAR (0.5m, 相手検出用)。下段では教壇に遮られ相手が見えないため別段 */
  lastUpperScan: LidarScan | null = null;
  /** 本物の自己位置推定 (計測輪オドメトリ+ICP)。表示値はすべてここから */
  readonly localizer: Localizer;
  readonly oppTracker = new OpponentTracker();
  // 操縦モードは試合リセット無しで切り替えられる (setControlMode が手動↔自動の遷移を処理)
  controlMode: ControlMode;
  readonly playerTeam: TeamId;
  manual: {
    f: number;
    s: number;
    rot: number;
    aimYaw: number;
    aimPitch: number;
    power: number;
    fireCooldown: number;
    /** TPS: カーソルが指す世界座標の狙い点 */
    target: { x: number; y: number; z: number } | null;
    /** TPS: 狙い点への弾道解 (毎フレーム更新)。ok=false は射程外 */
    sol: { ok: boolean; speed: number; angle: number };
  } = {
    f: 0,
    s: 0,
    rot: 0,
    aimYaw: Math.PI,
    aimPitch: 0.55,
    power: 9,
    fireCooldown: 0,
    target: null,
    sol: { ok: false, speed: 9, angle: 0.8 },
  };

  private buzzed = new Set<number>();
  private nextId = 1;
  private scanTimer = 0;
  private physGoals: PhysGoal[] = [];
  timeExpired = false;
  /** 操作側の瞬間加速度(m/s^2)と直前速度 (計測HUD用) */
  private pvx = 0;
  private pvz = 0;
  playerAccel = 0;

  constructor(opts: MatchOptions = {}) {
    this.par = opts.params ?? DEFAULT_PARAMS;
    this.rng = mulberry32(opts.seed ?? 1);
    this.archetype = opts.archetype ?? 'manual';
    this.variant = opts.variant ?? 'standard';
    this.redVariant = opts.redVariant ?? null;
    this.headless = opts.headless ?? false;
    this.blue = this.makeRobot(
      'blue',
      buildBlueScript(this.variant, this.par),
      opts.blueCycleScale ?? 1,
      opts.blueProbMul ?? 1,
      opts.blueDriveMul ?? 1,
      opts.blueMuzzleY ?? this.par.turret.muzzleY,
      opts.blueSwerve ?? false,
    );
    const skill = opts.oppSkill ?? (this.redVariant ? 0.85 : this.archetype === 'auto' ? 0.85 : 0.7);
    const cyc = opts.redCycleScale ?? (this.redVariant ? 1.15 : this.archetype === 'auto' ? 1.15 : 2.0);
    this.red = this.makeRobot(
      'red',
      buildRedScript(this.redVariant ?? this.archetype, this.par),
      cyc,
      opts.redProbMul ?? skill,
      opts.redDriveMul ?? 1,
      opts.redMuzzleY ?? 1.2,
      opts.redSwerve ?? true,
    );
    this.controlMode = opts.controlMode ?? 'auto';
    this.playerTeam = opts.playerTeam ?? 'blue';
    this.manual.aimYaw = initialPose(this.playerTeam).theta;
    if (this.playerManual) {
      const r = this.player;
      r.queue = [];
      r.status =
        this.controlMode === 'semi'
          ? '半自動待機'
          : this.controlMode === 'fps'
            ? 'FPS操縦'
            : 'TPS操縦';
    }
    this.localizer = new Localizer(initialPose(this.playerTeam));
    this.buildPhysGoals();
    this.event('sys', '競技スタート');
  }

  get player(): RobotState {
    return this.playerTeam === 'blue' ? this.blue : this.red;
  }

  /**
   * プレイヤーがロボットを直接操縦するモードか (青の自動スクリプトを止める対象)。
   * semi/tps/fps のみ該当。crane/multicam/ragcam/player は視点/選手操作だけなので、
   * ロボット自体は auto と同じく戦略スクリプトで動かす。
   */
  get playerManual(): boolean {
    return this.controlMode === 'semi' || this.controlMode === 'tps' || this.controlMode === 'fps';
  }

  /**
   * 操縦モードを試合リセット無しで切り替える。手動(semi/tps/fps)↔自動/ビューの
   * 切り替わりのときだけ、操作チームのスクリプトを止める/組み直す。ビュー間・自動間の
   * 切替はスクリプトに触れないので試合はそのまま継続する。
   */
  setControlMode(mode: ControlMode): void {
    if (mode === this.controlMode) return;
    const wasManual = this.playerManual;
    this.controlMode = mode;
    const nowManual = this.playerManual;
    if (nowManual === wasManual) return;
    const r = this.player;
    r.cur = null;
    r.aim = null;
    r.throwsLeft = 0;
    if (nowManual) {
      r.queue = [];
      r.status = mode === 'semi' ? '半自動待機' : mode === 'fps' ? 'FPS操縦' : 'TPS操縦';
    } else {
      // 自動へ復帰: 戦略スクリプトを組み直して現在地から継続
      r.queue =
        r.team === 'blue'
          ? buildBlueScript(this.variant, this.par)
          : buildRedScript(this.redVariant ?? this.archetype, this.par);
      r.status = '';
    }
  }

  get opponent(): RobotState {
    return this.other(this.playerTeam);
  }

  private buildPhysGoals(): void {
    for (const side of ['red', 'blue'] as const) {
      const S = (p: Vec2): Vec2 => (side === 'red' ? p : mirror(p));
      const P = RED_SIDE;
      this.physGoals.push(
        // 旗: 横棒はポールから片持ち (赤=+x/青=-x)。物理判定は棒の中点基準
        { key: 'flag', side, kind: 'flag', x: P.flag.x + FLAG_CLOTH_DIR[side] * 0.31, z: S(P.flag).z, topY: DIMS.flagBarY, r: 0.32 },
        { key: 'b1', side, kind: 'bucket', ...S(P.b1), topY: DIMS.bucketH, r: DIMS.bucketRimR },
        { key: 'b2', side, kind: 'bucket', ...S(P.b2), topY: 0.6 + DIMS.bucketH, r: DIMS.bucketRimR },
        { key: 'b3', side, kind: 'bucket', ...S(P.b3), topY: 0.3 + DIMS.bucketH, r: DIMS.bucketRimR },
        { key: 'desk1', side, kind: 'desk', ...S(P.desk1), topY: DIMS.desk.shelfY, r: 0.3 },
        { key: 'desk2', side, kind: 'desk', ...S(P.desk2), topY: DIMS.desk.shelfY, r: 0.3 },
      );
    }
  }

  private makeRobot(
    team: TeamId,
    queue: Action[],
    cycleScale: number,
    probMul: number,
    driveMul: number,
    muzzleY: number,
    swerve: boolean,
  ): RobotState {
    const pose = initialPose(team);
    return {
      team,
      ...pose,
      vx: 0,
      vz: 0,
      omega: 0,
      thetaTarget: pose.theta,
      path: null,
      pathIdx: 0,
      turretYaw: 0,
      turretPitch: 0.4,
      rollerRpm: 0,
      rollerTargetRpm: 0,
      powered: true,
      ammo: 0,
      superAmmo: 0,
      status: '待機',
      cycleScale,
      probMul,
      driveMul,
      muzzleY,
      bucketTopY: this.par.bucketTopY[team],
      liftY: 0,
      jamBoost: 0,
      turretYawVel: 0,
      turretPitchVel: 0,
      queue: [...queue],
      cur: null,
      actT: 0,
      throwsLeft: 0,
      anim: { feed: -1, grab: -1, wheels: [0, 0, 0, 0], wheelOmega: [0, 0, 0, 0], strafe: 0 },
      aim: null,
      swerve,
      moduleDir: pose.theta,
      lastShot: null,
      collided: false,
      banContactT: 0,
      ragDragT: 0,
      stallT: 0,
      stallBestD: Infinity,
      arriveTheta: null,
      pathRemain: Infinity,
      retry: null,
    };
  }

  private event(cls: MatchEvent['cls'], text: string): void {
    this.events.push({ t: this.t, text, cls });
  }

  private other(team: TeamId): RobotState {
    return team === 'blue' ? this.red : this.blue;
  }

  private movingBucketAimPoint(r: RobotState): { x: number; y: number; z: number } {
    // RobotVisual と同じ局所配置。移動バケツは車体中心ではなくマスト側にある。
    const bx = 0.24;
    const bz = -0.24;
    const c = Math.cos(r.theta);
    const s = Math.sin(r.theta);
    return {
      x: r.x + bx * c + bz * s,
      y: r.bucketTopY + 0.02,
      z: r.z - bx * s + bz * c,
    };
  }

  // ---------------------------------------------------------------- scoring

  private addScore(team: TeamId, goalKey: ThrowTargetKey, superRag: boolean, prob?: number): void {
    const pts = POINTS[goalKey] * (superRag ? 2 : 1);
    this.score[team] += pts;
    const bd = this.breakdown[team];
    if (goalKey === 'moving') bd.moving++;
    else if (goalKey === 'flag') bd.flag++;
    else if (goalKey === 'desk1' || goalKey === 'desk2') bd.desk++;
    else bd.fixed++;
    if (superRag) bd.superHits++;
    if (goalKey !== 'moving') this.scored[team][goalKey]++;
    const who = `${teamLabel(team)}: `;
    const pTag = prob !== undefined ? ` (狙い${Math.round(prob * 100)}%)` : '';
    this.event(
      team === this.playerTeam ? 'hit' : 'opp',
      `${who}${GOAL_LABEL[goalKey]}${superRag ? '(スーパー)' : ''} +${pts}${pTag}`,
    );
    // 全固定ゴール制覇ボーナス
    const s = this.scored[team];
    if (!bd.bonus && s.flag > 0 && s.b1 > 0 && s.b2 > 0 && s.b3 > 0 && s.desk1 > 0 && s.desk2 > 0) {
      bd.bonus = true;
      this.score[team] += 100;
      this.event(team === this.playerTeam ? 'ev' : 'opp', `${who}全固定ゴール制覇ボーナス +100`);
    }
    // 旗の落下は確率的 (stepFlagFall) に任せる — ハード容量での即時押し出しはしない
  }

  /** 旗の容量制限: 掛けすぎると最古の雑巾が押し出されて床に落ち、その掛かり得点は失われる。 */
  /** 旗の雑巾を1枚落下させる (床へ移し、得点・カウンタを戻す) */
  private dropFlagRag(team: TeamId, rag: HungRag): void {
    rag.kind = 'floor';
    rag.y = 0.012;
    rag.x += (this.rng() - 0.5) * 0.35;
    rag.z += (this.rng() - 0.5) * 0.18;
    const val = rag.superRag ? POINTS.flag * 2 : POINTS.flag;
    this.score[team] -= val;
    const bd = this.breakdown[team];
    bd.flag = Math.max(0, bd.flag - 1);
    if (rag.superRag) bd.superHits = Math.max(0, bd.superHits - 1);
    // 飽和カウンタも戻す (ただしボーナス維持のため最低1は残す)
    this.scored[team].flag = Math.max(1, this.scored[team].flag - 1);
    this.event(
      team === this.playerTeam ? 'miss' : 'opp',
      `${teamLabel(team)}: 旗の雑巾が1枚ずり落ちた (-${val})`,
    );
  }

  /**
   * 旗の雑巾の確率的落下 (毎フレーム)。1枚あたりの落下確率/秒 = flagFallPerSec × 枚数。
   * 枚数が多いほど落ちやすく、1枚でもまれに落ちる。ハード上限で3枚固定にはしない。
   */
  private stepFlagFall(dt: number): void {
    const base = this.par.flagFallPerSec;
    if (base <= 0) return;
    for (const team of ['blue', 'red'] as const) {
      const bars = this.hung.filter((h) => h.kind === 'bar' && h.team === team);
      const n = bars.length;
      if (n === 0) continue;
      const pPer = base * n * dt; // 1枚あたりこのフレームで落ちる確率
      for (const rag of bars) {
        if (this.rng() < pPer) this.dropFlagRag(team, rag);
      }
    }
  }

  // ---------------------------------------------------------------- throwing

  /** 旗の重ね掛け飽和: 掛かっている枚数が増えるほど新規に安定して掛かる確率が落ちる */
  private flagSaturation(team: TeamId): number {
    return Math.max(0.45, 1 - 0.02 * this.scored[team].flag);
  }

  /** 相手車体が旗への射線コリドーを塞いでいるか */
  private flagBlocked(r: RobotState): boolean {
    const target = goalAimPoint(r.team, 'flag');
    const opp = this.other(r.team);
    const dx = target.x - r.x;
    const dz = target.z - r.z;
    const len2 = dx * dx + dz * dz;
    const px = opp.x - r.x;
    const pz = opp.z - r.z;
    const s = Math.max(0, Math.min(1, (px * dx + pz * dz) / len2));
    const dist = Math.hypot(px - dx * s, pz - dz * s);
    return dist < 0.6 && s > 0.15 && s < 0.95;
  }

  private resolveThrowSpec(r: RobotState, act: Extract<Action, { t: 'throw' }>): AimSpec | null {
    const superRag = act.superRag ?? false;
    if (superRag ? r.superAmmo <= 0 : r.ammo <= 0) {
      this.event('sys', `${teamLabel(r.team)}: 弾切れ`);
      return null;
    }
    const opp = this.other(r.team);
    const p = this.par.probs;
    let goal: ThrowTargetKey;
    if (act.goal === 'smart') {
      // プレーコール「バケット」: 相手が電源OFF (手動装填) or 旗ブロック中の静止時のみ、
      // かつ期待値が旗 (飽和込み) を上回る場合に限って移動バケツへ切替える
      const loadingWindow = !opp.powered;
      const oppStill = loadingWindow || Math.hypot(opp.vx, opp.vz) < 0.05;
      const flagEff = p.flag * this.flagSaturation(r.team);
      const blocked = this.flagBlocked(r);
      if (loadingWindow && p.bucketStill > flagEff) goal = 'moving';
      else if (blocked && oppStill && p.bucketStill > Math.max(p.flagLob, flagEff * 0.8)) goal = 'moving';
      else goal = 'flag';
    } else {
      goal = act.goal;
    }
    // Q24: 操作側の移動バケツ狙いは「LiDAR推定位置」を使う (真値は使わない)。
    // 事前抽選もやめて物理判定 — 推定誤差がそのまま命中率に効く。
    let physical = false;
    let target: { x: number; y: number; z: number };
    if (goal === 'moving' && r === this.player) {
      const est = this.oppTracker.est;
      if (!est) {
        this.event('sys', '相手位置が未推定 (上段LiDARの視野外) — バケツ射撃を保留');
        return null;
      }
      target = { x: est.x, y: opp.bucketTopY + 0.02, z: est.z };
      physical = true;
    } else {
      target = goal === 'moving' ? this.movingBucketAimPoint(opp) : goalAimPoint(r.team, goal);
    }

    const lob = goal === 'flag' && this.flagBlocked(r);

    // 砲口は車体中心から前方 MUZZLE_FWD にある → 弾道は砲口からの距離で解く
    const MUZZLE_FWD = 0.4;
    const mx = r.x;
    const mz = r.z;
    const d = Math.max(0.3, Math.hypot(target.x - mx, target.z - mz) - MUZZLE_FWD);
    const dy = target.y - r.muzzleY;

    // 距離依存の命中率 (物理モデル): 標的での横ズレ σ ≈ 射程 × 投擲の角度ばらつき。
    // これが標的の実効半幅 halfW を食う。小さい標的(バケツ)ほど・遠いほど急に当たらなくなる。
    // 雑巾投擲は距離の影響が大きい、という指摘を反映 (旧 distF はほぼ距離無依存だった)。
    const sigmaAng = (this.par.aimSpreadDeg * Math.PI) / 180;
    const range = Math.hypot(d, dy); // 3D 射程 (旗は高所なので dy も効く)
    const halfW = goal === 'flag' ? 0.3 : goal === 'desk1' || goal === 'desk2' ? 0.32 : 0.14;
    const distFactor = Math.exp(-0.5 * ((range * sigmaAng) / halfW) ** 2);
    let prob: number;
    switch (goal) {
      case 'flag': {
        prob = (lob ? p.flagLob : p.flag) * this.flagSaturation(r.team) * distFactor;
        break;
      }
      case 'moving': {
        const oppStill = !opp.powered || Math.hypot(opp.vx, opp.vz) < 0.05;
        prob = (oppStill ? p.bucketStill : p.bucketMove) * distFactor;
        break;
      }
      case 'desk1':
      case 'desk2':
        prob = p.desk * distFactor;
        break;
      default:
        prob = p.fixedB * distFactor;
    }
    prob = Math.min(0.98, prob * r.probMul);
    const rag = superRag ? this.par.superRag : this.par.rag;
    const sol = solveShot(d, dy, rag, lob);
    const speed = sol?.speed ?? 8;
    const angleRad = sol?.angleRad ?? (55 * Math.PI) / 180;
    const tof = sol?.tof ?? 1.0;
    return { target, goalKey: goal, speed, angleRad, tof, prob, outcome: physical ? null : false, superRag, lob, fired: false };
  }

  /** 位置 (x,z) から静的ゴール goal を撃ったときの命中率 (射程外は0)。射点最適化の評価用。 */
  private shotProbAt(r: RobotState, x: number, z: number, goal: ThrowTargetKey): number {
    if (goal === 'moving') return 0;
    const target = goalAimPoint(r.team, goal);
    const MUZZLE_FWD = 0.4;
    const d = Math.max(0.3, Math.hypot(target.x - x, target.z - z) - MUZZLE_FWD);
    const dy = target.y - r.muzzleY;
    const lob = goal === 'flag' && this.flagBlocked(r);
    if (!solveShot(d, dy, this.par.rag, lob)) return 0; // 射程外
    const sigmaAng = (this.par.aimSpreadDeg * Math.PI) / 180;
    const range = Math.hypot(d, dy);
    const halfW = goal === 'flag' ? 0.3 : goal === 'desk1' || goal === 'desk2' ? 0.32 : 0.14;
    const distFactor = Math.exp(-0.5 * ((range * sigmaAng) / halfW) ** 2);
    const p = this.par.probs;
    const base =
      goal === 'flag'
        ? p.flag * this.flagSaturation(r.team)
        : goal === 'desk1' || goal === 'desk2'
          ? p.desk
          : p.fixedB;
    return Math.min(0.98, base * distFactor * r.probMul);
  }

  /**
   * 射点最適化: 割り当てられた固定射点 (destX,destZ) へ動く価値があるか判定する。
   * 現在地が射程内で、移動先の命中率が現在地より大きくは改善しない (差が小さい) なら、
   * 砲塔可動域内で狙える限りその場で撃つ → 不要な移動をなくす。移動で大きく改善するなら動く。
   */
  private shotGoodEnough(r: RobotState, goal: ThrowTargetKey, destX: number, destZ: number): boolean {
    if (goal === 'moving') return false; // 移動バケツは相手依存 → 射点最適化の対象外
    const curProb = this.shotProbAt(r, r.x, r.z, goal);
    if (curProb <= 0) return false; // 現在地は射程外 → 移動する
    const destProb = this.shotProbAt(r, destX, destZ, goal);
    if (destProb > curProb + 0.06) return false; // 移動で6%超改善するなら移動する
    const target = goalAimPoint(r.team, goal);
    const reqYaw = wrapAngle(Math.atan2(target.x - r.x, target.z - r.z) - r.theta);
    return Math.abs(reqYaw) <= 2.2; // 砲塔可動域 (±2.4) 内で届く
  }

  private fire(r: RobotState): void {
    const aim = r.aim;
    if (!aim) return;
    if (this.tryJam(r, aim.superRag)) return;
    aim.fired = true;
    // 自動/半自動: 射出時の狙い (命中確率) を記録して HUD / ログに表示
    r.lastShot = { key: aim.goalKey, prob: aim.prob, t: this.t };
    if (aim.outcome !== null) aim.outcome = this.rng() < aim.prob;
    if (aim.superRag) r.superAmmo--;
    else r.ammo--;
    const dx = aim.target.x - r.x;
    const dz = aim.target.z - r.z;
    const dHor = Math.hypot(dx, dz);
    let dirx = dx / dHor;
    let dirz = dz / dHor;
    let speed = aim.speed;
    if (aim.outcome === null) {
      // 物理判定弾: 機構分散のみ (推定誤差ぶんは狙点自体がズレている)
      const e = ((this.rng() - 0.5) * 2.4 * Math.PI) / 180;
      const c = Math.cos(e);
      const sn = Math.sin(e);
      [dirx, dirz] = [dirx * c - dirz * sn, dirx * sn + dirz * c];
      speed *= 1 + (this.rng() - 0.5) * 0.06;
    } else if (aim.outcome) {
      const e = ((this.rng() - 0.5) * 0.8 * Math.PI) / 180;
      const c = Math.cos(e);
      const s = Math.sin(e);
      [dirx, dirz] = [dirx * c - dirz * s, dirx * s + dirz * c];
      speed *= 1 + (this.rng() - 0.5) * 0.02;
    } else {
      const sign = this.rng() < 0.5 ? -1 : 1;
      const e = (sign * (2 + this.rng() * 3.5) * Math.PI) / 180;
      const c = Math.cos(e);
      const s = Math.sin(e);
      [dirx, dirz] = [dirx * c - dirz * s, dirx * s + dirz * c];
      speed *= 1 + (this.rng() - 0.5) * 0.14;
    }
    const vh = Math.cos(aim.angleRad) * speed;
    this.projectiles.push({
      id: this.nextId++,
      team: r.team,
      x: r.x + dirx * 0.4,
      y: r.muzzleY,
      z: r.z + dirz * 0.4,
      vx: dirx * vh,
      vy: Math.sin(aim.angleRad) * speed,
      vz: dirz * vh,
      spin: 18,
      superRag: aim.superRag,
      outcome: aim.outcome,
      prob: aim.prob,
      goalKey: aim.goalKey,
      target: { ...aim.target },
      t: 0,
      tof: aim.tof,
      landed: false,
      aeroK: 1 + (this.rng() - 0.5) * 0.16,
      aeroL: 1 + (this.rng() - 0.5) * 0.24,
    });
    // 射出反動でローラー回転がわずかに落ちる
    r.rollerRpm *= 0.92;
  }

  private landProjectile(pr: Projectile, hit: boolean): void {
    pr.landed = true;
    const rag = pr.superRag;
    if (hit) {
      if (pr.goalKey === 'flag') {
        this.hung.push({
          id: pr.id,
          x: pr.target.x + (this.rng() - 0.5) * 0.4,
          y: pr.target.y - 0.05,
          z: pr.target.z,
          yaw: (this.rng() - 0.5) * 0.6,
          kind: 'bar',
          superRag: rag,
          team: pr.team,
        });
      } else if (pr.goalKey === 'desk1' || pr.goalKey === 'desk2') {
        this.hung.push({
          id: pr.id,
          x: pr.target.x,
          y: 0.3,
          z: pr.target.z,
          yaw: this.rng() * Math.PI,
          kind: 'shelf',
          superRag: rag,
          team: pr.team,
        });
      } else if (pr.goalKey === 'moving') {
        const carrier = this.other(pr.team);
        const localX = 0.24 + (this.rng() - 0.5) * 0.11;
        const localY = carrier.bucketTopY - 0.11;
        const localZ = -0.24 + (this.rng() - 0.5) * 0.11;
        const c = Math.cos(carrier.theta);
        const s = Math.sin(carrier.theta);
        this.hung.push({
          id: pr.id,
          x: carrier.x + localX * c + localZ * s,
          y: localY,
          z: carrier.z - localX * s + localZ * c,
          attachedTo: carrier.team,
          localX,
          localY,
          localZ,
          yaw: this.rng() * Math.PI,
          kind: 'bucket',
          superRag: rag,
          team: pr.team,
        });
      } else {
        this.hung.push({
          id: pr.id,
          x: pr.target.x,
          y: pr.target.y - 0.1,
          z: pr.target.z,
          yaw: this.rng() * Math.PI,
          kind: 'bucket',
          superRag: rag,
          team: pr.team,
        });
      }
      this.addScore(pr.team, pr.goalKey, rag, pr.prob);
    } else {
      this.hung.push({
        id: pr.id,
        x: pr.x,
        y: 0.012,
        z: pr.z,
        yaw: this.rng() * Math.PI,
        kind: 'floor',
        superRag: rag,
        team: pr.team,
      });
      const pTag = pr.prob !== undefined ? ` (狙い${Math.round(pr.prob * 100)}%)` : '';
      this.event(
        pr.team === this.playerTeam ? 'miss' : 'opp',
        `${teamLabel(pr.team)}: ${GOAL_LABEL[pr.goalKey]} … 外れ${pTag}`,
      );
      // Q15: 雑巾がロボットの上に落ちたら巻き込みリスク → そのロボのジャム率を恒久的に大幅増
      for (const bot of [this.blue, this.red]) {
        if (Math.hypot(pr.x - bot.x, pr.z - bot.z) < 0.55) {
          bot.jamBoost += 0.3;
          this.event('ev', `${teamLabel(bot.team)}: 雑巾がロボットに直撃! 巻き込みリスク増大 (ジャム率+30%)`);
        }
      }
    }
  }

  private tryJam(r: RobotState, superRag: boolean): boolean {
    if (this.headless || r.retry) return false;
    const p = (this.par.shooter.jamProb + r.jamBoost) * (superRag ? 1.7 : 1);
    if (this.rng() >= p) return false;
    this.beginRetry(r, '射出機構ジャム');
    return true;
  }

  private beginRetry(r: RobotState, reason: string): void {
    if (r.retry) return;
    const retry: RetryState = {
      phase: 'declared',
      t: 0,
      from: { x: r.x, z: r.z, theta: r.theta },
    };
    r.retry = retry;
    r.cur = null;
    r.path = null;
    r.aim = null;
    r.throwsLeft = 0;
    r.vx = 0;
    r.vz = 0;
    r.omega = 0;
    r.powered = false;
    r.rollerTargetRpm = 0;
    r.rollerRpm *= 0.25;
    r.anim.feed = 1;
    r.status = `リトライ宣言: ${reason}`;
    this.event('ev', `${teamLabel(r.team)}: リトライ宣言 (${reason})`);
  }

  /** 折れ線経路上で、始点から弧長 dist の点を返す */
  private pointAlongPath(path: Vec2[], dist: number): Vec2 {
    if (dist <= 0 || path.length === 0) return { x: path[0]!.x, z: path[0]!.z };
    let acc = 0;
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1]!;
      const b = path[i]!;
      const seg = Math.hypot(b.x - a.x, b.z - a.z);
      if (acc + seg >= dist) {
        const t = (dist - acc) / (seg || 1e-9);
        return { x: a.x + (b.x - a.x) * t, z: a.z + (b.z - a.z) * t };
      }
      acc += seg;
    }
    const last = path[path.length - 1]!;
    return { x: last.x, z: last.z };
  }

  private setRetryPhase(r: RobotState, phase: RetryState['phase']): void {
    if (!r.retry || r.retry.phase === phase) return;
    r.retry.phase = phase;
    r.retry.t = 0;
    // 搬送開始時に、競技物品を避けてスタートゾーンへ運ぶ A* 経路を用意する
    if (phase === 'carry') {
      const start = initialPose(r.team);
      const path = planPath(r.team, { x: r.retry.from.x, z: r.retry.from.z }, { x: start.x, z: start.z });
      let len = 0;
      for (let i = 1; i < path.length; i++) {
        len += Math.hypot(path[i]!.x - path[i - 1]!.x, path[i]!.z - path[i - 1]!.z);
      }
      r.retry.carryPath = path;
      r.retry.carryLen = len;
    }
    const label =
      phase === 'approach'
        ? '選手2人がロボットへ移動'
        : phase === 'lift'
          ? 'ロボット持ち上げ'
          : phase === 'carry'
            ? 'スタートゾーンへ搬送'
            : phase === 'homing'
              ? '原点出し (非常停止解除・機構初期化)'
              : 'ジャム解除中';
    r.status = label;
    if (phase === 'lift' || phase === 'carry' || phase === 'repair' || phase === 'homing') {
      this.event('ev', `${teamLabel(r.team)}: ${label}`);
    }
  }

  private stepRetry(r: RobotState, dt: number): void {
    const retry = r.retry;
    if (!retry) return;
    retry.t += dt;
    const start = initialPose(r.team);
    r.vx = 0;
    r.vz = 0;
    r.omega = 0;
    r.path = null;
    r.rollerTargetRpm = 0;

    if (retry.phase === 'declared') {
      r.liftY = 0;
      r.status = retry.forced ? `強制リトライ: ${Math.ceil(15 - retry.t)}秒停止中` : 'リトライ宣言中';
      if (retry.t >= (retry.forced ? 15 : 1.1)) this.setRetryPhase(r, 'approach');
      return;
    }
    if (retry.phase === 'approach') {
      r.liftY = 0;
      if (retry.t >= 2.4) this.setRetryPhase(r, 'lift');
      return;
    }
    if (retry.phase === 'lift') {
      const p = clamp(retry.t / 0.9, 0, 1);
      r.liftY = p * 0.26;
      if (retry.t >= 0.9) this.setRetryPhase(r, 'carry');
      return;
    }
    if (retry.phase === 'carry') {
      const dur = 3.6;
      const p = clamp(retry.t / dur, 0, 1);
      const ease = p * p * (3 - 2 * p);
      // A* 経路に沿って弧長補間で運ぶ (競技物品を避ける)。経路が無ければ直線フォールバック。
      const path = retry.carryPath;
      const len = retry.carryLen ?? 0;
      if (path && path.length > 1 && len > 0.01) {
        const pos = this.pointAlongPath(path, ease * len);
        r.x = pos.x;
        r.z = pos.z;
      } else {
        r.x = retry.from.x + (start.x - retry.from.x) * ease;
        r.z = retry.from.z + (start.z - retry.from.z) * ease;
      }
      r.theta = retry.from.theta + wrapAngle(start.theta - retry.from.theta) * ease;
      r.thetaTarget = start.theta;
      // 持ち上げ高さ: 最後の0.7秒でゆっくり降ろす (優しく置く)
      const lowered = clamp((retry.t - (dur - 0.7)) / 0.7, 0, 1);
      r.liftY = (0.24 + Math.sin(this.t * 16) * 0.015) * (1 - lowered);
      if (retry.t >= dur) this.setRetryPhase(r, 'repair');
      return;
    }

    r.x = start.x;
    r.z = start.z;
    r.theta = start.theta;
    r.thetaTarget = start.theta;
    r.liftY = 0;
    if (retry.phase === 'repair') {
      r.anim.feed = retry.t < 1.8 ? 1 : -1;
      r.status = 'スタートゾーンでジャム解除中';
      if (retry.t >= 4.0) this.setRetryPhase(r, 'homing');
      return;
    }

    // 原点出し: 非常停止を解除し、砲塔を一度端まで振って基準を取り中心へ戻す (減衰する振り)。
    // ローラーも一度だけ回して確認する。~4秒。DJI起動音のような音は audio 側で homing を検出して鳴らす。
    r.anim.feed = -1;
    r.status = '原点出し中 (非常停止解除・機構初期化)';
    const hp = clamp(retry.t / 4, 0, 1);
    const damp = 1 - hp;
    r.turretYaw = Math.sin(hp * Math.PI * 4) * 0.55 * damp;
    r.turretPitch = 0.4 + Math.sin(hp * Math.PI * 6) * 0.14 * damp;
    r.rollerTargetRpm = retry.t < 1.4 ? 1600 : 0;
    if (retry.t >= 4.0) {
      r.retry = null;
      r.powered = true;
      r.anim.feed = -1;
      r.turretYaw = 0;
      r.rollerTargetRpm = 0;
      r.status = '復旧完了';
      this.event('sys', `${teamLabel(r.team)}: 復旧完了、競技復帰`);
      if (r === this.player && !this.headless) {
        // 持ち上げ搬送中はオドメトリ/LiDARが無効なので、既知のスタートゾーン座標で初期化
        this.localizer.reset(initialPose(r.team));
        this.event('sys', '自己位置リセット: スタートゾーンの既知座標で再初期化');
      }
      if (!this.playerManual || r !== this.player) {
        r.queue.unshift({ t: 'status', text: '戦線復帰' }, { t: 'goto', ...waypoints(r.team).fireR });
      }
    }
  }

  // ---------------------------------------------------------------- actions

  private startAction(r: RobotState, a: Action): void {
    r.actT = 0;
    switch (a.t) {
      case 'goto': {
        // 射点最適化: この移動の直後が静的ゴールへの投擲なら、現在地が既に良射点かを見て
        // 十分なら動かずその場で撃つ (固定射点への不要な移動をなくす)。a.for は明示指定。
        r.stallBestD = Infinity;
        r.stallT = 0;
        r.arriveTheta = null; // 既定は現在の向きを保持 (不要な回転をしない)。用途に応じ下で設定。
        const nextA = r.queue[0];
        const shootGoal =
          a.for ??
          (nextA && nextA.t === 'throw' && nextA.goal !== 'smart' && nextA.goal !== 'moving'
            ? nextA.goal
            : undefined);
        // 射撃前の移動: 射点で標的へ正対させる (arriveTheta)。砲塔は車体±2.4radしか回らないので、
        // 移動中から標的方位を向いておく。到達判定は角速度も見るので、標的へ向き終えるまで到達扱いにならない。
        if (shootGoal && shootGoal !== 'moving') {
          const aim = goalAimPoint(r.team, shootGoal);
          if (this.shotGoodEnough(r, shootGoal, a.x, a.z)) {
            r.thetaTarget = Math.atan2(aim.x - r.x, aim.z - r.z);
            r.arriveTheta = r.thetaTarget;
            r.path = [{ x: r.x, z: r.z }];
            r.pathIdx = 0;
            r.status = `${GOAL_LABEL[shootGoal]}を狙う (その場)`;
            break;
          }
          r.arriveTheta = Math.atan2(aim.x - a.x, aim.z - a.z);
          r.thetaTarget = r.arriveTheta; // 移動開始から回頭を始める (到達時に正対済み)
          r.status = `${GOAL_LABEL[shootGoal]}の射点へ`;
        } else if (nextA && nextA.t === 'pickup') {
          const atk = goalAimPoint(r.team, 'flag');
          r.arriveTheta = Math.atan2(atk.x - a.x, atk.z - a.z);
          r.thetaTarget = r.arriveTheta;
        } else if (nextA && nextA.t === 'throw' && nextA.goal === 'moving') {
          const opp = this.other(r.team);
          r.arriveTheta = Math.atan2(opp.x - a.x, opp.z - a.z);
          r.thetaTarget = r.arriveTheta;
        }
        const avoid = this.par.avoidFloorRags
          ? this.hung.filter((h) => h.kind === 'floor').map((h) => ({ x: h.x, z: h.z }))
          : [];
        r.path = planPath(r.team, { x: r.x, z: r.z }, { x: a.x, z: a.z }, avoid);
        r.pathIdx = 0;
        break;
      }
      case 'throw': {
        r.throwsLeft = a.count ?? 1;
        r.aim = this.resolveThrowSpec(r, a);
        // 弾切れなど恒久的に投げられないときだけキャンセル。移動バケツで相手位置未推定(est null)の
        // ときはすぐ諦めず、stepAction 側で少し待って再試行する(相手が見えたら投げる)。
        const outOfAmmo = (a.superRag ?? false) ? r.superAmmo <= 0 : r.ammo <= 0;
        if (!r.aim && outOfAmmo) r.throwsLeft = 0;
        break;
      }
      case 'pickup':
        // 補充机へ横付けし、攻撃方位を向いたまま側方グラバーで取るため車体旋回は不要
        r.status = a.superRag ? 'スーパー雑巾ピックアップ' : '一括ピックアップ';
        break;
      case 'status':
        r.status = a.text;
        break;
      case 'call':
        this.event('ev', `コール:「${a.text}」`);
        break;
      case 'power':
        r.powered = a.on;
        break;
      case 'replan':
        this.doReplan(r);
        break;
      case 'wait':
        if (a.label) r.status = a.label;
        break;
      case 'waitUntil':
        if (a.label) r.status = a.label;
        break;
      default:
        break;
    }
  }

  /** 最適戦略の射点(ゴール別) — scripts の fireOf と同じ割当 */
  private fireSpotFor(team: TeamId, g: ThrowTargetKey): { x: number; z: number } {
    const wp = waypoints(team);
    return g === 'flag' || g === 'b1'
      ? wp.fireR
      : g === 'b2'
        ? wp.fireB2
        : g === 'b3'
          ? wp.fireB3
          : g === 'desk1'
            ? wp.fireDesk1
            : wp.fireDesk2;
  }

  /** 動的ソルバへ渡すライブ状態を集める */
  private buildOptimalContext(r: RobotState): OptimalContext {
    const scored = { ...this.scored[r.team] } as Record<ThrowTargetKey, number>;
    const flagOcc = this.hung.filter((h) => h.kind === 'bar' && h.team === r.team).length;
    const shotDist = {} as Record<ThrowTargetKey, number>;
    for (const g of ['flag', 'desk1', 'desk2', 'b1', 'b2', 'b3'] as const) {
      const spot = this.fireSpotFor(r.team, g);
      const aim = goalAimPoint(r.team, g);
      shotDist[g] = Math.max(0.3, Math.hypot(aim.x - spot.x, aim.z - spot.z) - 0.4);
    }
    const cs = r.cycleScale;
    const perThrow = (this.par.shooter.aim + this.par.shooter.feed + this.par.shooter.recover) * cs;
    return {
      par: this.par,
      normalRags: r.ammo,
      superRags: r.superAmmo,
      remainingSec: Math.max(0, MATCH_LEN - this.t),
      scored,
      flagOcc,
      shotDist,
      perThrow,
    };
  }

  /**
   * 最適戦略: ライブ状態で planOptimal を呼び、「次の1投」だけをキュー先頭に置いて、その後に
   * 再び replan する。これで1投ごと(成功/失敗を反映した状態)に都度最良を選び直す。
   * 投げる対象が無ければ何も積まず、スクリプトが次(補充/待機)へ進む。
   */
  private doReplan(r: RobotState): void {
    const plan = planOptimal(this.buildOptimalContext(r));
    const superFirst = plan.superOrder.length > 0;
    const g = superFirst ? plan.superOrder[0] : plan.order[0];
    if (!g) return; // 投げるものが無い → 補充/待機へ
    r.status = `最適: ${GOAL_LABEL[g]}を狙う${superFirst ? ' (スーパー)' : ''}`;
    r.queue.unshift(
      { t: 'goto', ...this.fireSpotFor(r.team, g), for: g },
      { t: 'throw', goal: g, count: 1, superRag: superFirst },
      { t: 'replan' },
    );
  }

  private stepAction(r: RobotState, dt: number): void {
    if (!r.cur) {
      const next = r.queue.shift();
      if (!next) {
        r.status = this.over
          ? '試合終了'
          : r === this.player && this.controlMode === 'semi'
            ? '半自動待機'
            : r.status;
        return;
      }
      r.cur = next;
      this.startAction(r, next);
      // 即時完了系
      if (next.t === 'status' || next.t === 'call' || next.t === 'power' || next.t === 'replan') {
        r.cur = null;
        this.stepAction(r, dt);
        return;
      }
      if (next.t === 'throw' && r.throwsLeft <= 0) {
        r.cur = null;
        return;
      }
    }
    const a = r.cur;
    r.actT += dt;
    switch (a.t) {
      case 'goto': {
        const path = r.path ?? [{ x: a.x, z: a.z }];
        let arrived = this.followPath(r, path, dt, a.speed);
        const goal = path[path.length - 1]!;
        // 障害物に阻まれて目標点に密着できない場合は「可能な限り接近」で到達扱い
        if (!arrived && r.collided && r.actT > 1.5) {
          const d = Math.hypot(goal.x - r.x, goal.z - r.z);
          if (d < 0.95 && Math.hypot(r.vx, r.vz) < 0.35) arrived = true;
        }
        // スタック対策: 低速が続いたら一度経路を引き直す。それでも動けず時間切れなら諦めて次へ。
        // (経路追従にはまって永久停止するフリーズを防ぐ安全策)。精密な最終接近(終点付近の低速)は
        // スタック扱いしないよう、ゴールから離れているときだけ停滞を数える。
        if (!arrived) {
          // タイムアウトは「経過時間」ではなく「経路沿いに進めていない時間」で判定する。
          // 進捗指標は残弧長 pathRemain (直線距離だと迂回路がゴールから一旦離れる向きに膨らむため、
          // 経路沿いに前進していても直線距離が増え→誤ってスタック判定してしまう)。
          // 弧長が縮めばリセット → 単に長いだけ/膨らむ経路では発火しない。振動・停止は縮まないので検出できる。
          const prog = r.pathRemain;
          if (prog < r.stallBestD - 0.05) {
            r.stallBestD = prog;
            r.stallT = 0;
          } else if (prog > 0.3) {
            r.stallT += dt;
          }
          // 停滞が続く間は 1.6 秒ごとに一度、経路を引き直す (累積はリセットしない)
          if (r.stallT > 1.6 && Math.floor(r.stallT / 1.6) > Math.floor((r.stallT - dt) / 1.6)) {
            const avoid = this.par.avoidFloorRags
              ? this.hung.filter((h) => h.kind === 'floor').map((h) => ({ x: h.x, z: h.z }))
              : [];
            r.path = planPath(r.team, { x: r.x, z: r.z }, { x: a.x, z: a.z }, avoid);
            r.pathIdx = 0;
          }
          // 5 秒間ほぼ動けないまま → 本当にスタック。諦めて現在地から次の行動へ
          if (r.stallT > 5) {
            arrived = true;
            r.stallT = 0;
            this.event('sys', `${teamLabel(r.team)}: スタック検出 (5秒停滞)、その場から継続`);
          }
        } else {
          r.stallT = 0;
        }
        if (arrived) {
          r.vx = 0;
          r.vz = 0;
          r.cur = null;
          r.path = null;
        }
        break;
      }
      case 'wait':
        if (r.actT >= a.dur) r.cur = null;
        break;
      case 'waitUntil':
        if (this.t >= a.time) r.cur = null;
        break;
      case 'pickup': {
        r.anim.grab = Math.min(1, r.actT / a.dur);
        if (r.actT >= a.dur) {
          if (a.superRag) r.superAmmo += a.n;
          else r.ammo = Math.min(12, r.ammo + a.n);
          r.anim.grab = -1;
          r.cur = null;
          this.spotStock[r.team] = Math.max(0, this.spotStock[r.team] - a.n);
          this.event('sys', `${teamLabel(r.team)}: ${a.superRag ? 'スーパー雑巾' : '雑巾'}${a.n}枚を装填`);
        }
        break;
      }
      case 'throw': {
        let aim = r.aim;
        if (!aim) {
          // 移動バケツで相手位置未推定などで未解決 → 少し待って再試行 (est が来たら投げる)。
          // 2.5秒待っても解決しなければ その1枚は諦めて次へ (無限に固まらない)。
          const outOfAmmo = (a.superRag ?? false) ? r.superAmmo <= 0 : r.ammo <= 0;
          if (r.throwsLeft > 0 && !outOfAmmo) {
            r.status = a.goal === 'moving' ? '相手位置を推定中 (移動バケツ)' : '狙点計算中';
            r.aim = this.resolveThrowSpec(r, a);
            if (!r.aim) {
              if (r.actT > 2.5) {
                r.throwsLeft--;
                r.actT = 0;
                if (r.throwsLeft <= 0) r.cur = null;
              }
              break;
            }
            r.actT = 0;
            aim = r.aim;
          } else {
            r.cur = null;
            break;
          }
        }
        const shotPoseReady = aim.goalKey !== 'moving' || this.trackMovingBucket(r, aim, dt);
        const cs = r.cycleScale;
        const aimP = this.par.shooter.aim * cs;
        const feedP = this.par.shooter.feed * cs;
        const recP = this.par.shooter.recover * cs;
        // ローラー目標回転数
        r.rollerTargetRpm = (aim.speed / (Math.PI * this.par.shooter.rollerDia)) * 60;
        // 送給アニメーション
        r.anim.feed = r.actT > aimP ? Math.min(1, (r.actT - aimP) / feedP) : -1;
        if (!aim.fired && r.actT >= aimP + feedP) {
          const yawErr = Math.abs(this.turretError(r));
          const rpmOk = Math.abs(r.rollerRpm - r.rollerTargetRpm) < r.rollerTargetRpm * 0.04 + 20;
          if (shotPoseReady && ((yawErr < this.par.turret.settleYaw && rpmOk) || r.actT >= (aimP + feedP) * 1.5)) {
            this.fire(r);
            r.anim.feed = -1;
          }
        }
        if (aim.fired && r.actT >= aimP + feedP + recP) {
          r.throwsLeft--;
          r.aim = null;
          r.actT = 0;
          if (r.throwsLeft > 0) {
            r.aim = this.resolveThrowSpec(r, a);
            if (!r.aim) r.throwsLeft = 0;
          }
          if (r.throwsLeft <= 0) {
            r.cur = null;
            r.rollerTargetRpm = 0;
          }
        }
        break;
      }
      default:
        r.cur = null;
        break;
    }
  }

  // ---------------------------------------------------------------- motion

  /**
   * 経路追従 (メカナム=ホロノミック)。純追跡(ルックアヘッド)でクロストラック補正しつつ、
   * ・終点への加速度ブレーキ ・コーナー曲率に応じた横加速度制限 で速度を決め、
   * 望み速度への変化を加速度制限でなめらかにする。滑らか化した経路と合わせて追従性を上げる。
   */
  private followPath(
    r: RobotState,
    path: Vec2[],
    dt: number,
    speed: number | undefined,
  ): boolean {
    const p = r.swerve ? this.par.swerveDrive : this.par.drive;
    const mul = Math.max(0.1, r.driveMul);
    const vmax = (speed ?? p.vmax) * mul;
    const acc = p.acc * mul;
    const n = path.length;
    const end = path[n - 1]!;
    if (n <= 1) {
      r.pathRemain = Math.hypot(end.x - r.x, end.z - r.z);
      return this.driveToward(r, end.x, end.z, dt, speed, true);
    }
    // 終点付近は精密停止制御に委譲し、目標へ ±2cm で収束させる (純追跡は終点精度が甘いため)
    const dEnd0 = Math.hypot(end.x - r.x, end.z - r.z);
    if (dEnd0 < 0.25) {
      r.pathRemain = dEnd0;
      // 終点接近: 標的方位が指定されていれば正対、なければ最終進行方向を保持
      if (r.arriveTheta != null) r.thetaTarget = r.arriveTheta;
      return this.driveToward(r, end.x, end.z, dt, speed, true);
    }
    const curSpeed = Math.hypot(r.vx, r.vz);

    // 1) フット(経路上の最近点)を pathIdx から前方探索 (逆戻り防止)
    const from = Math.max(0, Math.min(r.pathIdx, n - 2));
    let bestSeg = from;
    let footX = r.x;
    let footZ = r.z;
    let footT = 0;
    let bestD2 = Infinity;
    for (let i = from; i <= Math.min(n - 2, from + 8); i++) {
      const a = path[i]!;
      const b = path[i + 1]!;
      const ex = b.x - a.x;
      const ez = b.z - a.z;
      const l2 = ex * ex + ez * ez || 1e-9;
      const t = clamp(((r.x - a.x) * ex + (r.z - a.z) * ez) / l2, 0, 1);
      const px = a.x + ex * t;
      const pz = a.z + ez * t;
      const d2 = (r.x - px) ** 2 + (r.z - pz) ** 2;
      if (d2 < bestD2) {
        bestD2 = d2;
        bestSeg = i;
        footX = px;
        footZ = pz;
        footT = t;
      }
    }
    r.pathIdx = bestSeg;

    // 2) 終点までの残弧長
    let remain = Math.hypot(path[bestSeg + 1]!.x - footX, path[bestSeg + 1]!.z - footZ);
    for (let i = bestSeg + 1; i < n - 1; i++) {
      remain += Math.hypot(path[i + 1]!.x - path[i]!.x, path[i + 1]!.z - path[i]!.z);
    }
    r.pathRemain = remain; // スタック判定用 (経路沿いの進捗。直線距離だと迂回で誤検知)

    // 3) ルックアヘッド点 (フットから弧長 L 前方)。短めにして経路に密着させ、オーバーを抑える
    const L = clamp(0.28 + curSpeed * this.par.follow.lookaheadK, 0.28, 0.85);
    let lx = end.x;
    let lz = end.z;
    let need = L;
    let seg = bestSeg;
    let t = footT;
    while (seg < n - 1) {
      const a = path[seg]!;
      const b = path[seg + 1]!;
      const segLen = Math.hypot(b.x - a.x, b.z - a.z) || 1e-9;
      const rem = segLen * (1 - t);
      if (rem >= need) {
        const tt = t + need / segLen;
        lx = a.x + (b.x - a.x) * tt;
        lz = a.z + (b.z - a.z) * tt;
        break;
      }
      need -= rem;
      seg++;
      t = 0;
    }

    // 4) 望み速度: 終点ブレーキ + 前方コーナーへの予見ブレーキ
    const brakeV = Math.sqrt(Math.max(0, 2 * acc * Math.max(0, remain - 0.03)));
    // 前方の各コーナーの曲率から安全通過速度を出し、そこまで減速が間に合う現在速度の上限を
    // back-propagate する。これで「手前で十分減速」してカーブでオーバーしないようにする。
    const safeAcc = this.par.follow.safeAccF * acc; // 旋回に回せる横加速度 (制動と共有するので予算を絞る)
    let curveV = vmax;
    {
      // フットからの累積弧長。曲率は弧長 WIN の窓で測る (平滑化で増えた微小セグメントで
      // 曲率がスパイクし、緩いカーブの一点で速度ほぼゼロになる問題を防ぐ)。
      const cum: number[] = new Array(n).fill(0);
      for (let i = bestSeg + 1; i < n; i++) {
        cum[i] = cum[i - 1]! + Math.hypot(path[i]!.x - path[i - 1]!.x, path[i]!.z - path[i - 1]!.z);
      }
      const WIN = 0.5;
      for (let j = bestSeg + 1; j < n - 1; j++) {
        const arcToJ = cum[j]!;
        if (arcToJ > 3.5) break;
        let jb = j;
        while (jb > bestSeg && cum[j]! - cum[jb - 1]! < WIN) jb--;
        let jf = j;
        while (jf < n - 1 && cum[jf + 1]! - cum[j]! < WIN) jf++;
        if (jb >= j || jf <= j) continue;
        const a1 = Math.atan2(path[j]!.x - path[jb]!.x, path[j]!.z - path[jb]!.z);
        const a2 = Math.atan2(path[jf]!.x - path[j]!.x, path[jf]!.z - path[j]!.z);
        const dth = Math.abs(wrapAngle(a2 - a1));
        if (dth < 0.06) continue;
        const arcW = cum[jf]! - cum[jb]!;
        const R = arcW / dth; // 窓で安定推定した曲率半径
        const vSafe = Math.sqrt(safeAcc * R);
        const vAllow = Math.sqrt(vSafe * vSafe + 2 * acc * Math.max(0, arcToJ));
        if (vAllow < curveV) curveV = vAllow;
      }
    }
    const vdes = Math.min(vmax, brakeV, curveV);

    // 5) Frenet分解の追従制御: 接線軸と法線軸を分離して独立に制御する。
    //    ・接線方向 (経路に沿う, フット→ルックアヘッド): プロファイル速度 vdes を指令 (フィードフォワード)。
    //      前進成分は常に vdes = min(vmax, 減速, 曲率制限) なので、終点手前で行き過ぎない。
    //    ・法線方向 (経路に垂直, ロボット→フット): 横ズレのPDで経路へ引き戻す。前進速度には混ざらない。
    let tux = lx - footX;
    let tuz = lz - footZ;
    const tl = Math.hypot(tux, tuz) || 1e-9;
    tux /= tl;
    tuz /= tl;
    // 接線速度 (進行) — フット射影なので along-track 誤差は ~0、素直に vdes を出す
    const vT = vdes;
    // 法線 (経路へ向かう単位ベクトル) と 横ズレ量 d
    const ex = footX - r.x;
    const ez = footZ - r.z;
    const d = Math.hypot(ex, ez);
    let nux = 0;
    let nuz = 0;
    let vN = 0;
    if (d > 0.004) {
      nux = ex / d;
      nuz = ez / d;
      const vNcur = r.vx * nux + r.vz * nuz; // 現在の法線速度 (経路へ向かう成分)
      vN = this.par.follow.crossP * d - this.par.follow.crossD * vNcur; // 法線PD
    }
    let desVx = tux * vT + nux * vN;
    let desVz = tuz * vT + nuz * vN;
    // 総速度は「終点で停止できる速度 brakeV」を超えない。法線(横補正)成分で速度が膨らんで
    // 終点を行き過ぎ→振動するのを防ぐ。本線では brakeV が vmax より大きいので横補正は自由に効く。
    const cap = Math.min(vmax, brakeV);
    const dmag = Math.hypot(desVx, desVz);
    if (dmag > cap && dmag > 1e-9) {
      desVx = (desVx / dmag) * cap;
      desVz = (desVz / dmag) * cap;
    }

    // 6) 加速度制限でなめらかに追従
    const ax = desVx - r.vx;
    const az = desVz - r.vz;
    const an = Math.hypot(ax, az);
    // 加減速とも設定した加速度上限を厳守する (旧: ブレーキ時1.5倍で上限を超えていた)
    const amax = acc * dt;
    const s = an > amax ? amax / an : 1;
    r.vx += ax * s;
    r.vz += az * s;

    // 7) ヨー制御: 到達後に必要な目的方位 (次の射撃の標的方位 / 回収時の攻撃方位) へ、移動中から
    //    正対させておく (砲塔は車体±2.4rad しか回らないので、機体を向けておかないと撃てない)。
    //    進行方向(接線)へは向けない — 全方向ロボは横移動できるので接線に合わせる回頭は無駄。
    //    目的方位が無い goto は現在の向きを保持し、不要な回転をしない。
    if (r.arriveTheta != null) {
      r.thetaTarget = r.arriveTheta;
    }

    // 終点 0.25m 以内は上で driveToward に委譲済み。ここに来る間は未到達。
    return false;
  }

  private driveToward(
    r: RobotState,
    tx: number,
    tz: number,
    dt: number,
    speed: number | undefined,
    finalStop: boolean,
  ): boolean {
    const p = r.swerve ? this.par.swerveDrive : this.par.drive;
    const mul = Math.max(0.1, r.driveMul);
    const vmax = (speed ?? p.vmax) * mul;
    const acc = p.acc * mul;
    const dx = tx - r.x;
    const dz = tz - r.z;
    const d = Math.hypot(dx, dz);
    let dvx = 0;
    let dvz = 0;
    if (d > 1e-4) {
      // 目標点の 5mm 手前で速度0になるよう制動 → ±2cm 以内へ確実に収束できる
      const vdes = finalStop ? Math.min(vmax, Math.sqrt(2 * acc * Math.max(d - 0.005, 0))) : vmax;
      dvx = (dx / d) * vdes;
      dvz = (dz / d) * vdes;
    }
    const ax = dvx - r.vx;
    const az = dvz - r.vz;
    const an = Math.hypot(ax, az);
    const amax = acc * dt;
    const s = an > amax ? amax / an : 1;
    r.vx += ax * s;
    r.vz += az * s;
    // 到達判定を厳密化: 目標に ±2cm・ほぼ停止・角速度も収束するまで到達扱いにしない
    // 到達は位置±2cm・ほぼ停止で判定。機体の回頭整定(omega)は緩めに見る — 射撃精度は投擲側が
    // 砲塔整定(照準誤差)を待って撃つので、ここで機体を完全静止まで待つのは冗長で遅くなるだけ。
    return finalStop
      ? d < 0.02 && Math.hypot(r.vx, r.vz) < 0.04 && Math.abs(r.omega) < 0.8
      : d < 0.15;
  }

  private trackMovingBucket(r: RobotState, aim: AimSpec, dt: number): boolean {
    const opp = this.other(r.team);
    let target = this.movingBucketAimPoint(opp);
    if (r === this.player) {
      const est = this.oppTracker.est;
      if (!est) {
        r.status = '相手位置ロスト (推定待ち)';
        r.vx *= 0.8;
        r.vz *= 0.8;
        return false;
      }
      target = { x: est.x, y: opp.bucketTopY + 0.02, z: est.z };
    }
    aim.target = target;
    const best = this.optimizeMovingBucketShotPose(r, aim, target, opp);
    if (!best) {
      r.status = '移動バケツ射撃点なし';
      r.vx *= 0.8;
      r.vz *= 0.8;
      return false;
    }

    aim.shotPose = { x: best.x, z: best.z, cost: best.cost, dist: best.dist, prob: best.prob };
    aim.speed = best.speed;
    aim.angleRad = best.angleRad;
    aim.tof = best.tof;
    aim.prob = best.prob;
    r.path = null;
    r.thetaTarget = Math.atan2(target.x - r.x, target.z - r.z);
    r.path = [{ x: best.x, z: best.z }];
    r.pathIdx = 0;
    this.driveToward(r, best.x, best.z, dt, this.par.drive.vmax, true);

    const poseErr = Math.hypot(best.x - r.x, best.z - r.z);
    const speed = Math.hypot(r.vx, r.vz);
    const ready = poseErr < 0.38 && speed < 0.9;
    r.status = ready
      ? `最適射撃点: d=${best.dist.toFixed(1)}m p=${Math.round(best.prob * 100)}%`
      : `最適射撃点へ: cost=${best.cost.toFixed(1)}`;
    return ready;
  }

  private optimizeMovingBucketShotPose(
    r: RobotState,
    aim: AimSpec,
    target: { x: number; y: number; z: number },
    opp: RobotState,
  ): { x: number; z: number; cost: number; dist: number; prob: number; speed: number; angleRad: number; tof: number } | null {
    const sign = r.team === 'blue' ? 1 : -1;
    const shotClearance = ROBOT_R + 0.12;
    const xMax = FIELD.w / 2 - shotClearance;
    const zNear = FIELD.podium.d / 2 + shotClearance;
    const zFar = FIELD.l / 2 - shotClearance;
    const legalZ = (z: number) => sign * clamp(Math.abs(z), zNear, zFar);
    const obs = chassisObstacles(r.team);
    const rag = aim.superRag ? this.par.superRag : this.par.rag;
    const dy = target.y - r.muzzleY;
    const lanes = [zNear, zNear + 0.35, zNear + 0.8, zNear + 1.35, zNear + 2.0, Math.min(zFar, Math.abs(r.z))];
    const xOffsets = [-2.0, -1.4, -0.9, -0.45, 0, 0.45, 0.9, 1.4, 2.0];
    const candidates: Array<{ x: number; z: number }> = [
      { x: clamp(r.x, -xMax, xMax), z: legalZ(r.z) },
    ];
    for (const lane of lanes) {
      for (const dx of xOffsets) candidates.push({ x: clamp(target.x + dx, -xMax, xMax), z: sign * lane });
    }

    let best:
      | { x: number; z: number; cost: number; dist: number; prob: number; speed: number; angleRad: number; tof: number }
      | null = null;
    for (const c of candidates) {
      if (pointBlocked(c.x, c.z, obs, shotClearance)) continue;
      const dist = Math.max(0.3, Math.hypot(target.x - c.x, target.z - c.z) - 0.4);
      const sol = solveShot(dist, dy, rag, false);
      if (!sol) continue;
      const prob = this.movingBucketPoseProb(r, opp, c.x, c.z, dist, sol.speed);
      const travel = Math.hypot(c.x - r.x, c.z - r.z) / Math.max(0.1, this.par.drive.vmax);
      const oppSpeed = Math.hypot(opp.vx, opp.vz);
      const lateral = Math.abs(c.x - target.x);
      const speedCost = Math.max(0, sol.speed - 10);
      const movingUncertainty = oppSpeed * (travel + sol.tof);
      const cost =
        -prob * POINTS.moving +
        travel * 8.5 +
        dist * 2.6 +
        lateral * 0.45 +
        speedCost * speedCost * 0.18 +
        movingUncertainty * 5.0;
      if (!best || cost < best.cost) {
        best = { x: c.x, z: c.z, cost, dist, prob, speed: sol.speed, angleRad: sol.angleRad, tof: sol.tof };
      }
    }
    return best;
  }

  private movingBucketPoseProb(
    r: RobotState,
    opp: RobotState,
    x: number,
    z: number,
    dist: number,
    shotSpeed: number,
  ): number {
    const oppStill = !opp.powered || Math.hypot(opp.vx, opp.vz) < 0.05;
    const base = oppStill ? this.par.probs.bucketStill : this.par.probs.bucketMove;
    const distanceFactor = clamp(1.12 - dist * 0.055, 0.52, 1.05);
    const speedFactor = clamp(1.08 - Math.max(0, shotSpeed - 10) * 0.035, 0.55, 1.05);
    const approachFactor = clamp(1.02 - Math.hypot(x - r.x, z - r.z) * 0.015, 0.88, 1.02);
    return clamp(base * r.probMul * distanceFactor * speedFactor * approachFactor, 0.02, 0.98);
  }

  private turretError(r: RobotState): number {
    if (!r.aim) return 0;
    const want = Math.atan2(r.aim.target.x - r.x, r.aim.target.z - r.z);
    return wrapAngle(want - r.theta - r.turretYaw);
  }

  private classifyManualGoal(t: { x: number; y: number; z: number }): ThrowTargetKey {
    let bestKey: ThrowTargetKey = 'flag';
    let bestScore = Number.POSITIVE_INFINITY;
    const consider = (key: ThrowTargetKey, p: { x: number; y: number; z: number }): void => {
      const score = Math.hypot(t.x - p.x, t.z - p.z) + Math.abs(t.y - p.y) * 0.35;
      if (score < bestScore) {
        bestKey = key;
        bestScore = score;
      }
    };
    consider('moving', this.movingBucketAimPoint(this.opponent));
    for (const g of this.physGoals) consider(g.key, { x: g.x, y: g.topY, z: g.z });
    return bestScore < 0.78 ? bestKey : 'flag';
  }

  /** 手動/FPS操縦: 入力→車体速度・砲塔。スクリプトは使わない */
  private stepManualRobot(r: RobotState, dt: number): void {
    const p = r.swerve ? this.par.swerveDrive : this.par.drive;
    // FPSでは「見ている方向=前」が直感 (視線と車体が逆を向くとWASDが逆になる問題の修正)。
    // TPSはカメラが車体後方固定なので車体基準のままが正しい。
    const baseYaw = this.controlMode === 'fps' ? r.theta + r.turretYaw : r.theta;
    const fx = Math.sin(baseYaw);
    const fz = Math.cos(baseYaw);
    // 画面右 = forward×up。forward=(sinθ,0,cosθ), up=(0,1,0) → right=(-cosθ,0,sinθ)
    const rx = -Math.cos(baseYaw);
    const rz = Math.sin(baseYaw);
    const dvx = (fx * this.manual.f + rx * this.manual.s) * p.vmax;
    const dvz = (fz * this.manual.f + rz * this.manual.s) * p.vmax;
    const ax = dvx - r.vx;
    const az = dvz - r.vz;
    const an = Math.hypot(ax, az);
    const amax = p.acc * dt;
    const sc = an > amax ? amax / an : 1;
    r.vx += ax * sc;
    r.vz += az * sc;
    r.thetaTarget = wrapAngle(r.thetaTarget + this.manual.rot * 2.4 * dt);
    if (this.controlMode === 'fps') {
      // マウス照準: 砲塔を絶対方位へ
      r.turretYaw = Math.max(-2.4, Math.min(2.4, wrapAngle(this.manual.aimYaw - r.theta)));
      r.turretPitch = this.manual.aimPitch;
      r.rollerTargetRpm = (this.manual.power / (Math.PI * this.par.shooter.rollerDia)) * 60;
    } else if (this.controlMode === 'tps') {
      // TPS: カーソルの狙い点に向けて砲塔+弾道を自動解算 (クリックで即射出できる状態を維持)
      const t = this.manual.target;
      if (t) {
        const yaw = Math.atan2(t.x - r.x, t.z - r.z);
        r.turretYaw = Math.max(-2.4, Math.min(2.4, wrapAngle(yaw - r.theta)));
        const d = Math.max(0.3, Math.hypot(t.x - r.x, t.z - r.z) - 0.4);
        const dy = t.y - r.muzzleY;
        // 高い的 (旗の横棒など) は高弧を優先: 低伸弾道より上下の当たり余裕が大きい
        const preferLob = t.y > 1.2;
        const sol = preferLob
          ? (solveShot(d, dy, this.par.rag, true) ?? solveShot(d, dy, this.par.rag, false))
          : (solveShot(d, dy, this.par.rag, false) ?? solveShot(d, dy, this.par.rag, true));
        if (sol) {
          this.manual.sol = { ok: true, speed: sol.speed, angle: sol.angleRad };
          r.turretPitch = sol.angleRad;
          r.rollerTargetRpm = (sol.speed / (Math.PI * this.par.shooter.rollerDia)) * 60;
        } else {
          this.manual.sol = { ok: false, speed: 9, angle: 0.9 };
        }
      }
    }
  }

  /**
   * FPS/TPSの射出。事前判定なし — 物理判定 (checkPhysicalHit) で決まる。
   * 撃てない場合は必ず理由を返す (UX: 無反応にしない)。
   */
  manualFire(): { ok: boolean; reason?: string } {
    const r = this.player;
    if (this.over) return { ok: false, reason: '競技終了' };
    if (r.retry) return { ok: false, reason: 'リトライ復旧中' };
    if (r.ammo <= 0) return { ok: false, reason: '弾切れ — Rキー/接近して補充' };
    if (this.manual.fireCooldown > 0)
      return { ok: false, reason: `再装填中 (あと${this.manual.fireCooldown.toFixed(1)}s)` };
    if (this.controlMode === 'tps' && !this.manual.sol.ok)
      return { ok: false, reason: '射程外 — もっと近くを狙う' };
    if (this.tryJam(r, false)) {
      this.manual.fireCooldown = 1.0;
      return { ok: false, reason: `${teamLabel(r.team)}: 射出機構ジャム、リトライ宣言` };
    }
    r.ammo--;
    this.manual.fireCooldown = 0.85;
    const yaw = r.theta + r.turretYaw;
    const pitch = this.controlMode === 'tps' ? this.manual.sol.angle : r.turretPitch;
    const dirx = Math.sin(yaw);
    const dirz = Math.cos(yaw);
    const speed = this.controlMode === 'tps' ? this.manual.sol.speed : this.manual.power;
    const vh = Math.cos(pitch) * speed;
    const fallbackTarget = { x: r.x + dirx * 4, y: 1.5, z: r.z + dirz * 4 };
    const target = this.controlMode === 'tps' && this.manual.target ? { ...this.manual.target } : fallbackTarget;
    this.projectiles.push({
      id: this.nextId++,
      team: r.team,
      x: r.x + dirx * 0.4,
      y: r.muzzleY,
      z: r.z + dirz * 0.4,
      vx: dirx * vh,
      vy: Math.sin(pitch) * speed,
      vz: dirz * vh,
      spin: 18,
      superRag: false,
      outcome: null,
      goalKey: this.classifyManualGoal(target),
      target,
      t: 0,
      tof: 1.2,
      landed: false,
      aeroK: 1 + (this.rng() - 0.5) * 0.3,
      aeroL: 1 + (this.rng() - 0.5) * 0.4,
    });
    r.rollerRpm *= 0.92;
    return { ok: true };
  }

  /** 手動/FPSモードの装填 (Rキー)。撃てない/取れない理由を必ず返す */
  manualPickup(): { ok: boolean; reason?: string } {
    const b = this.player;
    const wp = waypoints(b.team).resupFront;
    if (this.over) return { ok: false, reason: '競技終了' };
    if (b.retry) return { ok: false, reason: 'リトライ復旧中' };
    const dist = Math.hypot(b.x - wp.x, b.z - wp.z);
    if (dist > 1.0)
      return { ok: false, reason: `補充スポットまで${dist.toFixed(1)}m — 1.0m以内に接近` };
    const stock = this.spotStock[b.team];
    if (stock <= 0) return { ok: false, reason: '補充スポットに雑巾なし (次のブザー待ち)' };
    if (b.ammo >= 12) return { ok: false, reason: 'ホッパー満杯 (12枚)' };
    const take = Math.min(10, stock, 12 - b.ammo);
    b.ammo += take;
    this.spotStock[b.team] = stock - take;
    this.event('sys', `${teamLabel(b.team)}: 手動装填 雑巾${take}枚`);
    return { ok: true, reason: `雑巾${take}枚を装填` };
  }

  /** 半自動操縦の指令 (スマホコントローラーUI / T22)。semiモードでだけ有効 */
  injectCommand(cmd: ControlCommand): void {
    const b = this.player;
    if (this.controlMode !== 'semi') return;
    const oppLabel = teamLabel(this.other(b.team).team);
    if (cmd === 'halt') {
      b.queue = [];
      b.cur = null;
      b.path = null;
      b.aim = null;
      b.throwsLeft = 0;
      b.vx = 0;
      b.vz = 0;
      b.status = '停止 (指令)';
      this.event('ev', `${teamLabel(b.team)}: 指令 停止`);
      return;
    }
    const wp = waypoints(b.team);
    const abort = (): void => {
      b.cur = null;
      b.path = null;
      b.aim = null;
      b.throwsLeft = 0;
    };
    const enqueueShot = (
      label: string,
      goto: Vec2 | null,
      goal: ThrowTargetKey,
      count = 1,
    ): void => {
      abort();
      const seq: Action[] = [{ t: 'status', text: `${label} (指令)` }];
      if (goto) seq.push({ t: 'goto', ...goto });
      seq.push({ t: 'throw', goal, count });
      b.queue.unshift(...seq);
      this.event('ev', `${teamLabel(b.team)}: 指令 ${label}`);
    };
    if (cmd === 'resup') {
      abort();
      b.queue.unshift(
        { t: 'status', text: '補充へ (指令)' },
        { t: 'goto', ...wp.resupFront },
        { t: 'pickup', n: 10, dur: this.par.pickup.dur },
      );
      this.event('ev', `${teamLabel(b.team)}: 指令 補充`);
    } else if (cmd === 'retry') {
      abort();
      this.beginRetry(b, '操作リトライ');
    } else if (cmd === 'throwFlag') {
      enqueueShot('旗へ射撃', wp.fireR, 'flag');
    } else if (cmd === 'throwB1') {
      enqueueShot('固定バケツ1へ射撃', wp.fireR, 'b1');
    } else if (cmd === 'throwB2') {
      enqueueShot('固定バケツ2へ射撃', wp.fireB2, 'b2');
    } else if (cmd === 'throwB3') {
      enqueueShot('固定バケツ3へ射撃', wp.fireB3, 'b3');
    } else if (cmd === 'throwDesk1') {
      enqueueShot('机1へ射撃', wp.fireDesk1, 'desk1');
    } else if (cmd === 'throwDesk2') {
      enqueueShot('机2へ射撃', wp.fireDesk2, 'desk2');
    } else if (cmd === 'throwMoving') {
      enqueueShot(`${oppLabel}移動バケツへ射撃`, null, 'moving');
    }
  }

  private stepRobot(r: RobotState, dt: number): void {
    if (r.retry) {
      this.stepRetry(r, dt);
    } else if (this.over) {
      r.rollerTargetRpm = 0;
    } else if (r === this.player && (this.controlMode === 'tps' || this.controlMode === 'fps')) {
      this.stepManualRobot(r, dt);
    } else if (r.powered) {
      this.stepAction(r, dt);
      // 任意の防御微動。通常は射撃中の姿勢を止めるため DEFAULT_PARAMS では無効。
      if (
        r === this.player &&
        this.par.defense.microMove &&
        r.cur?.t === 'throw' &&
        r.aim?.goalKey !== 'moving'
      ) {
        const rx = Math.cos(r.theta);
        const rz = -Math.sin(r.theta);
        const sway = Math.sin(this.t * 2.9) * 0.12;
        r.vx = rx * sway;
        r.vz = rz * sway;
      }
    } else {
      // 電源OFF: 待機と人による装填 (ルール4.1.4b) のみ進行
      if (r.cur && (r.cur.t === 'wait' || r.cur.t === 'waitUntil' || r.cur.t === 'pickup'))
        this.stepAction(r, dt);
      else if (!r.cur) this.stepAction(r, dt);
      r.vx = 0;
      r.vz = 0;
    }

    // 位置積分 + 衝突解決
    // 独立ステア(スワーブ): 速度ベクトルはモジュール方位に沿い、方位は操舵レートでしか回らない。
    // → 青のメカナム(瞬時に横移動)と違い、方向転換で弧を描く。
    if (r.swerve) {
      const spd = Math.hypot(r.vx, r.vz);
      if (spd > 0.02) {
        const desDir = Math.atan2(r.vx, r.vz);
        const d = wrapAngle(desDir - r.moduleDir);
        const maxStep = 4.5 * dt; // 操舵レート 4.5 rad/s (急激な操舵はできない)
        r.moduleDir = wrapAngle(r.moduleDir + clamp(d, -maxStep, maxStep));
        // モジュールが向ききるまでは前進成分だけ = 未整列だと減速して曲がる
        const align = Math.max(0.3, Math.cos(wrapAngle(desDir - r.moduleDir)));
        const out = spd * align;
        r.vx = Math.sin(r.moduleDir) * out;
        r.vz = Math.cos(r.moduleDir) * out;
      }
    }
    if (r.powered && !this.over) {
      r.x += r.vx * dt;
      r.z += r.vz * dt;
      const res = resolveCircle(r.x, r.z, ROBOT_R, chassisObstacles(r.team));
      if (res.hit) {
        r.x = res.x;
        r.z = res.z;
        const vn = r.vx * res.nx + r.vz * res.nz;
        if (vn < 0) {
          r.vx -= vn * res.nx;
          r.vz -= vn * res.nz;
        }
        r.collided = true;
        // Q25: 「動かすと反則」の競技用品 (固定バケツ/机/旗/椅子) に押し込んだら強制リトライ。
        // ただし経路追従中の一瞬の擦り (押し出しで即解消) では発火させず、
        // 押し込みが継続した (=本当にめり込んで動かしている) 場合のみ審判が宣告する。
        const kind = res.tag.split(':')[1] ?? res.tag;
        const banItem = ['b1', 'b2', 'b3', 'desk1', 'desk2', 'chair', 'flag'].includes(kind);
        // 接触は押し出しで depth が小さくなるため、接触した時点で加算 (擦り抜けは1フレームで減衰)。
        if (banItem && res.depth > 0.008) r.banContactT += dt;
        else r.banContactT = Math.max(0, r.banContactT - dt * 1.5);
        // 強くめり込んだ (depth>0.04) か、短時間でも当たり続けた (0.12s) ら強制リトライ
        if (!r.retry && banItem && (res.depth > 0.04 || r.banContactT > 0.12)) {
          this.beginRetry(r, `競技用品(${kind})に接触 — 強制リトライ`);
          if (r.retry) {
            (r.retry as RetryState).forced = true;
            this.event('ev', `審判: ${teamLabel(r.team)}に強制リトライ! 15秒間その場で停止`);
          }
        }
      } else {
        r.collided = false;
        r.banContactT = Math.max(0, r.banContactT - dt * 1.5);
      }
      // 自陣 + フェンス内へクランプ
      const zMin = FIELD.podium.d / 2 + ROBOT_R;
      const zMax = FIELD.l / 2 - ROBOT_R;
      const xMax = FIELD.w / 2 - ROBOT_R;
      if (r.team === 'blue') r.z = Math.max(zMin, Math.min(zMax, r.z));
      else r.z = Math.min(-zMin, Math.max(-zMax, r.z));
      r.x = Math.max(-xMax, Math.min(xMax, r.x));

      // 雑巾踏みハザード: 床の雑巾を踏むとトラクション喪失(スリップ)、
      // 踏み続けるとタイヤに絡まってリトライになる (par.ragHazard で on/off)
      if (this.par.ragHazard && !r.retry) {
        const spd0 = Math.hypot(r.vx, r.vz);
        const rag =
          spd0 > 0.05
            ? this.hung.find(
                (h) => h.kind === 'floor' && Math.hypot(h.x - r.x, h.z - r.z) < 0.42,
              )
            : undefined;
        if (rag) {
          r.vx *= 0.5; // トラクション喪失で減速
          r.vz *= 0.5;
          r.anim.strafe = 1; // ホイールスピンの視覚
          r.ragDragT += dt;
          if (this.rng() < 0.35 * dt) {
            const idx = this.hung.indexOf(rag);
            if (idx >= 0) this.hung.splice(idx, 1); // 絡まって巻き込まれた
            this.beginRetry(r, '雑巾の絡まり');
            r.ragDragT = 0;
            this.event(
              r.team === this.playerTeam ? 'ev' : 'opp',
              `${teamLabel(r.team)}: 雑巾がタイヤに絡まりリトライ`,
            );
          }
        } else {
          r.ragDragT = Math.max(0, r.ragDragT - dt * 2);
        }
      }
    }

    // 車体ヨー: 経路追従中は目標方位へ「終端までに均等に」回頭する。始めに全力で首を振ってすぐ
    // 正対する動きは急で不自然なので、残り到達時間に合わせた角速度で緩やかに回し、到達の少し手前で
    // ちょうど回頭し終える。(全方向ロボは並進と旋回が独立なので到達時間自体は変わらないが、動きが
    // 滑らかになり、swerveでは無駄な操舵の切り返しも減る。)
    const err = wrapAngle(r.thetaTarget - r.theta);
    let omega: number;
    if (r.cur && r.cur.t === 'goto' && r.pathRemain > 0.08 && Number.isFinite(r.pathRemain)) {
      const spd = Math.max(0.5, Math.hypot(r.vx, r.vz));
      const tFinish = Math.max(dt, (r.pathRemain / spd) * 0.85); // 残り到達時間の85%で回頭完了
      omega = Math.max(-3, Math.min(3, err / tFinish));
    } else {
      omega = Math.max(-3, Math.min(3, err * 4)); // その場照準・手動など: 素早く整定
    }
    r.omega = omega;
    if (Math.abs(err) < 0.01) r.omega = 0;
    r.theta += r.omega * dt;

    // 砲塔 (TPS/FPS操縦時は手動照準直結なので自動整定をスキップ)
    const manualPlayer = r === this.player && (this.controlMode === 'tps' || this.controlMode === 'fps');
    if (!manualPlayer) {
      // Q20: 躍度制御 — 目標角速度に加速度制限つきで追従するS字プロファイル
      const ACC = 16; // rad/s^2 (M3508+ベルト減速の現実的な砲塔加速度)
      const yawErr = r.aim ? this.turretError(r) : -r.turretYaw;
      const yawVmax = r.aim ? this.par.turret.yawRate : 2.0;
      const yawVDes = clamp(yawErr * 5, -yawVmax, yawVmax);
      r.turretYawVel += clamp(yawVDes - r.turretYawVel, -ACC * dt, ACC * dt);
      r.turretYaw = clamp(r.turretYaw + r.turretYawVel * dt, -2.4, 2.4);
      const pitchTarget = r.aim ? r.aim.angleRad : 0.4;
      const pErr = pitchTarget - r.turretPitch;
      const pVDes = clamp(pErr * 6, -2.6, 2.6);
      r.turretPitchVel += clamp(pVDes - r.turretPitchVel, -ACC * dt, ACC * dt);
      r.turretPitch += r.turretPitchVel * dt;
    }

    // ローラー回転数
    const dr = this.par.shooter.spinupRpmPerSec * dt;
    const diff = r.rollerTargetRpm - r.rollerRpm;
    r.rollerRpm += Math.max(-dr * 2, Math.min(dr, diff));
    if (r.rollerRpm < 0) r.rollerRpm = 0;

    // ホイール回転 (アニメーション用)
    const om = wheelOmegas(r.theta, r.vx, r.vz, r.omega, this.par.wheel.r, this.par.wheel.k);
    r.anim.wheelOmega = om;
    for (let i = 0; i < 4; i++) r.anim.wheels[i] = (r.anim.wheels[i] ?? 0) + om[i]! * dt;
    r.anim.strafe = strafeSpeed(r.theta, r.vx, r.vz);
  }

  // ---------------------------------------------------------------- step

  step(dt: number): void {
    if (this.over && this.projectiles.length === 0) return;
    if (!this.over) {
      this.t += dt;
      for (const b of BUZZERS) {
        if (this.t >= b && !this.buzzed.has(b)) {
          this.buzzed.add(b);
          if (b === 30 || b === 60) {
            this.spotStock.blue += 10;
            this.spotStock.red += 10;
          } else if (b === 120) {
            this.spotStock.blue += 2;
            this.spotStock.red += 2;
          }
          const msg =
            b === 30 || b === 60
              ? `ブザー: 雑巾+10枚 (${b}秒)`
              : b === 120
                ? 'ブザー: スーパー雑巾+2枚'
                : '2:30 固定バケツからの回収解禁';
          this.event('sys', msg);
        }
      }
      if (this.t >= MATCH_LEN && !this.timeExpired) {
        this.timeExpired = true;
        // 0:00 以降はフリーモード: ロボットは動き続ける (over にはしない)。
        // 歓声はここで一度だけ鳴らし、以降は抑制する (audio 側で timeExpired を見る)。
        const b = this.score.blue;
        const r = this.score.red;
        this.event('sys', `タイマー0:00 ${b} - ${r} ${b > r ? '勝ち' : b < r ? '負け' : '同点'} / 競技終了 (以降フリー走行)`);
      }
    }

    this.stepRobot(this.blue, dt);
    this.stepRobot(this.red, dt);
    if (!this.over) this.stepFlagFall(dt); // 旗の雑巾の確率的落下
    // 操作側の瞬間加速度 (速度ベクトルの変化)
    const pl = this.player;
    this.playerAccel = Math.hypot(pl.vx - this.pvx, pl.vz - this.pvz) / Math.max(dt, 1e-4);
    this.pvx = pl.vx;
    this.pvz = pl.vz;

    // 弾道 (抗力 + バックスピン揚力: ソルバーと同一モデル)
    const kmN = this.par.rag.k / this.par.rag.m;
    const kmS = this.par.superRag.k / this.par.superRag.m;
    for (const pr of this.projectiles) {
      if (pr.landed) continue;
      let rem = dt;
      // 個体差: 雑巾ごとに空力係数がばらつく (T23 — 布の変形/回転の簡易表現)
      const km = (pr.superRag ? kmS : kmN) * (pr.aeroK ?? 1);
      const lift = (pr.superRag ? this.par.superRag.lift : this.par.rag.lift) * (pr.aeroL ?? 1);
      while (rem > 1e-6 && !pr.landed) {
        const h = Math.min(PROJ_DT, rem);
        rem -= h;
        const v = Math.hypot(pr.vx, pr.vy, pr.vz);
        const vh = Math.hypot(pr.vx, pr.vz);
        pr.vx -= km * v * pr.vx * h;
        pr.vy -= (9.81 + km * v * pr.vy - lift * vh) * h;
        pr.vz -= km * v * pr.vz * h;
        pr.x += pr.vx * h;
        pr.y += pr.vy * h;
        pr.z += pr.vz * h;
        pr.t += h;
        if (pr.outcome === null) {
          // 手動射出: 事前判定なし。ゴール形状との実交差で判定
          if (!this.checkPhysicalHit(pr) && pr.y < 0.02 && pr.vy < 0) {
            this.landProjectile(pr, false);
          }
        } else {
          const distT = Math.hypot(pr.x - pr.target.x, pr.y - pr.target.y, pr.z - pr.target.z);
          if (pr.outcome && distT < 0.3) this.landProjectile(pr, true);
          else if (pr.outcome && pr.t > pr.tof * 1.7) this.landProjectile(pr, true);
          else if (pr.y < 0.02 && pr.vy < 0) this.landProjectile(pr, pr.outcome === true && distT < 0.6);
        }
      }
    }
    this.projectiles = this.projectiles.filter((p) => !p.landed);

    this.manual.fireCooldown = Math.max(0, this.manual.fireCooldown - dt);

    // センサー + 推定 (操作チーム搭載 UST-20LX / 計測輪+ICP)
    if (!this.headless) {
      const p = this.player;
      const opp = this.opponent;
      this.localizer.predict(p.vx, p.vz, p.omega, dt, this.rng);
      this.scanTimer -= dt;
      if (this.scanTimer <= 0) {
        // 実機は40Hzだが推定・描画は10Hzに間引き
        this.scanTimer = 0.1;
        const pose = { x: p.x, z: p.z, theta: p.theta };
        const vel = { vx: p.vx, vz: p.vz, omega: p.omega };
        const oppPose = { x: opp.x, z: opp.z, theta: opp.theta };
        // 下段 UST-20LX ×2 (前向き/後ろ向き): 270°×2 = 完全360°+側方オーバーラップ
        this.lastScan = castScan(pose, vel, oppPose, this.rng, this.t);
        this.lastScanRear = castScan(pose, vel, oppPose, this.rng, this.t, undefined, Math.PI);
        this.localizer.correct(this.lastScan, this.lastScanRear);
        // 上段LiDAR: 教壇越しに相手車体を検出
        this.lastUpperScan = castScan(pose, vel, oppPose, this.rng, this.t, getUpperSegs());
        this.oppTracker.update(this.lastUpperScan, this.localizer.est, 0.1);
      }
    }
  }

  /** 手動射出弾の物理ゴール判定。命中したら着弾処理して true */
  private checkPhysicalHit(pr: Projectile): boolean {
    // 移動バケツ (両ロボット)
    for (const carrier of [this.blue, this.red]) {
      const b = this.movingBucketAimPoint(carrier);
      const horiz = Math.hypot(pr.x - b.x, pr.z - b.z);
      if (horiz < 0.115 && pr.vy < 0 && pr.y < b.y + 0.02 && pr.y > b.y - 0.26) {
        const scorer: TeamId = carrier.team === 'blue' ? 'red' : 'blue';
        this.landPhysical(pr, 'moving', scorer, { x: b.x, y: b.y, z: b.z }, carrier.team);
        return true;
      }
    }
    for (const g of this.physGoals) {
      const scorer: TeamId = g.side === 'red' ? 'blue' : 'red';
      if (g.kind === 'flag') {
        if (
          Math.abs(pr.y - g.topY) < 0.15 &&
          Math.abs(pr.x - g.x) < 0.34 &&
          Math.abs(pr.z - g.z) < 0.13 &&
          pr.t > 0.08
        ) {
          this.landPhysical(pr, 'flag', scorer, { x: g.x, y: g.topY, z: g.z }, null);
          return true;
        }
      } else if (g.kind === 'bucket') {
        const horiz = Math.hypot(pr.x - g.x, pr.z - g.z);
        if (horiz < g.r - 0.015 && pr.vy < 0 && pr.y < g.topY + 0.02 && pr.y > g.topY - 0.26) {
          this.landPhysical(pr, g.key, scorer, { x: g.x, y: g.topY, z: g.z }, null);
          return true;
        }
      } else {
        // 机下棚 W650(x)×D450(z)
        if (
          Math.abs(pr.x - g.x) < 0.31 &&
          Math.abs(pr.z - g.z) < 0.22 &&
          pr.y > 0.3 &&
          pr.y < 0.62
        ) {
          this.landPhysical(pr, g.key, scorer, { x: g.x, y: g.topY, z: g.z }, null);
          return true;
        }
      }
    }
    return false;
  }

  private landPhysical(
    pr: Projectile,
    key: ThrowTargetKey,
    scorer: TeamId,
    at: { x: number; y: number; z: number },
    movingCarrier: TeamId | null,
  ): void {
    pr.landed = true;
    if (key === 'flag') {
      this.hung.push({
        id: pr.id,
        x: at.x + (this.rng() - 0.5) * 0.55,
        y: at.y - 0.05,
        z: at.z,
        yaw: (this.rng() - 0.5) * 0.6,
        kind: 'bar',
        superRag: false,
        team: scorer,
      });
    } else if (key === 'desk1' || key === 'desk2') {
      this.hung.push({
        id: pr.id, x: at.x, y: 0.3, z: at.z, yaw: this.rng() * Math.PI,
        kind: 'shelf', superRag: false, team: scorer,
      });
    } else if (key === 'moving' && movingCarrier) {
      const carrier = movingCarrier === 'blue' ? this.blue : this.red;
      const localX = 0.24 + (this.rng() - 0.5) * 0.1;
      const localZ = -0.24 + (this.rng() - 0.5) * 0.1;
      this.hung.push({
        id: pr.id, x: at.x, y: carrier.bucketTopY - 0.11, z: at.z,
        attachedTo: carrier.team, localX, localY: carrier.bucketTopY - 0.11, localZ,
        yaw: this.rng() * Math.PI, kind: 'bucket', superRag: false, team: scorer,
      });
    } else {
      this.hung.push({
        id: pr.id, x: at.x, y: at.y - 0.1, z: at.z, yaw: this.rng() * Math.PI,
        kind: 'bucket', superRag: false, team: scorer,
      });
    }
    if (scorer !== pr.team) {
      this.event('opp', `${teamLabel(pr.team)}のオウンゴール: ${teamLabel(scorer)}の得点になります`);
    }
    this.addScore(scorer, key, false);
  }

  /** 行動判断に効く実数値 (T9 計測HUD用)。すべて実測/実推定値 */
  metrics(): {
    estErrMm: number;
    icpRmseMm: number;
    oppErrMm: number | null;
    pFlag: number;
    pMoving: number;
    ammo: number;
    superAmmo: number;
    nextEvent: { label: string; inSec: number } | null;
    estX: number;
    estZ: number;
    estThetaDeg: number;
    speed: number;
    accel: number;
    muzzleSpeed: number;
    muzzlePitchDeg: number;
    lastShot: { label: string; prob: number; agoSec: number } | null;
  } {
    const b = this.player;
    const oppRobot = this.opponent;
    const est = this.localizer.est;
    const estErrMm = Math.hypot(est.x - b.x, est.z - b.z) * 1000;
    const opp = this.oppTracker.est;
    const oppErrMm = opp ? Math.hypot(opp.x - oppRobot.x, opp.z - oppRobot.z) * 1000 : null;
    const target = goalAimPoint(b.team, 'flag');
    const d = Math.max(0.3, Math.hypot(target.x - b.x, target.z - b.z) - 0.4);
    const distFactor = clamp(1 + (2.5 - d) * 0.075, 0.55, 1.18);
    const pFlag = clamp(this.par.probs.flag * this.flagSaturation(b.team) * distFactor, 0, 0.98);
    const oppStill = !oppRobot.powered || Math.hypot(oppRobot.vx, oppRobot.vz) < 0.05;
    const pMoving = oppStill ? this.par.probs.bucketStill : this.par.probs.bucketMove;
    let nextEvent: { label: string; inSec: number } | null = null;
    for (const [tt, label] of [
      [30, '雑巾+10'],
      [60, '雑巾+10'],
      [120, 'スーパー+2'],
      [150, '回収解禁'],
      [180, '終了'],
    ] as const) {
      if (this.t < tt) {
        nextEvent = { label, inSec: tt - this.t };
        break;
      }
    }
    let thDeg = (est.theta * 180) / Math.PI;
    while (thDeg > 180) thDeg -= 360;
    while (thDeg < -180) thDeg += 360;
    const ls = b.lastShot;
    const lastShot =
      ls && this.t - ls.t < 5
        ? { label: GOAL_LABEL[ls.key], prob: ls.prob, agoSec: this.t - ls.t }
        : null;
    return {
      estErrMm,
      icpRmseMm: this.localizer.diag.rmse * 1000,
      oppErrMm,
      pFlag,
      pMoving,
      ammo: b.ammo,
      superAmmo: b.superAmmo,
      nextEvent,
      estX: est.x,
      estZ: est.z,
      estThetaDeg: thDeg,
      speed: Math.hypot(b.vx, b.vz),
      accel: this.playerAccel,
      muzzleSpeed: b.aim ? b.aim.speed : this.manual.power,
      muzzlePitchDeg: ((b.aim ? b.aim.angleRad : this.manual.aimPitch) * 180) / Math.PI,
      lastShot,
    };
  }

  /** ヘッドレス一括実行 */
  runToEnd(dt = 1 / 30): void {
    let guard = 0;
    while (((!this.timeExpired && !this.over) || this.projectiles.length > 0) && guard < 200000) {
      this.step(dt);
      guard++;
      if (this.t > MATCH_LEN + 10) break;
    }
  }
}
