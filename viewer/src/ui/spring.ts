/**
 * Apple 流のフルイド・モーション基盤 (.claude/skills/apple-design)。
 *
 * - パラメータは物理三つ組 (mass/stiffness/damping) ではなく
 *   damping ratio (overshoot 量) と response (秒) の 2 つ。
 * - 常に「現在の表示値」から動くので、途中で掴んで反転できる (interruptible)。
 * - retarget 時に速度を引き継ぐので、反転しても "brick wall" にならない。
 * - ジェスチャ終了時は指の速度をそのまま初速として渡せる (velocity handoff)。
 */

export interface SpringConfig {
  /** 1.0 = 臨界減衰 (オーバーシュートなし)。0.8 前後で軽いバウンス。 */
  damping: number;
  /** 目標へ到達する速さ (秒)。duration ではない。 */
  response: number;
}

/** UI 既定: オーバーシュートしない。フリック由来の動きだけ bounce を足す。 */
export const SPRING_UI: SpringConfig = { damping: 1.0, response: 0.35 };
/** 移動・リポジション (PiP 相当)。 */
export const SPRING_MOVE: SpringConfig = { damping: 1.0, response: 0.4 };
/** ドロワー / シート。運動量を伴うので少しだけ跳ねる。 */
export const SPRING_SHEET: SpringConfig = { damping: 0.8, response: 0.3 };

const REST_VALUE = 0.4;
const REST_VELOCITY = 4;

type Tick = (dt: number) => void;
const ticking = new Set<Tick>();
let looping = false;
let lastTs = 0;

// ループの生死は ticking の中身だけで決まる。rAF の id を持ち回すと
// 「id は残っているがコールバックは死んでいる」状態で二度と回らなくなる。
function loop(ts: number): void {
  const dt = lastTs ? Math.min(0.064, (ts - lastTs) / 1000) : 1 / 60;
  lastTs = ts;
  for (const fn of [...ticking]) fn(dt);
  if (ticking.size > 0) {
    requestAnimationFrame(loop);
  } else {
    looping = false;
    lastTs = 0;
  }
}

function schedule(fn: Tick): void {
  ticking.add(fn);
  if (!looping) {
    looping = true;
    lastTs = 0;
    requestAnimationFrame(loop);
  }
}

export function prefersReducedMotion(): boolean {
  return typeof matchMedia === 'function' && matchMedia('(prefers-reduced-motion: reduce)').matches;
}

/** 1 自由度のスプリング。2D は X/Y で別インスタンスにする (単一スプリングだと軸がずれる)。 */
export class Spring {
  value: number;
  velocity = 0;
  target: number;
  private cfg: SpringConfig;
  private readonly onUpdate: (v: number) => void;
  private readonly onRest?: () => void;
  private readonly tick: Tick;
  private running = false;

  constructor(
    initial: number,
    onUpdate: (v: number) => void,
    cfg: SpringConfig = SPRING_UI,
    onRest?: () => void,
  ) {
    this.value = initial;
    this.target = initial;
    this.cfg = cfg;
    this.onUpdate = onUpdate;
    this.onRest = onRest;
    this.tick = (dt) => this.step(dt);
  }

  /** 目標だけ差し替える。現在値と速度はそのまま = 途中割り込みが滑らかに繋がる。 */
  set(target: number, opts?: { velocity?: number; config?: Partial<SpringConfig> }): void {
    this.target = target;
    if (opts?.config) this.cfg = { ...this.cfg, ...opts.config };
    if (opts?.velocity !== undefined) this.velocity = opts.velocity;
    if (prefersReducedMotion()) {
      this.stop();
      this.value = target;
      this.velocity = 0;
      this.onUpdate(this.value);
      this.onRest?.();
      return;
    }
    if (!this.running) {
      this.running = true;
      schedule(this.tick);
    }
  }

  /** ドラッグ中など、物理を止めて値を直接書く (1:1 追従)。 */
  jump(value: number, velocity = 0): void {
    this.stop();
    this.value = value;
    this.target = value;
    this.velocity = velocity;
    this.onUpdate(value);
  }

  stop(): void {
    this.running = false;
    ticking.delete(this.tick);
  }

  private step(dt: number): void {
    const omega = (2 * Math.PI) / this.cfg.response;
    const k = omega * omega;
    const c = 2 * this.cfg.damping * omega;
    // 硬いスプリングでも発散しないよう小刻みに積分する
    const steps = Math.max(1, Math.ceil(dt / (1 / 240)));
    const h = dt / steps;
    for (let i = 0; i < steps; i++) {
      const a = -k * (this.value - this.target) - c * this.velocity;
      this.velocity += a * h;
      this.value += this.velocity * h;
    }
    if (Math.abs(this.value - this.target) < REST_VALUE && Math.abs(this.velocity) < REST_VELOCITY) {
      this.value = this.target;
      this.velocity = 0;
      this.onUpdate(this.value);
      this.stop();
      this.onRest?.();
      return;
    }
    this.onUpdate(this.value);
  }
}

/**
 * 慣性の着地点を予測する (Apple の Designing Fluid Interfaces サンプルと同式)。
 * 教科書の v²/(2a) ではなく指数減衰形を使う。
 */
export function project(initialVelocity: number, decelerationRate = 0.998): number {
  return ((initialVelocity / 1000) * decelerationRate) / (1 - decelerationRate);
}

/** 境界の外へ引っ張るほど付いてこなくなる (ハードストップにしない)。 */
export function rubberband(overshoot: number, dimension: number, constant = 0.55): number {
  return (overshoot * dimension * constant) / (dimension + constant * Math.abs(overshoot));
}

/** 直近の pointermove 履歴から速度 (px/s) を出す。releaseVelocity をスプリングに渡すため。 */
export class VelocityTracker {
  private samples: Array<{ v: number; t: number }> = [];

  add(value: number, t: number): void {
    this.samples.push({ v: value, t });
    // 直近 100ms だけ見る。長く取ると指を止めても速度が残る。
    while (this.samples.length > 2 && t - this.samples[0].t > 0.1) this.samples.shift();
  }

  clear(): void {
    this.samples = [];
  }

  /** px/s */
  velocity(): number {
    if (this.samples.length < 2) return 0;
    const a = this.samples[0];
    const b = this.samples[this.samples.length - 1];
    const dt = b.t - a.t;
    if (dt <= 0.001) return 0;
    return (b.v - a.v) / dt;
  }
}
