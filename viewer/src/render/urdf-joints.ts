import type { RobotState } from '../sim/types';

/**
 * 試合シムの状態 → URDF の関節値。**Three.js に依存しない**ので、
 * ブラウザを起こさずに Node からも検査できる（`npm run urdf:check`）。
 *
 * ⚠ 描画の中に埋めてはいけない。ここが「どのモーターをどう回すか」の
 *   仕様そのもので、実機の制御（`cad/control/tr_control.py`）と
 *   突き合わせる相手になる。見た目のコードに紛れていると、
 *   シムだけ直して実機の指令が古いまま、という食い違いが起きる。
 */

/** continuous 関節は角度を積算する必要があるので、呼ぶ側が持つ */
export interface JointAccum {
  roller: number;
  singulator: number;
}

export function newAccum(): JointAccum {
  return { roller: 0, singulator: 0 };
}

/** URDF の関節名（12 個）。順番は表示・検査の並び順 */
export const JOINT_NAMES = [
  'wheel_fl',
  'wheel_fr',
  'wheel_rl',
  'wheel_rr',
  'turret_yaw',
  'shooter_pitch',
  'roller_upper',
  'roller_lower',
  'singulator',
  'grabber_slide',
  'grabber_press',
  'fork_tilt',
] as const;

export type JointName = (typeof JOINT_NAMES)[number];

/** グラバーの可動域 [m, m, rad]（URDF の upper と一致させること） */
export const GRAB_STROKE = 0.316;
export const PRESS_STROKE = 0.138;
export const FORK_TILT = 0.383972;

/**
 * 各軸の実速度 [m/s, m/s, rad/s]。⚠ **URDF の `<limit velocity>` と同じ値**。
 * `tr_urdf.JOINT_LIMITS`: grabber_slide 0.30 / grabber_press 0.10 / fork_tilt 2.0
 */
export const GRAB_SPEED = 0.3;
export const PRESS_SPEED = 0.1;
export const TILT_SPEED = 2.0;

/** 各工程に要る時間 [s]（可動域 ÷ 速度）。合計 3.7s が機構の下限 */
const T_EXT = GRAB_STROKE / GRAB_SPEED; // 1.053
const T_PRESS = PRESS_STROKE / PRESS_SPEED; // 1.380
const T_TILT = FORK_TILT / TILT_SPEED; // 0.192
const T_RETRACT = T_EXT;
export const GRAB_CYCLE_MIN = T_EXT + T_PRESS + T_TILT + T_RETRACT;

/**
 * グラバーの 1 サイクル (`anim.grab` 0..1) → スライド量・押さえ量・傾き (0..1)。
 *
 * ⚠ **工程の切れ目は割合ではなく、モーターが出せる速度で決まる。**
 *   0.3 / 0.5 / 0.75 という丸い数字を置いていたが、それだと上押さえ
 *   （0.1m/s で 138mm ＝ 1.38s かかる）が一瞬で降りて、スライド
 *   （0.3m/s で 316mm ＝ 1.05s）より速く見えていた。実機では逆。
 *
 * `dur` はそのピックアップ動作に与えられた総時間。機構の下限（3.7s）より
 * 長いぶんは、押さえたまま待つ「間」に配る（山を崩さないための整定）。
 */
export function grabPhase(p: number, dur = 8): { ext: number; press: number; tilt: number } {
  if (p < 0) return { ext: 0, press: 0, tilt: 0 };
  const t = p * Math.max(dur, GRAB_CYCLE_MIN);
  const dwell = Math.max(0, Math.max(dur, GRAB_CYCLE_MIN) - GRAB_CYCLE_MIN);
  const t0 = T_EXT; // 伸ばし終わり
  const t1 = t0 + T_PRESS; // 押さえ終わり
  const t2 = t1 + T_TILT; // 傾け終わり
  const t3 = t2 + dwell; // 整定待ち終わり
  const clamp01 = (v: number): number => (v < 0 ? 0 : v > 1 ? 1 : v);
  return {
    ext: t <= t2 + dwell ? clamp01(t / t0) : 1 - clamp01((t - t3) / T_RETRACT),
    press: clamp01((t - t0) / T_PRESS),
    tilt: clamp01((t - t1) / T_TILT),
  };
}

/** 分離ローラーの回転速度 [rad/s]（送給中のみ） */
export const SINGULATOR_RATE = 12;

/**
 * 1 フレームぶんの関節値を出す。`acc` は破壊的に進む（continuous の積算）。
 */
export function mapJoints(r: RobotState, acc: JointAccum, dt: number): Record<JointName, number> {
  acc.roller += (r.rollerRpm / 60) * Math.PI * 2 * dt;
  if (r.anim.feed >= 0) acc.singulator -= SINGULATOR_RATE * dt;
  const g = grabPhase(r.anim.grab, r.anim.grabDur);
  return {
    // ⚠ 車輪の軸は +Y（左）。前進は右ねじで**負**の回転
    wheel_fl: -(r.anim.wheels[0] ?? 0),
    wheel_fr: -(r.anim.wheels[1] ?? 0),
    wheel_rl: -(r.anim.wheels[2] ?? 0),
    wheel_rr: -(r.anim.wheels[3] ?? 0),
    turret_yaw: r.turretYaw,
    shooter_pitch: r.turretPitch,
    // 対向ローラー。⚠ 符号が同じだと布を噛まずに弾き返す。
    // 軸は両方 +Y、上ローラーはニップが自分の下面（-Z）なので **負**、
    // 下ローラーは上面（+Z）なので正。どちらもニップ面が +X（射出方向）へ
    // 動く。`shooter_pitch` 座標で 0.2rad ぶん回すと接触点は上下とも
    // Δx=+8.94mm・Δz=±0.90mm（= R sinθ, R(1-cosθ), R=45mm）。
    // ⚠ 「回転がおかしい」ときはまず**メッシュの向き**を疑うこと。
    //    ここの符号が合っていても、STL が軸を縦にして書き出されていれば
    //    首を振って見える（`cad/scripts/export_meshes.py: _leaves`）。
    roller_upper: -acc.roller,
    roller_lower: acc.roller,
    singulator: acc.singulator,
    grabber_slide: g.ext * GRAB_STROKE,
    grabber_press: g.press * PRESS_STROKE,
    fork_tilt: g.tilt * FORK_TILT,
  };
}
