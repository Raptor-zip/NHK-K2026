/**
 * ロボットの実体（CAD の当たり判定そのもの）。**自動生成。手で書き換えない。**
 *
 *     npm run urdf:shape        # cad/urdf/tr.urdf から作り直す
 *
 * LiDAR シムはこの箱をスキャン面で切って相手機の断面にする。以前は 0.76m 角の
 * 正方形 1 つで近似していたので、実際には見えないはずの向きから見え、
 * 逆に机の脚のような細い物と区別が付かなかった。
 *
 * 座標はビューア局所（X 左 / Y 上＝床から / Z 前）で、単位は m。
 * 全関節 0 の姿勢で取ってある（動く部品はどれもスキャン面より上）。
 * ⚠ URDF は生成物で git に入らないので、**この表が版に残る唯一の実体**。
 *   `npm run urdf:check` が毎回 URDF から作り直して一致を確かめる。
 */
export interface ShapeBox {
  x: number;
  y: number;
  z: number;
  sx: number;
  sy: number;
  sz: number;
  link: string;
}

export const ROBOT_BOXES: readonly ShapeBox[] = [
  { x: 0, y: 0.08, z: 0, sx: 0.722, sy: 0.11, sz: 0.842, link: 'base_link' },
  { x: 0.35, y: 0.4885, z: 0.035, sx: 0.024, sy: 0.707, sz: 0.73, link: 'base_link' },
  { x: -0.35, y: 0.4885, z: 0.035, sx: 0.024, sy: 0.707, sz: 0.73, link: 'base_link' },
  { x: 0, y: 0.582, z: -0.225, sx: 0.628, sy: 0.02, sz: 0.428, link: 'base_link' },
  { x: 0.312, y: 0.652, z: -0.225, sx: 0.004, sy: 0.12, sz: 0.428, link: 'base_link' },
  { x: -0.312, y: 0.652, z: -0.225, sx: 0.004, sy: 0.12, sz: 0.428, link: 'base_link' },
  { x: 0, y: 0.652, z: -0.437, sx: 0.628, sy: 0.12, sz: 0.004, link: 'base_link' },
  { x: 0, y: 0.652, z: -0.013, sx: 0.628, sy: 0.12, sz: 0.004, link: 'base_link' },
  { x: 0, y: 0.76, z: 0.072, sx: 0.632, sy: 0.14, sz: 0.076, link: 'base_link' },
  { x: 0, y: 0.832, z: 0.235, sx: 0.704, sy: 0.02, sz: 0.24, link: 'base_link' },
  { x: 0.35, y: 1.007, z: -0.275, sx: 0.024, sy: 0.33, sz: 0.022, link: 'base_link' },
  { x: -0.35, y: 1.007, z: -0.275, sx: 0.024, sy: 0.33, sz: 0.022, link: 'base_link' },
  { x: 0, y: 1.1625, z: -0.1075, sx: 0.344, sy: 0.025, sz: 0.355, link: 'base_link' },
  { x: 0, y: 1.3025, z: -0.07, sx: 0.28, sy: 0.255, sz: 0.28, link: 'base_link' },
  { x: 0.175, y: 0.47, z: 0.25, sx: 0.34, sy: 0.64, sz: 0.33, link: 'base_link' },
  { x: 0, y: 0.789, z: -0.055, sx: 0.4, sy: 0.052, sz: 0.03, link: 'base_link' },
  { x: 0, y: 0.0961, z: 0.4385, sx: 0.084, sy: 0.099, sz: 0.077, link: 'base_link' },
  { x: 0, y: 0.0961, z: -0.4385, sx: 0.084, sy: 0.099, sz: 0.077, link: 'base_link' },
  { x: 0.355, y: 0.492, z: 0.423, sx: 0.084, sy: 0.16, sz: 0.066, link: 'base_link' },
  { x: 0.3, y: 0.05, z: 0.3, sx: 0.05, sy: 0.1, sz: 0.1, link: 'wheel_fl' },
  { x: -0.3, y: 0.05, z: 0.3, sx: 0.05, sy: 0.1, sz: 0.1, link: 'wheel_fr' },
  { x: 0.3, y: 0.05, z: -0.3, sx: 0.05, sy: 0.1, sz: 0.1, link: 'wheel_rl' },
  { x: -0.3, y: 0.05, z: -0.3, sx: 0.05, sy: 0.1, sz: 0.1, link: 'wheel_rr' },
  { x: 0, y: 0.929376, z: 0.238144, sx: 0.641, sy: 0.182499, sz: 0.386289, link: 'turret_yaw' },
  { x: 0, y: 1.017878, z: 0.239618, sx: 0.616, sy: 0.293755, sz: 0.222763, link: 'shooter_pitch' },
  { x: 0, y: 1.04575, z: 0.235, sx: 0.602, sy: 0.09, sz: 0.09, link: 'roller_upper' },
  { x: 0, y: 0.95425, z: 0.235, sx: 0.602, sy: 0.09, sz: 0.09, link: 'roller_lower' },
  { x: 0.02, y: 0.72, z: -0.01, sx: 0.66, sy: 0.04, sz: 0.04, link: 'singulator' },
  { x: 0, y: 0.778, z: 0.0046, sx: 0.626, sy: 0.03, sz: 0.09, link: 'grabber_slide' },
  { x: 0.3155, y: 0.7955, z: -0.1454, sx: 0.013, sy: 0.065, sz: 0.26, link: 'grabber_slide' },
  { x: -0.3155, y: 0.7955, z: -0.1454, sx: 0.013, sy: 0.065, sz: 0.26, link: 'grabber_slide' },
  { x: 0, y: 0.915, z: -0.2154, sx: 0.38, sy: 0.026, sz: 0.22, link: 'grabber_press' },
  { x: -0.205, y: 0.76975, z: -0.1454, sx: 0.02, sy: 0.0025, sz: 0.26, link: 'fork_tilt' },
  { x: -0.1025, y: 0.76975, z: -0.1454, sx: 0.02, sy: 0.0025, sz: 0.26, link: 'fork_tilt' },
  { x: 0, y: 0.76975, z: -0.1454, sx: 0.02, sy: 0.0025, sz: 0.26, link: 'fork_tilt' },
  { x: 0.1025, y: 0.76975, z: -0.1454, sx: 0.02, sy: 0.0025, sz: 0.26, link: 'fork_tilt' },
  { x: 0.205, y: 0.76975, z: -0.1454, sx: 0.02, sy: 0.0025, sz: 0.26, link: 'fork_tilt' },
  { x: -0.205, y: 0.76975, z: -0.2904, sx: 0.02, sy: 0.0025, sz: 0.03, link: 'fork_tilt' },
  { x: -0.1025, y: 0.76975, z: -0.2904, sx: 0.02, sy: 0.0025, sz: 0.03, link: 'fork_tilt' },
  { x: 0, y: 0.76975, z: -0.2904, sx: 0.02, sy: 0.0025, sz: 0.03, link: 'fork_tilt' },
  { x: 0.1025, y: 0.76975, z: -0.2904, sx: 0.02, sy: 0.0025, sz: 0.03, link: 'fork_tilt' },
  { x: 0.205, y: 0.76975, z: -0.2904, sx: 0.02, sy: 0.0025, sz: 0.03, link: 'fork_tilt' },
  { x: 0, y: 0.783, z: -0.0304, sx: 0.62, sy: 0.024, sz: 0.03, link: 'fork_tilt' },
] as const;

/** LiDAR の取付（ビューア局所 m / rad）。走査面の高さは y そのもの。 */
export interface LidarMount {
  name: string;
  x: number;
  y: number;
  z: number;
  yaw: number;
}

/**
 * ⚠ **走査原点は車体中心ではない。** 前後の下段は 908mm 離れている。
 *   中心 1 点から飛ばすと、実機なら片方からしか見えない物が両方から
 *   見えることになり、自己位置推定が実機より当たってしまう。
 */
export const LIDAR_MOUNTS: readonly LidarMount[] = [
  { name: 'lidar_low_front', x: 0, y: 0.0696, z: 0.447, yaw: 0 },
  { name: 'lidar_low_rear', x: 0, y: 0.0696, z: -0.447, yaw: 3.14159 },
  { name: 'lidar_high', x: 0.355, y: 0.5, z: 0.426, yaw: 0 },
] as const;

export function lidarMount(name: string): LidarMount {
  const m = LIDAR_MOUNTS.find((v) => v.name === name);
  if (!m) throw new Error(`URDF に LiDAR フレーム ${name} が無い`);
  return m;
}

/** 高さ h [m] でロボットを水平に切った断面（軸並行の長方形の集まり） */
export function sectionRects(h: number): { cx: number; cz: number; hw: number; hd: number }[] {
  const out: { cx: number; cz: number; hw: number; hd: number }[] = [];
  for (const b of ROBOT_BOXES) {
    if (h < b.y - b.sy / 2 || h > b.y + b.sy / 2) continue;
    out.push({ cx: b.x, cz: b.z, hw: b.sx / 2, hd: b.sz / 2 });
  }
  return out;
}
