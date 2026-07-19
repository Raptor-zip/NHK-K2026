import type { Rng } from './rng';
import { FIELD, RED_SIDE, DIMS, mirror, type Vec2, REFLECTORS_RED } from '../config/field';
import { fenceSegments, type Seg } from './collision';

/**
 * 2D LiDAR シミュレーション — 北陽 UST-20LX 想定 (strategy.md §4.5.7)
 *   視野角 270°(前方中心。下段は前後2台で360°を補う) / 測距 0.06〜20m / 40Hz
 *   角度分解能は実機0.25°(1081步)だが、シムでは1°=271本に間引き
 *
 * リアリティ要素:
 *   - 距離比例ノイズ: σ = 10mm + 5mm/m (カタログ精度±40mmと整合)
 *   - 距離比例ドロップアウト (遠距離ほど反射強度低下で欠測)
 *   - スキャン歪み: 1回転25ms中の自己移動により、レイごとに原点ポーズが異なる
 *     (実機ドライバと同じく「全レイ同一ポーズ」を仮定して復元するとズレて見える)
 *
 * 搭載高さ = スキャン面 0.12 m (フェンスH150/教壇H200を観測するため)。
 * この高さで見えるもの: フェンス・教壇・バケツ(H255)・台・旗土台・机/椅子の「脚」。
 */
export const LIDAR_SCAN_HEIGHT = 0.12;

export const LIDAR = {
  model: 'UST-20LX',
  fovDeg: 270,
  rays: 271, // 1°刻み (実機は0.25°)
  range: 20,
  minRange: 0.06,
  scanHz: 40,
  noiseBase: 0.01,
  noisePerM: 0.005,
} as const;

/** 後方互換 (旧コード参照用) */
export const LIDAR_RANGE = LIDAR.range;

export interface Pose2 {
  x: number;
  z: number;
  theta: number;
}

export interface LidarScan {
  /**
   * センサー出力そのもの: レイ i の相対角 angleOf(i) と測距 ranges[i]。
   * 欠測・レンジ外は range (=20) が入る。
   */
  ranges: Float32Array;
  rays: number;
  /** センサーの車体取付方位 (0=前向き, π=後ろ向き)。下段は前後2台構成 */
  mountYaw: number;
  /** スキャン終了時点の真ポーズ (シム内部でのみ既知。推定器は使用不可) */
  truePose: Pose2;
  /** スキャン中の移動歪みの元になった速度 (デバッグ用) */
  vel: { vx: number; vz: number; omega: number };
  t: number;
}

/** レイ i のセンサー相対角 (前方0, 左+) */
export function angleOf(i: number): number {
  const fov = (LIDAR.fovDeg * Math.PI) / 180;
  return -fov / 2 + (i / (LIDAR.rays - 1)) * fov;
}

/**
 * 実機ドライバと同じ「スキャン終了ポーズに全レイが属す」仮定でセンサー座標へ復元。
 * 移動中はこの仮定が破れて歪む — それがそのまま点群に現れる (T14)。
 * 戻り値: ロボット座標系 (前方+z, 右+x) の点列 [x0,z0,...] と有効数 n
 */
export function scanToLocalPoints(scan: LidarScan): { pts: Float32Array; n: number } {
  const pts = new Float32Array(scan.rays * 2);
  let n = 0;
  for (let i = 0; i < scan.rays; i++) {
    const r = scan.ranges[i]!;
    if (r >= LIDAR.range - 1e-3 || r <= LIDAR.minRange) continue;
    const a = scan.mountYaw + angleOf(i);
    pts[n * 2] = Math.sin(a) * r;
    pts[n * 2 + 1] = Math.cos(a) * r;
    n++;
  }
  return { pts, n };
}

/** ロボット座標系の点をポーズで世界座標へ */
export function localToWorld(px: number, pz: number, pose: Pose2): [number, number] {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return [pose.x + px * c + pz * s, pose.z - px * s + pz * c];
}

function rect(cx: number, cz: number, hw: number, hd: number): Seg[] {
  return [
    { x1: cx - hw, z1: cz - hd, x2: cx + hw, z2: cz - hd },
    { x1: cx + hw, z1: cz - hd, x2: cx + hw, z2: cz + hd },
    { x1: cx + hw, z1: cz + hd, x2: cx - hw, z2: cz + hd },
    { x1: cx - hw, z1: cz + hd, x2: cx - hw, z2: cz - hd },
  ];
}

function ngon(cx: number, cz: number, r: number, n = 10): Seg[] {
  const out: Seg[] = [];
  for (let i = 0; i < n; i++) {
    const a1 = (i / n) * Math.PI * 2;
    const a2 = ((i + 1) / n) * Math.PI * 2;
    out.push({
      x1: cx + Math.cos(a1) * r,
      z1: cz + Math.sin(a1) * r,
      x2: cx + Math.cos(a2) * r,
      z2: cz + Math.sin(a2) * r,
    });
  }
  return out;
}

/** スキャン面 0.12 m で実際に見える静的物体の線分集合 (=自己位置推定の地図でもある) */
function buildScanSegs(): Seg[] {
  const segs: Seg[] = [...fenceSegments()];
  segs.push(...rect(0, 0, 10.5 / 2, 0.3)); // 教壇 (H200)
  const deskLegs = (p: Vec2): void => {
    // 机: W650(x)×D450(z) → 脚は (±0.29, ±0.19)
    for (const [lx, lz] of [
      [-0.29, -0.19],
      [0.29, -0.19],
      [-0.29, 0.19],
      [0.29, 0.19],
    ] as const) {
      segs.push(...rect(p.x + lx, p.z + lz, 0.018, 0.018));
    }
  };
  const chairLegs = (p: Vec2): void => {
    for (const [lx, lz] of [
      [-0.15, -0.17],
      [0.15, -0.17],
      [-0.15, 0.17],
      [0.15, 0.17],
    ] as const) {
      segs.push(...rect(p.x + lx, p.z + lz, 0.013, 0.013));
    }
  };
  // 自陣 (青) 側の造作物 + 相手陣の大物。ICP 地図と同一にする。
  const S = (p: Vec2): Vec2 => mirror(p);
  const P = RED_SIDE;
  // b1 は床置きの透明バケツ本体 (台なし)。透明樹脂は赤外を透過し実体として返らないので
  // 不透明マップからは外し、getTransparentSegs 側で散乱ノイズとして扱う。
  segs.push(...rect(S(P.b2).x, S(P.b2).z, 0.15, 0.15));
  segs.push(...rect(S(P.b3).x, S(P.b3).z, 0.15, 0.15));
  segs.push(...rect(S(P.flag).x, S(P.flag).z, 0.195, 0.195)); // 旗土台390角 (円でなく四角)
  deskLegs(S(P.desk1));
  deskLegs(S(P.desk2));
  deskLegs(S(P.resup));
  deskLegs(S(P.control));
  chairLegs(S(P.chair));
  // Q2: 両陣コントロールステーション脇の反射材マーカー柱 (h=0.78, 両スキャン面にかかる)
  for (const m of REFLECTORS_RED) {
    segs.push(...ngon(m.x, m.z, 0.03, 6));
    const b = mirror(m);
    segs.push(...ngon(b.x, b.z, 0.03, 6));
  }
  // 相手陣側で見える大物 (旗土台・台) — 前方視野に入るため地図に含める。b1(透明バケツ)は除外。
  segs.push(...rect(P.b2.x, P.b2.z, 0.15, 0.15));
  segs.push(...rect(P.b3.x, P.b3.z, 0.15, 0.15));
  segs.push(...rect(P.flag.x, P.flag.z, 0.195, 0.195)); // 旗土台390角 (円でなく四角)
  return segs;
}

let scanSegs: Seg[] | null = null;
export function getMapSegs(): Seg[] {
  if (!scanSegs) scanSegs = buildScanSegs();
  return scanSegs;
}

/**
 * 上段LiDAR (相手検出用, スキャン面 0.50 m)。
 * 下段(0.12m)では相手ロボットが教壇(H200)の陰に隠れて原理的に見えない — シムで発覚した
 * 実設計知見 (strategy.md §4.5.5)。0.50mでは逆にフェンス・教壇が消え、
 * 見えるのは 台(H600)・椅子・旗ポール・机脚の一部と「相手車体」だけ → クラスタリングに最適。
 */
export const LIDAR_UPPER_HEIGHT = 0.5;

function buildUpperSegs(): Seg[] {
  const segs: Seg[] = [];
  for (const side of [1, -1] as const) {
    const S = (p: Vec2): Vec2 => (side === -1 ? p : mirror(p));
    const P = RED_SIDE;
    segs.push(...rect(S(P.b2).x, S(P.b2).z, 0.15, 0.15)); // 台H600のみ0.5mで見える
    segs.push(...ngon(S(P.flag).x, S(P.flag).z, 0.028, 8)); // 旗ポール
    segs.push(...rect(S(P.chair).x, S(P.chair).z, 0.19, 0.21)); // 椅子 (座面0.46+背もたれ)
    for (const d of [P.desk1, P.desk2, P.resup, P.control]) {
      const q = S(d);
      for (const [lx, lz] of [
        [-0.29, -0.19],
        [0.29, -0.19],
        [-0.29, 0.19],
        [0.29, 0.19],
      ] as const) {
        segs.push(...rect(q.x + lx, q.z + lz, 0.018, 0.018));
      }
    }
    // Q2: 反射材マーカー柱 — 上段地図に含めることで相手クラスタ誤検出も防ぐ
    for (const m of REFLECTORS_RED) {
      const q = S(m);
      segs.push(...ngon(q.x, q.z, 0.03, 6));
    }
  }
  return segs;
}

let upperSegs: Seg[] | null = null;
export function getUpperSegs(): Seg[] {
  if (!upperSegs) upperSegs = buildUpperSegs();
  return upperSegs;
}

function raySeg(ox: number, oz: number, dx: number, dz: number, s: Seg): number | null {
  // o + t·d = p1 + u·e を解く。u = (p1−o)×d / den
  const ex = s.x2 - s.x1;
  const ez = s.z2 - s.z1;
  const den = dx * ez - dz * ex; // d×e
  if (Math.abs(den) < 1e-9) return null;
  const t = ((s.x1 - ox) * ez - (s.z1 - oz) * ex) / den;
  const u = ((s.x1 - ox) * dz - (s.z1 - oz) * dx) / den;
  if (t > 0.05 && u >= 0 && u <= 1) return t;
  return null;
}

function opponentSegs(opponent: Pose2 | null): Seg[] {
  const dyn: Seg[] = [];
  if (!opponent) return dyn;
  const h = 0.38;
  const c = Math.cos(opponent.theta);
  const s = Math.sin(opponent.theta);
  const corners = [
    [-h, -h],
    [h, -h],
    [h, h],
    [-h, h],
  ].map(([px, pz]) => [opponent.x + px! * c + pz! * s, opponent.z - px! * s + pz! * c]);
  for (let i = 0; i < 4; i++) {
    const a = corners[i]!;
    const b = corners[(i + 1) % 4]!;
    dyn.push({ x1: a[0]!, z1: a[1]!, x2: b[0]!, z2: b[1]! });
  }
  return dyn;
}

function castOne(ox: number, oz: number, ang: number, segs: Seg[], dyn: Seg[]): number {
  const dx = Math.sin(ang);
  const dz = Math.cos(ang);
  let best: number = LIDAR.range;
  for (const s of segs) {
    const t = raySeg(ox, oz, dx, dz, s);
    if (t !== null && t < best) best = t;
  }
  for (const s of dyn) {
    const t = raySeg(ox, oz, dx, dz, s);
    if (t !== null && t < best) best = t;
  }
  return best;
}

/**
 * 1スキャン実行。スキャン中の移動 (vel) によるレイごとのポーズ差 = 歪みを再現。
 * pose はスキャン「終了」時点の真ポーズ。レイ i は t_off = (i/rays − 1)·(1/40Hz) 秒前に発射。
 */
export function castScan(
  pose: Pose2,
  vel: { vx: number; vz: number; omega: number },
  opponent: Pose2 | null,
  rng: Rng,
  t: number,
  segsOverride?: Seg[],
  mountYaw = 0,
): LidarScan {
  const segs = segsOverride ?? getMapSegs();
  const dyn = opponentSegs(opponent);
  const ranges = new Float32Array(LIDAR.rays);
  const scanT = 1 / LIDAR.scanHz;
  for (let i = 0; i < LIDAR.rays; i++) {
    const frac = i / (LIDAR.rays - 1);
    const tOff = (frac - 1) * scanT; // 過去に遡る
    const ox = pose.x + vel.vx * tOff;
    const oz = pose.z + vel.vz * tOff;
    const th = pose.theta + vel.omega * tOff;
    const trueR = castOne(ox, oz, th + mountYaw + angleOf(i), segs, dyn);
    if (trueR >= LIDAR.range) {
      ranges[i] = LIDAR.range;
      continue;
    }
    // 距離比例ドロップアウト (反射強度低下)
    const pDrop = 0.006 + 0.028 * (trueR / LIDAR.range);
    if (rng() < pDrop) {
      ranges[i] = LIDAR.range;
      continue;
    }
    // 距離比例ノイズ
    const sigma = LIDAR.noiseBase + LIDAR.noisePerM * trueR;
    const g = (rng() + rng() + rng() - 1.5) / 0.6124; // 疑似ガウス (σ≈1)
    ranges[i] = Math.max(LIDAR.minRange, trueR + g * sigma);
  }
  return { ranges, rays: LIDAR.rays, mountYaw, truePose: { ...pose }, vel: { ...vel }, t };
}
