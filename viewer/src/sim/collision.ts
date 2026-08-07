import { FIELD, RED_SIDE, DIMS, mirror, type TeamId, type Vec2 } from '../config/field';

/** 2D 衝突・遮蔽・LiDAR 用の障害物モデル (strategy.md §4.5.2「静的既知環境」) */

export type Obstacle =
  | { kind: 'circle'; x: number; z: number; r: number; tag: string }
  | { kind: 'rect'; x: number; z: number; hw: number; hd: number; tag: string };

/**
 * 機体を円で近似した半径 [m]。⚠ **保守側の簡略化**（実体は 842×722）。
 * 外接円は √(0.421²+0.361²)=0.555 なので、0.5 でも既に少し攻めている。
 */
export const ROBOT_R = 0.5;

/**
 * 「向きを合わせて正対で寄せる」ときの実効半径 [m]。
 *
 * ⚠ 円で近似していると、**機体をどう向けても同じだけ離れる**ことになる。
 *   補充机には後ろ向きで正対して寄せる（ToF 2 個で平行を出す。strategy.md
 *   §位置決め）ので、効くのは車体の後端 421mm であって外接円 500mm ではない。
 *   円のままだと机から 79mm 余計に離れ、そのぶん櫛歯が山に届かない。
 */
export const ROBOT_HALF_LEN = 0.421;

/** 正対とみなす角度差 [rad]。ToF での平行合わせ ±2°（strategy.md）に余裕を見た値 */
export const SQUARE_TOL = 0.12;

function sideProps(sign: 1 | -1): Obstacle[] {
  // sign=-1: 赤陣 (RED_SIDE そのまま) / sign=1: 青陣 (鏡映 = z反転)
  const m = (p: Vec2): Vec2 => (sign === -1 ? p : mirror(p));
  const P = RED_SIDE;
  const t = sign === -1 ? 'red' : 'blue';
  // 机: W650(x)×D450(z) (公式図精査 2026-07-04)。+マージン
  return [
    { kind: 'circle', ...m(P.b1), r: DIMS.bucketRimR + 0.03, tag: `${t}:b1` },
    { kind: 'rect', ...m(P.b2), hw: 0.16, hd: 0.16, tag: `${t}:b2` },
    { kind: 'rect', ...m(P.b3), hw: 0.16, hd: 0.16, tag: `${t}:b3` },
    { kind: 'rect', ...m(P.desk1), hw: 0.345, hd: 0.245, tag: `${t}:desk1` },
    { kind: 'rect', ...m(P.desk2), hw: 0.345, hd: 0.245, tag: `${t}:desk2` },
    { kind: 'rect', ...m(P.chair), hw: 0.19, hd: 0.21, tag: `${t}:chair` },
    { kind: 'rect', ...m(P.flag), hw: 0.2, hd: 0.2, tag: `${t}:flag` }, // 土台390角+マージン
    { kind: 'rect', ...m(P.resup), hw: 0.345, hd: 0.245, tag: `${t}:resup` },
    { kind: 'rect', ...m(P.control), hw: 0.345, hd: 0.245, tag: `${t}:control` },
  ];
}

export const PODIUM_OBS: Obstacle = {
  kind: 'rect',
  x: 0,
  z: 0,
  hw: FIELD.podium.w / 2,
  hd: FIELD.podium.d / 2,
  tag: 'podium',
};

const CACHE = new Map<string, Obstacle[]>();

/** チームの車体が衝突しうる障害物 (自陣の造作物 + 教壇) */
export function chassisObstacles(team: TeamId): Obstacle[] {
  const key = `chassis:${team}`;
  let v = CACHE.get(key);
  if (!v) {
    v = [...sideProps(team === 'blue' ? 1 : -1), PODIUM_OBS];
    CACHE.set(key, v);
  }
  return v;
}

/** 全静的物 (LiDAR 用) */
export function allStaticObstacles(): Obstacle[] {
  const key = 'all';
  let v = CACHE.get(key);
  if (!v) {
    v = [...sideProps(1), ...sideProps(-1), PODIUM_OBS];
    CACHE.set(key, v);
  }
  return v;
}

/** 円 (ロボット) を障害物から押し出す。返り値は補正後座標と接触の有無 */
export function resolveCircle(
  x: number,
  z: number,
  r: number,
  obstacles: readonly Obstacle[],
): { x: number; z: number; hit: boolean; nx: number; nz: number; tag: string; depth: number } {
  let hit = false;
  let nx = 0;
  let nz = 0;
  let tag = '';
  let depth = 0;
  for (const o of obstacles) {
    if (o.kind === 'circle') {
      const dx = x - o.x;
      const dz = z - o.z;
      const d = Math.hypot(dx, dz);
      const min = r + o.r;
      if (d < min && d > 1e-6) {
        const push = min - d;
        x += (dx / d) * push;
        z += (dz / d) * push;
        nx = dx / d;
        nz = dz / d;
        hit = true;
        if (push > depth) {
          depth = push;
          tag = o.tag;
        }
      }
    } else {
      const cx = Math.max(o.x - o.hw, Math.min(x, o.x + o.hw));
      const cz = Math.max(o.z - o.hd, Math.min(z, o.z + o.hd));
      const dx = x - cx;
      const dz = z - cz;
      const d = Math.hypot(dx, dz);
      if (d < r) {
        if (d > 1e-6) {
          const push = r - d;
          x += (dx / d) * push;
          z += (dz / d) * push;
          nx = dx / d;
          nz = dz / d;
          if (push > depth) {
            depth = push;
            tag = o.tag;
          }
        } else {
          // 中心が矩形内: 最小軸で脱出
          const exL = x - (o.x - o.hw - r);
          const exR = o.x + o.hw + r - x;
          const exD = z - (o.z - o.hd - r);
          const exU = o.z + o.hd + r - z;
          const m = Math.min(exL, exR, exD, exU);
          if (m === exL) { x -= exL; nx = -1; nz = 0; }
          else if (m === exR) { x += exR; nx = 1; nz = 0; }
          else if (m === exD) { z -= exD; nx = 0; nz = -1; }
          else { z += exU; nx = 0; nz = 1; }
        }
        hit = true;
      }
    }
  }
  return { x, z, hit, nx, nz, tag, depth };
}

/** 点が障害物 (inflate 分膨張) 内か */
export function pointBlocked(
  x: number,
  z: number,
  obstacles: readonly Obstacle[],
  inflate: number,
): boolean {
  for (const o of obstacles) {
    if (o.kind === 'circle') {
      if (Math.hypot(x - o.x, z - o.z) < o.r + inflate) return true;
    } else {
      if (Math.abs(x - o.x) < o.hw + inflate && Math.abs(z - o.z) < o.hd + inflate) return true;
    }
  }
  return false;
}

/** 線分が障害物 (inflate 膨張) と交差するか (経路スムージング用) */
export function segmentBlocked(
  x1: number,
  z1: number,
  x2: number,
  z2: number,
  obstacles: readonly Obstacle[],
  inflate: number,
): boolean {
  const len = Math.hypot(x2 - x1, z2 - z1);
  const n = Math.max(2, Math.ceil(len / 0.08));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (pointBlocked(x1 + (x2 - x1) * t, z1 + (z2 - z1) * t, obstacles, inflate)) return true;
  }
  return false;
}

export interface Seg {
  x1: number;
  z1: number;
  x2: number;
  z2: number;
}

/** LiDAR 用の線分集合へ変換 */
export function obstacleSegments(obstacles: readonly Obstacle[]): Seg[] {
  const segs: Seg[] = [];
  for (const o of obstacles) {
    if (o.kind === 'rect') {
      const { x, z, hw, hd } = o;
      segs.push(
        { x1: x - hw, z1: z - hd, x2: x + hw, z2: z - hd },
        { x1: x + hw, z1: z - hd, x2: x + hw, z2: z + hd },
        { x1: x + hw, z1: z + hd, x2: x - hw, z2: z + hd },
        { x1: x - hw, z1: z + hd, x2: x - hw, z2: z - hd },
      );
    } else {
      const n = 10;
      for (let i = 0; i < n; i++) {
        const a1 = (i / n) * Math.PI * 2;
        const a2 = ((i + 1) / n) * Math.PI * 2;
        segs.push({
          x1: o.x + Math.cos(a1) * o.r,
          z1: o.z + Math.sin(a1) * o.r,
          x2: o.x + Math.cos(a2) * o.r,
          z2: o.z + Math.sin(a2) * o.r,
        });
      }
    }
  }
  return segs;
}

/** フィールド外周フェンスの線分 */
export function fenceSegments(): Seg[] {
  const hx = FIELD.w / 2;
  const hz = FIELD.l / 2;
  return [
    { x1: -hx, z1: -hz, x2: hx, z2: -hz },
    { x1: hx, z1: -hz, x2: hx, z2: hz },
    { x1: hx, z1: hz, x2: -hx, z2: hz },
    { x1: -hx, z1: hz, x2: -hx, z2: -hz },
  ];
}
