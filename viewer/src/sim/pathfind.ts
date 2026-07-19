import { FIELD, type TeamId, type Vec2 } from '../config/field';
import {
  chassisObstacles,
  pointBlocked,
  segmentBlocked,
  ROBOT_R,
  type Obstacle,
} from './collision';

/**
 * 経路生成: 自陣ハーフの占有格子上で A* → 見通し線スムージング。
 * strategy.md §4.5.2 の「事前定義ウェイポイント + 固定経路」を、
 * シミュレーターでは同じ静的環境に対する A* で自動生成している (等価な安全経路)。
 */

const RES = 0.08; // Q19: 細格子で経路の質を向上
// 車体は1m角相当だが、後方LiDAR・配線・機構の突起と制御誤差を見込む。
// マージンを広めに取り、追従誤差で競技用品を擦る「狭い隙間」を経路生成の段階で通行不可にする。
const INFLATE = ROBOT_R + 0.18;

interface Grid {
  nx: number;
  nz: number;
  x0: number;
  z0: number;
  blocked: Uint8Array;
}

const gridCache = new Map<TeamId, Grid>();

function buildGrid(team: TeamId): Grid {
  const obstacles = chassisObstacles(team);
  const xMax = FIELD.w / 2 - INFLATE;
  const zNear = FIELD.podium.d / 2 + INFLATE;
  const zFar = FIELD.l / 2 - INFLATE;
  const nx = Math.ceil((xMax * 2) / RES) + 1;
  const nz = Math.ceil((zFar - zNear) / RES) + 1;
  const x0 = -xMax;
  const z0 = zNear;
  const blocked = new Uint8Array(nx * nz);
  for (let iz = 0; iz < nz; iz++) {
    for (let ix = 0; ix < nx; ix++) {
      const x = x0 + ix * RES;
      const zh = z0 + iz * RES;
      const z = team === 'blue' ? zh : -zh;
      blocked[iz * nx + ix] = pointBlocked(x, z, obstacles, INFLATE) ? 1 : 0;
    }
  }
  return { nx, nz, x0, z0, blocked };
}

function getGrid(team: TeamId): Grid {
  let g = gridCache.get(team);
  if (!g) {
    g = buildGrid(team);
    gridCache.set(team, g);
  }
  return g;
}

function toCell(g: Grid, team: TeamId, p: Vec2): [number, number] {
  const zh = team === 'blue' ? p.z : -p.z;
  let ix = Math.round((p.x - g.x0) / RES);
  let iz = Math.round((zh - g.z0) / RES);
  ix = Math.max(0, Math.min(g.nx - 1, ix));
  iz = Math.max(0, Math.min(g.nz - 1, iz));
  return [ix, iz];
}

function nearestFree(g: Grid, ix: number, iz: number): [number, number] {
  if (!g.blocked[iz * g.nx + ix]) return [ix, iz];
  for (let r = 1; r < 22; r++) {
    for (let dz = -r; dz <= r; dz++) {
      for (let dx = -r; dx <= r; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dz)) !== r) continue;
        const x = ix + dx;
        const z = iz + dz;
        if (x < 0 || z < 0 || x >= g.nx || z >= g.nz) continue;
        if (!g.blocked[z * g.nx + x]) return [x, z];
      }
    }
  }
  return [ix, iz];
}

/**
 * A* (8近傍・斜め角切り禁止)。overlay は雑巾回避などの動的コスト。
 * overlay[n] === Infinity はハード禁止 (車体が触れる範囲)。有限値は「そのマスに入る追加コスト」で、
 * 迂回できるなら迂回し、迂回不能なときだけ余裕を削って通る (ソフト回避)。
 */
function astar(
  g: Grid,
  s: [number, number],
  t: [number, number],
  overlay: Float32Array | null,
): Array<[number, number]> | null {
  const { nx, nz } = g;
  const blockedAt = (n: number): boolean => g.blocked[n] === 1 || (overlay ? overlay[n] === Infinity : false);
  const N = nx * nz;
  const open: number[] = [];
  const gCost = new Float32Array(N).fill(Infinity);
  const fCost = new Float32Array(N).fill(Infinity);
  const came = new Int32Array(N).fill(-1);
  const closed = new Uint8Array(N);
  const id = (c: [number, number]) => c[1] * nx + c[0];
  const h = (ix: number, iz: number) => Math.hypot(ix - t[0], iz - t[1]);
  const sId = id(s);
  gCost[sId] = 0;
  fCost[sId] = h(s[0], s[1]);
  open.push(sId);
  const DIRS = [
    [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
    [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
  ] as const;
  while (open.length) {
    let bi = 0;
    for (let i = 1; i < open.length; i++) if (fCost[open[i]!]! < fCost[open[bi]!]!) bi = i;
    const cur = open.splice(bi, 1)[0]!;
    if (cur === id(t)) {
      const path: Array<[number, number]> = [];
      let c = cur;
      while (c >= 0) {
        path.push([c % nx, Math.floor(c / nx)]);
        c = came[c]!;
      }
      return path.reverse();
    }
    closed[cur] = 1;
    const cx = cur % nx;
    const cz = Math.floor(cur / nx);
    for (const [dx, dz, w] of DIRS) {
      const x = cx + dx;
      const z = cz + dz;
      if (x < 0 || z < 0 || x >= nx || z >= nz) continue;
      const n = z * nx + x;
      if (blockedAt(n) || closed[n]) continue;
      if (dx !== 0 && dz !== 0 && (blockedAt(cz * nx + x) || blockedAt(z * nx + cx))) continue; // 角切り禁止
      const soft = overlay ? overlay[n]! : 0; // 雑巾近傍のソフトコスト (迂回優先)
      const ng = gCost[cur]! + w + soft;
      if (ng < gCost[n]!) {
        gCost[n] = ng;
        fCost[n] = ng + h(x, z);
        came[n] = cur;
        if (!open.includes(n)) open.push(n);
      }
    }
  }
  return null;
}

/**
 * コーナーを面取り(chamfer)して滑らかにする。メカナムでも速度ベクトルは有限加速度で
 * しか回せないため、鋭角の折れ線は「急カーブで減速→再加速」のカクつきを生む。
 * 各頂点を安全マージン内で面取りし、複数回かけて丸める。危険な面取りは行わない。
 */
function smoothCorners(
  pts: Vec2[],
  obstacles: readonly Obstacle[],
  safeInflate: number,
  ragBlocks?: (x1: number, z1: number, x2: number, z2: number) => boolean,
): Vec2[] {
  let cur = pts;
  // 丸め量を大きめ・回数を多めにして、開けた場所では大きな曲率半径のなめらかなカーブにする
  // (高曲率スポットで急ブレーキになる問題の対策)。障害物付近は安全な範囲まで自動で縮む。
  for (let pass = 0; pass < 10; pass++) {
    if (cur.length < 3) break;
    const out: Vec2[] = [cur[0]!];
    for (let i = 1; i < cur.length - 1; i++) {
      const a = cur[i - 1]!;
      const v = cur[i]!;
      const c = cur[i + 1]!;
      const dAV = Math.hypot(v.x - a.x, v.z - a.z) || 1e-9;
      const dVC = Math.hypot(c.x - v.x, c.z - v.z) || 1e-9;
      // 障害物付近でも「安全に切れる最大量」まで縮めて必ず丸める (むら防止)。
      let cut = Math.min(0.8, dAV * 0.48, dVC * 0.48);
      let p1: Vec2 | null = null;
      let p2: Vec2 | null = null;
      while (cut > 0.02) {
        const q1 = { x: v.x + ((a.x - v.x) / dAV) * cut, z: v.z + ((a.z - v.z) / dAV) * cut };
        const q2 = { x: v.x + ((c.x - v.x) / dVC) * cut, z: v.z + ((c.z - v.z) / dVC) * cut };
        if (
          !segmentBlocked(q1.x, q1.z, q2.x, q2.z, obstacles, safeInflate) &&
          !(ragBlocks && ragBlocks(q1.x, q1.z, q2.x, q2.z))
        ) {
          p1 = q1;
          p2 = q2;
          break;
        }
        cut *= 0.6;
      }
      if (p1 && p2) out.push(p1, p2);
      else out.push(v); // どうしても切れない鋭角のみ残す (ほぼ発生しない)
    }
    out.push(cur[cur.length - 1]!);
    cur = out;
  }
  return cur;
}

// 床雑巾の回避半径。ハード禁止(車体が触れる範囲)と、その外側のソフトコスト帯に分ける。
// ハード内は通行不可、ソフト帯は「入るたび追加コスト」で迂回を強く優先させる。迂回路がある限り
// 必ず迂回し、迂回不能なときだけ余裕を削る (旧: 半径を大きく取りすぎてA*が失敗→無回避直進で踏んでいた)。
const RAG_HARD = ROBOT_R + 0.2; // 0.70: 車体(半径0.5)+雑巾(半径~0.15)+5cm余裕 = ハード禁止
const RAG_SOFT = ROBOT_R + 0.8; // 1.30: ここまではソフトコストで迂回を優先
const RAG_PENALTY = 40; // ソフト帯1マスあたりの追加コスト (1マス≈1.0)。迂回を強く優先

/**
 * from→to の安全経路 (滑らかな中継点列)。到達不能時はその場停止。
 * avoid: 回避したい床の雑巾など動的な点 (床雑巾回避モードで渡す)。
 */
export function planPath(team: TeamId, from: Vec2, to: Vec2, avoid: readonly Vec2[] = []): Vec2[] {
  const g = getGrid(team);
  const obstacles = chassisObstacles(team);
  // 床雑巾を避ける場合: 線分がどの雑巾にも clear 未満に近づかないかを追加チェック
  const nearRag = (x1: number, z1: number, x2: number, z2: number, clear: number): boolean => {
    if (avoid.length === 0 || clear <= 0) return false;
    const len = Math.hypot(x2 - x1, z2 - z1);
    const n = Math.max(2, Math.ceil(len / 0.08));
    for (let k = 0; k <= n; k++) {
      const t = k / n;
      const px = x1 + (x2 - x1) * t;
      const pz = z1 + (z2 - z1) * t;
      for (const r of avoid) if (Math.hypot(px - r.x, pz - r.z) < clear) return true;
    }
    return false;
  };
  const targetBlocked = pointBlocked(to.x, to.z, obstacles, INFLATE);
  if (
    !targetBlocked &&
    !segmentBlocked(from.x, from.z, to.x, to.z, obstacles, INFLATE) &&
    !nearRag(from.x, from.z, to.x, to.z, RAG_HARD)
  ) {
    return [{ x: from.x, z: from.z }, to];
  }
  const s = nearestFree(g, ...toCell(g, team, from));
  const t = nearestFree(g, ...toCell(g, team, to));
  // A* の格子へ雑巾回避コストを重畳。ハード内=通行不可(Infinity)、ソフト帯=追加コスト。
  // 始点/終点セルは踏み外せるよう常に開けておく。
  const buildOverlay = (): Float32Array => {
    const ov = new Float32Array(g.nx * g.nz);
    const rc = Math.ceil(RAG_SOFT / RES);
    for (const r of avoid) {
      const zh = team === 'blue' ? r.z : -r.z;
      const cix = Math.round((r.x - g.x0) / RES);
      const ciz = Math.round((zh - g.z0) / RES);
      for (let dz = -rc; dz <= rc; dz++)
        for (let dx = -rc; dx <= rc; dx++) {
          const x = cix + dx;
          const z = ciz + dz;
          if (x < 0 || z < 0 || x >= g.nx || z >= g.nz) continue;
          const idx = z * g.nx + x;
          if (ov[idx] === Infinity) continue;
          const dist = Math.hypot(dx * RES, dz * RES);
          if (dist < RAG_HARD) ov[idx] = Infinity;
          else if (dist < RAG_SOFT) {
            const pen = RAG_PENALTY * (1 - (dist - RAG_HARD) / (RAG_SOFT - RAG_HARD));
            if (pen > ov[idx]!) ov[idx] = pen;
          }
        }
    }
    ov[s[1] * g.nx + s[0]] = 0;
    ov[t[1] * g.nx + t[0]] = 0;
    return ov;
  };

  // ソフトコスト付きで一度探索 → 迂回路があれば必ず迂回する。ハードで到達不能なときだけ無回避。
  let cells: [number, number][] | null = null;
  let usedOverlay = false;
  if (avoid.length) {
    cells = astar(g, s, t, buildOverlay());
    usedOverlay = cells !== null;
  }
  if (!cells) cells = astar(g, s, t, null); // 回避不能 or 回避不要: 静的経路
  if (!cells) return [from];
  const effClear = usedOverlay ? RAG_HARD : 0;

  const pts: Vec2[] = cells.map(([ix, iz]) => ({
    x: g.x0 + ix * RES,
    z: team === 'blue' ? g.z0 + iz * RES : -(g.z0 + iz * RES),
  }));
  // 目標を経路の最終点として必ず含める。障害物のINFLATEマージン内で終える(=手前で止まる)と
  // ロボットが慣性で経路端を越えて真の目標へ進み「オーバー」して見えるため。ただし障害物の奥で
  // 到達不能(最近セルから遠い)なときだけは含めない。
  const lastCell = pts[pts.length - 1];
  if (!targetBlocked) {
    pts.push(to);
  } else if (lastCell && Math.hypot(lastCell.x - to.x, lastCell.z - to.z) < 0.8) {
    pts.push(to);
  }
  // 見通し線スムージング (折れ線を最小頂点に。雑巾に近づく短絡はしない)
  const simplified: Vec2[] = [];
  let anchor: Vec2 = from;
  let i = 0;
  while (i < pts.length) {
    let j = pts.length - 1;
    for (; j > i; j--) {
      const p = pts[j]!;
      if (
        !segmentBlocked(anchor.x, anchor.z, p.x, p.z, obstacles, INFLATE) &&
        !nearRag(anchor.x, anchor.z, p.x, p.z, effClear)
      )
        break;
    }
    anchor = pts[j]!;
    simplified.push(anchor);
    i = j + 1;
  }
  // 現在地から始めてコーナーを面取り (加速度に優しい滑らかな経路)。面取りも雑巾を跨がない。
  const full = [{ x: from.x, z: from.z }, ...simplified];
  const ragBlocks = effClear > 0 ? (x1: number, z1: number, x2: number, z2: number) => nearRag(x1, z1, x2, z2, effClear) : undefined;
  return smoothCorners(full, obstacles, ROBOT_R + 0.12, ragBlocks);
}
