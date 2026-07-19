/**
 * 経路追従の膨らみ(オーバー)を分類計測する診断。カーブ区間 / ゴール接近 / 直線 で分け、
 * さらに「ゴールを行き過ぎて戻る(overshoot)」量も測る。
 *   npx tsx scripts/pathdiag.ts [crossP crossD safeAccF lookaheadK]
 */
import { MatchSim } from '../src/sim/match';
import { clonedParams } from '../src/config/params';

function distToSeg(px: number, pz: number, ax: number, az: number, bx: number, bz: number): number {
  const ex = bx - ax;
  const ez = bz - az;
  const l2 = ex * ex + ez * ez || 1e-9;
  let t = ((px - ax) * ex + (pz - az) * ez) / l2;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(px - (ax + ex * t), pz - (az + ez * t));
}
function crossErrIdx(px: number, pz: number, path: { x: number; z: number }[]): { e: number; seg: number } {
  let best = Infinity;
  let seg = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const d = distToSeg(px, pz, path[i]!.x, path[i]!.z, path[i + 1]!.x, path[i + 1]!.z);
    if (d < best) {
      best = d;
      seg = i;
    }
  }
  return { e: best, seg };
}
// フット付近 ±0.4m の窓での曲率(向き変化)
function localTurn(p: { x: number; z: number }[], seg: number): number {
  const cum: number[] = [0];
  for (let i = 1; i < p.length; i++) cum[i] = cum[i - 1]! + Math.hypot(p[i]!.x - p[i - 1]!.x, p[i]!.z - p[i - 1]!.z);
  let jb = seg;
  while (jb > 0 && cum[seg]! - cum[jb - 1]! < 0.4) jb--;
  let jf = seg;
  while (jf < p.length - 1 && cum[jf + 1]! - cum[seg]! < 0.4) jf++;
  if (jb >= seg || jf <= seg) return 0;
  const a1 = Math.atan2(p[seg]!.x - p[jb]!.x, p[seg]!.z - p[jb]!.z);
  const a2 = Math.atan2(p[jf]!.x - p[seg]!.x, p[jf]!.z - p[seg]!.z);
  let d = Math.abs(a2 - a1);
  if (d > Math.PI) d = 2 * Math.PI - d;
  return d;
}

const a = process.argv.slice(2).map(Number);
const params = clonedParams();
if (a.length >= 4) params.follow = { crossP: a[0]!, crossD: a[1]!, safeAccF: a[2]!, lookaheadK: a[3]! };
console.log('follow =', JSON.stringify(params.follow));

let curveMax = 0;
let goalMax = 0;
let straightMax = 0;
let curveSum = 0;
let curveN = 0;
let overshootMax = 0; // ゴールを行き過ぎた最大量
for (const seed of [1, 2, 3, 4]) {
  const m = new MatchSim({ seed, variant: 'optimal', redVariant: 'bucketFocus', headless: true, params });
  const dt = 1 / 60;
  let prevDTarget = Infinity;
  let minDTargetSeen = Infinity;
  for (let i = 0; i < Math.ceil(185 / dt); i++) {
    m.step(dt);
    const r = m.blue;
    if (r.path && r.path.length > 1 && r.cur && r.cur.t === 'goto' && !r.retry) {
      const { e, seg } = crossErrIdx(r.x, r.z, r.path);
      const cur = r.cur as { x: number; z: number };
      const dTarget = Math.hypot(cur.x - r.x, cur.z - r.z);
      const turn = localTurn(r.path, seg);
      const nearGoal = dTarget < 0.6;
      if (nearGoal) {
        if (e > goalMax) goalMax = e;
        // 行き過ぎ検出: 一度近づいた後に離れたら overshoot
        minDTargetSeen = Math.min(minDTargetSeen, dTarget);
        if (dTarget > prevDTarget && dTarget - minDTargetSeen > overshootMax) overshootMax = dTarget - minDTargetSeen;
      } else {
        minDTargetSeen = Infinity;
      }
      if (turn > 0.15) {
        if (e > curveMax) curveMax = e;
        curveSum += e;
        curveN++;
      } else if (!nearGoal) {
        if (e > straightMax) straightMax = e;
      }
      prevDTarget = dTarget;
    } else {
      prevDTarget = Infinity;
      minDTargetSeen = Infinity;
    }
    if (m.over && m.projectiles.length === 0) break;
  }
}
console.log(`カーブ区間: max横ズレ ${curveMax.toFixed(3)}m, 平均 ${(curveSum / Math.max(1, curveN)).toFixed(3)}m (${curveN}サンプル)`);
console.log(`ゴール接近(<0.6m): max横ズレ ${goalMax.toFixed(3)}m, 行き過ぎ(overshoot)最大 ${overshootMax.toFixed(3)}m`);
console.log(`直線区間: max横ズレ ${straightMax.toFixed(3)}m`);
