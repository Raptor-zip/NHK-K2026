import type { RagPhys } from '../config/params';

/**
 * 雑巾の弾道モデル: 質点 + 二次抗力 + バックスピン揚力
 *   a = g − (k/m)|v|v + lift·v_horizontal (上向き)
 * strategy.md §4.5.4 の「実測ルックアップテーブル」に相当するものを、
 * シミュレーター内では簡易空力モデルから生成する (実機では実射データで置換する)。
 */

const G = 9.81;
const DT = 1 / 240;

export interface ShotSolution {
  speed: number;
  angleRad: number;
  tof: number;
  apex: number;
}

interface FlightSample {
  h: number;
  t: number;
  apex: number;
}

/** 距離 d 進んだ時点の高さ (発射点基準)。到達不能なら null */
export function heightAtDistance(
  speed: number,
  angleRad: number,
  rag: RagPhys,
  d: number,
): FlightSample | null {
  let vx = Math.cos(angleRad) * speed;
  let vy = Math.sin(angleRad) * speed;
  let s = 0;
  let h = 0;
  let t = 0;
  let apex = 0;
  const km = rag.k / rag.m;
  for (let i = 0; i < 4000; i++) {
    if (s >= d) return { h, t, apex };
    const v = Math.hypot(vx, vy);
    vx -= km * v * vx * DT;
    vy -= (G + km * v * vy - rag.lift * vx) * DT;
    s += vx * DT;
    h += vy * DT;
    t += DT;
    if (h > apex) apex = h;
    if (vx < 0.03) return null; // 失速
    if (h < -2.5) return null;
  }
  return null;
}

/** 高さプロファイル (ブロック越え検証用): 距離 step ごとの高さ配列 */
export function heightProfile(
  speed: number,
  angleRad: number,
  rag: RagPhys,
  dMax: number,
  step: number,
): number[] {
  const out: number[] = [];
  let vx = Math.cos(angleRad) * speed;
  let vy = Math.sin(angleRad) * speed;
  let s = 0;
  let h = 0;
  let next = 0;
  const km = rag.k / rag.m;
  for (let i = 0; i < 6000 && s < dMax; i++) {
    while (s >= next && next <= dMax) {
      out.push(h);
      next += step;
    }
    const v = Math.hypot(vx, vy);
    vx -= km * v * vx * DT;
    vy -= (G + km * v * vy - rag.lift * vx) * DT;
    s += vx * DT;
    h += vy * DT;
    if (vx < 0.03) break;
  }
  return out;
}

function solveSpeedForAngle(
  d: number,
  dy: number,
  rag: RagPhys,
  angleRad: number,
): ShotSolution | null {
  // e(speed) = h(d) − dy は探索域内で概ね単調増加 → 区間探索 + 二分法
  let lo = -1;
  let hi = -1;
  let eLo = 0;
  for (let sp = 2.0; sp <= 20; sp += 0.5) {
    const f = heightAtDistance(sp, angleRad, rag, d);
    if (!f) continue;
    const e = f.h - dy;
    if (e < 0) {
      lo = sp;
      eLo = e;
    } else if (lo > 0) {
      hi = sp;
      break;
    } else {
      // 最初から上を通る → 低速側では届かないだけ。この角度では lo なし
      hi = sp;
      lo = sp - 0.5;
      break;
    }
  }
  if (hi < 0) return null;
  if (lo < 0) lo = 2.0;
  let best: ShotSolution | null = null;
  for (let i = 0; i < 12; i++) {
    const mid = (lo + hi) / 2;
    const f = heightAtDistance(mid, angleRad, rag, d);
    if (!f) {
      lo = mid;
      continue;
    }
    const e = f.h - dy;
    if (Math.abs(e) < 0.03) {
      best = { speed: mid, angleRad, tof: f.t, apex: f.apex };
      break;
    }
    if (e < 0) lo = mid;
    else hi = mid;
    best = { speed: mid, angleRad, tof: f.t, apex: f.apex };
  }
  if (best) {
    const f = heightAtDistance(best.speed, best.angleRad, rag, d);
    if (!f || Math.abs(f.h - dy) > 0.08) return null;
  }
  return best;
}

const cache = new Map<string, ShotSolution | null>();

/** 水平距離 d・高低差 dy に当てる最小速度解 (lob=true で高弾道側の角度を使う) */
export function solveShot(d: number, dy: number, rag: RagPhys, lob: boolean): ShotSolution | null {
  const key = `${d.toFixed(2)}|${dy.toFixed(2)}|${rag.m}|${rag.k}|${lob ? 1 : 0}`;
  const hit = cache.get(key);
  if (hit !== undefined) return hit;
  const angles = lob ? [58, 63, 68, 72] : [33, 38, 43, 48, 53, 58];
  let best: ShotSolution | null = null;
  for (const deg of angles) {
    const sol = solveSpeedForAngle(d, dy, rag, (deg * Math.PI) / 180);
    if (sol && (!best || sol.speed < best.speed)) best = sol;
  }
  cache.set(key, best);
  return best;
}
