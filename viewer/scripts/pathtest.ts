/**
 * 経路追従の誤差をヘッドレスで計測し、追従制御パラメータ (par.follow) を掃引して最良を探す。
 * 各ステップで「ロボット位置 → 現在の経路 への最短距離(横ズレ)」を測り、maxオーバー量・RMS・
 * 平均速度・スコアを集計する。結果は pathtest-log.json に追記し、標準出力にも表を出す。
 *
 *   npx tsx scripts/pathtest.ts
 *
 * 目的: オーバー(カーブでの膨らみ = 大きな横ズレ)を最小化しつつスループット(スコア)を保つ
 * パラメータを、当てずっぽうでなく計測で決める。
 */
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { availableParallelism } from 'node:os';
import { appendFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { MatchSim } from '../src/sim/match';
import { clonedParams } from '../src/config/params';

interface FollowCfg {
  crossP: number;
  crossD: number;
  safeAccF: number;
  lookaheadK: number;
}
interface Result {
  cfg: FollowCfg;
  maxCross: number; // 最大横ズレ (最悪オーバー量, m)
  curveMax: number; // カーブ区間の最大横ズレ (膨らみ)
  goalOver: number; // ゴール接近での行き過ぎ最大
  rms: number; // 横ズレ RMS (m)
  p95: number; // 95パーセンタイル横ズレ
  avgSpeed: number;
  score: number; // 青の平均得点 (スループット代理)
}

function localTurn(p: { x: number; z: number }[], seg: number, cum: number[]): number {
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

/** 点 (px,pz) から線分 a-b への最短距離 */
function distToSeg(px: number, pz: number, ax: number, az: number, bx: number, bz: number): number {
  const ex = bx - ax;
  const ez = bz - az;
  const l2 = ex * ex + ez * ez || 1e-9;
  let t = ((px - ax) * ex + (pz - az) * ez) / l2;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const qx = ax + ex * t;
  const qz = az + ez * t;
  return Math.hypot(px - qx, pz - qz);
}
function crossErr(px: number, pz: number, path: { x: number; z: number }[]): number {
  let best = Infinity;
  for (let i = 0; i < path.length - 1; i++) {
    const d = distToSeg(px, pz, path[i]!.x, path[i]!.z, path[i + 1]!.x, path[i + 1]!.z);
    if (d < best) best = d;
  }
  return best;
}

function runInstrumented(cfg: FollowCfg, seed: number): Omit<Result, 'cfg'> {
  const params = clonedParams();
  Object.assign(params.follow, cfg);
  const m = new MatchSim({ seed, variant: 'optimal', redVariant: 'bucketFocus', headless: true, params });
  const dt = 1 / 60;
  const errs: number[] = [];
  let maxCross = 0;
  let curveMax = 0;
  let goalOver = 0;
  let sumSpeed = 0;
  let moveCnt = 0;
  let prevDT = Infinity;
  let minDT = Infinity;
  const maxSteps = Math.ceil(185 / dt);
  for (let i = 0; i < maxSteps; i++) {
    m.step(dt);
    const r = m.blue;
    if (r.path && r.path.length > 1 && r.cur && r.cur.t === 'goto' && !r.retry) {
      const p = r.path as { x: number; z: number }[];
      let best = Infinity;
      let seg = 0;
      for (let k = 0; k < p.length - 1; k++) {
        const d = distToSeg(r.x, r.z, p[k]!.x, p[k]!.z, p[k + 1]!.x, p[k + 1]!.z);
        if (d < best) {
          best = d;
          seg = k;
        }
      }
      errs.push(best);
      if (best > maxCross) maxCross = best;
      const cum: number[] = [0];
      for (let k = 1; k < p.length; k++) cum[k] = cum[k - 1]! + Math.hypot(p[k]!.x - p[k - 1]!.x, p[k]!.z - p[k - 1]!.z);
      if (localTurn(p, seg, cum) > 0.15 && best > curveMax) curveMax = best;
      const cur = r.cur as { x: number; z: number };
      const dT = Math.hypot(cur.x - r.x, cur.z - r.z);
      if (dT < 0.6) {
        minDT = Math.min(minDT, dT);
        if (dT > prevDT && dT - minDT > goalOver) goalOver = dT - minDT;
      } else minDT = Infinity;
      prevDT = dT;
      sumSpeed += Math.hypot(r.vx, r.vz);
      moveCnt++;
    } else {
      prevDT = Infinity;
      minDT = Infinity;
    }
    if (m.over && m.projectiles.length === 0) break;
  }
  errs.sort((a, b) => a - b);
  const rms = Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / Math.max(1, errs.length));
  const p95 = errs.length ? errs[Math.floor(errs.length * 0.95)]! : 0;
  return { maxCross, curveMax, goalOver, rms, p95, avgSpeed: sumSpeed / Math.max(1, moveCnt), score: m.score.blue };
}

if (!isMainThread) {
  const { configs, seeds } = workerData as { configs: FollowCfg[]; seeds: number[] };
  const out: Result[] = configs.map((cfg) => {
    const runs = seeds.map((s) => runInstrumented(cfg, s));
    const n = runs.length;
    return {
      cfg,
      maxCross: Math.max(...runs.map((r) => r.maxCross)),
      curveMax: Math.max(...runs.map((r) => r.curveMax)),
      goalOver: Math.max(...runs.map((r) => r.goalOver)),
      rms: runs.reduce((s, r) => s + r.rms, 0) / n,
      p95: runs.reduce((s, r) => s + r.p95, 0) / n,
      avgSpeed: runs.reduce((s, r) => s + r.avgSpeed, 0) / n,
      score: runs.reduce((s, r) => s + r.score, 0) / n,
    };
  });
  parentPort!.postMessage(out);
} else {
  // グリッド定義 (掃引対象)
  const grid = (args?: string[]): FollowCfg[] => {
    void args;
    const crossP = [18, 26, 38];
    const crossD = [2, 3.5];
    const safeAccF = [0.7, 0.85, 1.0];
    const lookaheadK = [0.12, 0.18, 0.26];
    const out: FollowCfg[] = [];
    for (const p of crossP) for (const d of crossD) for (const s of safeAccF) for (const l of lookaheadK) out.push({ crossP: p, crossD: d, safeAccF: s, lookaheadK: l });
    return out;
  };
  const configs = grid();
  const seeds = [1, 2, 3, 4, 5, 6];
  const nW = Math.max(1, Math.min(availableParallelism() - 1, 16));
  const chunks: FollowCfg[][] = Array.from({ length: nW }, () => []);
  configs.forEach((c, i) => chunks[i % nW]!.push(c));

  console.log(`追従誤差計測: ${configs.length}構成 × ${seeds.length}試合 を ${nW}並列で...`);
  const t0 = Date.now();
  const self = fileURLToPath(import.meta.url);
  const results = (
    await Promise.all(
      chunks.map(
        (cfgs) =>
          new Promise<Result[]>((resolve, reject) => {
            if (cfgs.length === 0) return resolve([]);
            const w = new Worker(self, { workerData: { configs: cfgs, seeds }, execArgv: process.execArgv });
            w.once('message', (m) => {
              resolve(m);
              void w.terminate();
            });
            w.once('error', reject);
          }),
      ),
    )
  ).flat();

  // 品質: カーブ膨らみ・ゴール行き過ぎを主指標に、RMSとスループット低下も加味 (小さいほど良い)
  const quality = (r: Result): number =>
    r.curveMax + r.goalOver * 1.3 + r.rms * 2 + Math.max(0, (330 - r.score) / 330) * 0.3;
  results.sort((a, b) => quality(a) - quality(b));

  console.log(`\n${'crossP crossD safeAccF lookK'.padEnd(26)} | カーブ膨 | ゴール過 | RMS   | スコア`);
  console.log('-'.repeat(70));
  for (const r of results.slice(0, 14)) {
    const c = r.cfg;
    const label = `${c.crossP} ${c.crossD} ${c.safeAccF} ${c.lookaheadK}`.padEnd(26);
    console.log(
      `${label} | ${r.curveMax.toFixed(3).padStart(7)} | ${r.goalOver.toFixed(3).padStart(7)} | ${r.rms.toFixed(3)} | ${r.score.toFixed(0)}`,
    );
  }
  const best = results[0]!;
  console.log('-'.repeat(70));
  console.log(
    `最良: ${JSON.stringify(best.cfg)} → カーブ膨 ${best.curveMax.toFixed(3)}m, ゴール過 ${best.goalOver.toFixed(3)}m, RMS ${best.rms.toFixed(3)}m, スコア ${best.score.toFixed(0)}`,
  );
  console.log(`(${configs.length * seeds.length}試合を ${((Date.now() - t0) / 1000).toFixed(1)}秒)`);
  appendFileSync(
    'pathtest-log.json',
    JSON.stringify({ ts: t0, best: best.cfg, top: results.slice(0, 8) }) + '\n',
  );
}
