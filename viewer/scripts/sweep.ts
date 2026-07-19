/**
 * パラメータ探索 CLI: あるパラメータを範囲でふり、各値で多数の試合を回して、
 * どの値が一番スコアが高いかを表示する。worker_threads でコア数ぶん並列実行する。
 *
 * 使い方:
 *   npx tsx scripts/sweep.ts <param.path> <min> <max> <steps> [games] [variant] [oppVariant]
 * 例:
 *   npx tsx scripts/sweep.ts aimSpreadDeg 1 6 11 60
 *   npx tsx scripts/sweep.ts probs.flag 0.4 0.9 6 40 optimal bucketFocus
 *   npx tsx scripts/sweep.ts flagFallPerSec 0 0.08 9 60
 *
 * param.path は SimParams へのドットパス (例: aimSpreadDeg, probs.flag, drive.vmax,
 * swerveDrive.acc, flagFallPerSec, bucketTopY.blue)。青チームに適用して評価する。
 */
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { availableParallelism } from 'node:os';
import { fileURLToPath } from 'node:url';
import { MatchSim } from '../src/sim/match';
import { clonedParams, type StrategyVariant } from '../src/config/params';

interface Job {
  value: number;
  seed: number;
}
interface Cfg {
  paramPath: string;
  variant: StrategyVariant;
  opp: StrategyVariant;
}

function setByPath(obj: Record<string, unknown>, path: string, val: number): void {
  const keys = path.split('.');
  let o = obj;
  for (let i = 0; i < keys.length - 1; i++) o = o[keys[i]!] as Record<string, unknown>;
  o[keys[keys.length - 1]!] = val;
}

function runGame(cfg: Cfg, value: number, seed: number): { blue: number; win: boolean } {
  const params = clonedParams();
  setByPath(params as unknown as Record<string, unknown>, cfg.paramPath, value);
  const m = new MatchSim({
    seed,
    variant: cfg.variant,
    redVariant: cfg.opp,
    headless: true,
    params,
  });
  m.runToEnd(1 / 30);
  return { blue: m.score.blue, win: m.score.blue > m.score.red };
}

if (!isMainThread) {
  // ---- ワーカー: 割り当てられた試合を回して返す ----
  const { cfg, jobs } = workerData as { cfg: Cfg; jobs: Job[] };
  const out = jobs.map((j) => ({ value: j.value, ...runGame(cfg, j.value, j.seed) }));
  parentPort!.postMessage(out);
} else {
  // ---- メイン: 掃引を組み、ワーカーへ分配、集計 ----
  const a = process.argv.slice(2);
  if (a.length < 4) {
    console.error('使い方: npx tsx scripts/sweep.ts <param.path> <min> <max> <steps> [games] [variant] [oppVariant]');
    process.exit(1);
  }
  const paramPath = a[0]!;
  const min = Number(a[1]);
  const max = Number(a[2]);
  const steps = Math.max(1, Math.floor(Number(a[3])));
  const games = a[4] ? Math.max(1, Math.floor(Number(a[4]))) : 40;
  const variant = (a[5] as StrategyVariant) ?? 'optimal';
  const opp = (a[6] as StrategyVariant) ?? 'bucketFocus';
  const cfg: Cfg = { paramPath, variant, opp };

  const values = steps === 1 ? [min] : Array.from({ length: steps }, (_, i) => min + ((max - min) * i) / (steps - 1));
  const jobs: Job[] = [];
  for (const v of values) for (let s = 1; s <= games; s++) jobs.push({ value: v, seed: s * 7919 + 1 });

  const nW = Math.max(1, Math.min(availableParallelism() - 1, 16));
  const chunks: Job[][] = Array.from({ length: nW }, () => []);
  jobs.forEach((j, i) => chunks[i % nW]!.push(j));

  console.log(
    `掃引: ${paramPath} を ${min}〜${max} で ${steps}点 × ${games}試合 (青=${variant} vs 赤=${opp})、` +
      `${nW}並列で計 ${jobs.length} 試合...`,
  );
  const t0 = Date.now();
  const self = fileURLToPath(import.meta.url);

  const results = await Promise.all(
    chunks.map(
      (jobsChunk) =>
        new Promise<Array<{ value: number; blue: number; win: boolean }>>((resolve, reject) => {
          if (jobsChunk.length === 0) return resolve([]);
          const w = new Worker(self, { workerData: { cfg, jobs: jobsChunk }, execArgv: process.execArgv });
          w.once('message', (m) => {
            resolve(m);
            void w.terminate();
          });
          w.once('error', reject);
        }),
    ),
  );

  // 集計
  const byVal = new Map<number, { sum: number; wins: number; n: number }>();
  for (const arr of results)
    for (const r of arr) {
      const e = byVal.get(r.value) ?? { sum: 0, wins: 0, n: 0 };
      e.sum += r.blue;
      e.wins += r.win ? 1 : 0;
      e.n++;
      byVal.set(r.value, e);
    }

  const rows = values.map((v) => {
    const e = byVal.get(v)!;
    return { v, avg: e.sum / e.n, win: (e.wins / e.n) * 100 };
  });
  rows.sort((x, y) => y.avg - x.avg);

  console.log(`\n${paramPath.padEnd(18)} | 平均青得点 | 勝率`);
  console.log('-'.repeat(44));
  for (const r of [...rows].sort((x, y) => x.v - y.v)) {
    console.log(`${r.v.toFixed(4).padStart(18)} | ${r.avg.toFixed(1).padStart(9)} | ${r.win.toFixed(0)}%`);
  }
  const best = rows[0]!;
  console.log('-'.repeat(44));
  console.log(`最良: ${paramPath} = ${best.v.toFixed(4)} → 平均 ${best.avg.toFixed(1)}点 (勝率${best.win.toFixed(0)}%)`);
  console.log(`(${jobs.length}試合を ${((Date.now() - t0) / 1000).toFixed(1)}秒)`);
}
