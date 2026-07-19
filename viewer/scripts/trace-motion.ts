/**
 * Motion trace for diagnosing unwanted vibration while the robot should be still.
 *
 * Usage:
 *   npx tsx scripts/trace-motion.ts --csv=/tmp/k2026-motion-trace.csv
 */
import { writeFileSync } from 'node:fs';
import { MatchSim } from '../src/sim/match';
import { clonedParams, type SimParams } from '../src/config/params';

interface Sample {
  caseName: string;
  t: number;
  cur: string;
  status: string;
  pathIdx: number;
  pathLen: number;
  pathErr: number | null;
  actT: number;
  goal: string;
  aimFired: boolean;
  x: number;
  z: number;
  vx: number;
  vz: number;
  speed: number;
  omega: number;
  theta: number;
  thetaTarget: number;
  estX: number;
  estZ: number;
  estErrMm: number;
  corrMm: number;
  rmseMm: number;
  matched: number;
  collided: boolean;
  driver: string;
}

interface RunSummary {
  caseName: string;
  throwWindows: number;
  throwSamples: number;
  throwPathSamples: number;
  meanThrowSpeed: number;
  p95ThrowSpeed: number;
  maxThrowSpeed: number;
  maxWindowSpanMm: number;
  meanThrowIcpCorrMm: number;
  maxThrowIcpCorrMm: number;
}

function csvEscape(v: unknown): string {
  if (v === null || v === undefined) return '';
  const s = String(v);
  return /[",\n]/.test(s) ? `"${s.replaceAll('"', '""')}"` : s;
}

function percentile(xs: number[], p: number): number {
  if (xs.length === 0) return 0;
  const sorted = [...xs].sort((a, b) => a - b);
  const i = Math.min(sorted.length - 1, Math.max(0, Math.floor((sorted.length - 1) * p)));
  return sorted[i]!;
}

function mean(xs: number[]): number {
  return xs.length === 0 ? 0 : xs.reduce((a, b) => a + b, 0) / xs.length;
}

function runCase(caseName: string, mutate?: (p: SimParams) => void): Sample[] {
  const params = clonedParams();
  mutate?.(params);
  const match = new MatchSim({
    seed: 20260704,
    archetype: 'manual',
    variant: 'standard',
    params,
    headless: false,
  });
  const out: Sample[] = [];
  const dt = 1 / 60;
  const sampleEvery = 2;
  for (let i = 0; i < 90 / dt; i++) {
    match.step(dt);
    if (i % sampleEvery !== 0) continue;
    const b = match.blue;
    const cur = b.cur?.t ?? 'none';
    const goal = b.aim?.goalKey ?? '';
    const pathLen = b.path?.length ?? 0;
    const wp = b.path ? b.path[Math.min(b.pathIdx, b.path.length - 1)] : null;
    const pathErr = wp ? Math.hypot(wp.x - b.x, wp.z - b.z) : null;
    const est = match.localizer.est;
    const diag = match.localizer.diag;
    const fixedThrow = cur === 'throw' && goal !== '' && goal !== 'moving';
    let driver = 'none';
    if (fixedThrow && params.defense.microMove) driver = 'microMove';
    else if (cur === 'goto') driver = 'pathFollow';
    else if (cur === 'throw' && goal === 'moving') driver = 'movingBucketPlanner';
    out.push({
      caseName,
      t: match.t,
      cur,
      status: b.status,
      pathIdx: b.pathIdx,
      pathLen,
      pathErr,
      actT: b.actT,
      goal,
      aimFired: b.aim?.fired ?? false,
      x: b.x,
      z: b.z,
      vx: b.vx,
      vz: b.vz,
      speed: Math.hypot(b.vx, b.vz),
      omega: b.omega,
      theta: b.theta,
      thetaTarget: b.thetaTarget,
      estX: est.x,
      estZ: est.z,
      estErrMm: Math.hypot(est.x - b.x, est.z - b.z) * 1000,
      corrMm: diag.corrMm,
      rmseMm: diag.rmse * 1000,
      matched: diag.matched,
      collided: b.collided,
      driver,
    });
  }
  return out;
}

function fixedThrowWindows(samples: Sample[]): Sample[][] {
  const windows: Sample[][] = [];
  let cur: Sample[] = [];
  for (const s of samples) {
    const fixedThrow = s.cur === 'throw' && s.goal !== '' && s.goal !== 'moving';
    const contiguous = cur.length === 0 || s.t - cur[cur.length - 1]!.t < 0.08;
    if (fixedThrow && contiguous) {
      cur.push(s);
    } else {
      if (cur.length > 0) windows.push(cur);
      cur = fixedThrow ? [s] : [];
    }
  }
  if (cur.length > 0) windows.push(cur);
  return windows;
}

function spanMm(samples: Sample[]): number {
  if (samples.length === 0) return 0;
  const xs = samples.map((s) => s.x);
  const zs = samples.map((s) => s.z);
  return Math.hypot(Math.max(...xs) - Math.min(...xs), Math.max(...zs) - Math.min(...zs)) * 1000;
}

function summarize(caseName: string, samples: Sample[]): RunSummary {
  const fixedThrow = samples.filter((s) => s.cur === 'throw' && s.goal !== 'moving');
  const windows = fixedThrowWindows(samples);
  const speeds = fixedThrow.map((s) => s.speed);
  const corr = fixedThrow.map((s) => s.corrMm);
  return {
    caseName,
    throwWindows: windows.length,
    throwSamples: fixedThrow.length,
    throwPathSamples: fixedThrow.filter((s) => s.pathLen > 0).length,
    meanThrowSpeed: mean(speeds),
    p95ThrowSpeed: percentile(speeds, 0.95),
    maxThrowSpeed: Math.max(0, ...speeds),
    maxWindowSpanMm: Math.max(0, ...windows.map(spanMm)),
    meanThrowIcpCorrMm: mean(corr),
    maxThrowIcpCorrMm: Math.max(0, ...corr),
  };
}

function toCsv(samples: Sample[]): string {
  const keys = Object.keys(samples[0] ?? {}) as Array<keyof Sample>;
  return [
    keys.join(','),
    ...samples.map((s) => keys.map((k) => csvEscape(s[k])).join(',')),
  ].join('\n');
}

const csvArg = process.argv.find((a) => a.startsWith('--csv='));
const csvPath = csvArg?.slice('--csv='.length);

const runs = [
  runCase('forced-microMove-on', (p) => {
    p.defense.microMove = true;
  }),
  runCase('default-microMove-off', (p) => {
    p.defense.microMove = false;
  }),
];
const all = runs.flat();
const summaries = runs.map((samples) => summarize(samples[0]?.caseName ?? 'unknown', samples));

console.log('Motion trace summary');
console.table(
  summaries.map((s) => ({
    case: s.caseName,
    throwWindows: s.throwWindows,
    throwSamples: s.throwSamples,
    pathSamplesDuringThrow: s.throwPathSamples,
    meanSpeed_mm_s: +(s.meanThrowSpeed * 1000).toFixed(1),
    p95Speed_mm_s: +(s.p95ThrowSpeed * 1000).toFixed(1),
    maxSpeed_mm_s: +(s.maxThrowSpeed * 1000).toFixed(1),
    maxThrowWindowSpan_mm: +s.maxWindowSpanMm.toFixed(1),
    meanIcpCorr_mm: +s.meanThrowIcpCorrMm.toFixed(2),
    maxIcpCorr_mm: +s.maxThrowIcpCorrMm.toFixed(2),
  })),
);

for (const samples of runs) {
  const firstThrow = samples.find((s) => s.cur === 'throw' && s.goal !== 'moving');
  if (!firstThrow) continue;
  const window = samples
    .filter((s) => s.t >= firstThrow.t && s.t < firstThrow.t + 1.2)
    .map((s) => ({
      t: s.t.toFixed(2),
      cur: s.cur,
      goal: s.goal,
      pathIdx: `${s.pathIdx}/${s.pathLen}`,
      x: s.x.toFixed(3),
      z: s.z.toFixed(3),
      speed_mm_s: (s.speed * 1000).toFixed(1),
      corrMm: s.corrMm.toFixed(2),
      driver: s.driver,
    }));
  console.log(`\nFirst fixed-throw window: ${firstThrow.caseName}`);
  console.table(window.slice(0, 12));
}

if (csvPath) {
  writeFileSync(csvPath, toCsv(all));
  console.log(`\nCSV written: ${csvPath}`);
}
