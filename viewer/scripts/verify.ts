/**
 * strategy.md の主張をシミュレーションで検証するモンテカルロ実験。
 *   npm run verify
 * 結果は stdout (markdown) と verify-report.json に出力。
 */
import { writeFileSync } from 'node:fs';
import { MatchSim } from '../src/sim/match';
import { clonedParams, type OppArchetype, type StrategyVariant } from '../src/config/params';
import { solveShot, heightProfile } from '../src/sim/ballistics';
import { DEFAULT_PARAMS } from '../src/config/params';

interface RunResult {
  blue: number;
  red: number;
  win: boolean;
  flagHits: number;
  movingHits: number;
  bonus: boolean;
  unusedAmmo: number;
}

function runOne(
  seed: number,
  archetype: OppArchetype,
  variant: StrategyVariant,
  mutate?: (p: ReturnType<typeof clonedParams>) => void,
): RunResult {
  const params = clonedParams();
  mutate?.(params);
  const m = new MatchSim({ seed, archetype, variant, headless: true, params });
  m.runToEnd(1 / 30);
  return {
    blue: m.score.blue,
    red: m.score.red,
    win: m.score.blue > m.score.red,
    flagHits: m.breakdown.blue.flag,
    movingHits: m.breakdown.blue.moving,
    bonus: m.breakdown.blue.bonus,
    unusedAmmo: m.blue.ammo + m.blue.superAmmo,
  };
}

interface Agg {
  mean: number;
  sd: number;
  winRate: number;
  meanFlag: number;
  meanMoving: number;
  bonusRate: number;
  meanUnused: number;
  meanRed: number;
}

function agg(rs: RunResult[]): Agg {
  const n = rs.length;
  const mean = rs.reduce((s, r) => s + r.blue, 0) / n;
  const sd = Math.sqrt(rs.reduce((s, r) => s + (r.blue - mean) ** 2, 0) / n);
  return {
    mean: Math.round(mean),
    sd: Math.round(sd),
    winRate: rs.filter((r) => r.win).length / n,
    meanFlag: +(rs.reduce((s, r) => s + r.flagHits, 0) / n).toFixed(1),
    meanMoving: +(rs.reduce((s, r) => s + r.movingHits, 0) / n).toFixed(1),
    bonusRate: rs.filter((r) => r.bonus).length / n,
    meanUnused: +(rs.reduce((s, r) => s + r.unusedAmmo, 0) / n).toFixed(1),
    meanRed: Math.round(rs.reduce((s, r) => s + r.red, 0) / n),
  };
}

function batch(
  n: number,
  archetype: OppArchetype,
  variant: StrategyVariant,
  mutate?: (p: ReturnType<typeof clonedParams>) => void,
): Agg {
  const rs: RunResult[] = [];
  for (let i = 0; i < n; i++) rs.push(runOne(1000 + i * 7919, archetype, variant, mutate));
  return agg(rs);
}

const N = 300;
const report: Record<string, unknown> = {};

console.log('# シミュレーション検証レポート\n');

// ---------------------------------------------------------------- E0 弾道
console.log('## E0: 弾道テーブル (旗: Δh=+1.5m / 抗力モデル)');
console.log('| 距離 | 通常弾道 速度/角度/滞空 | ロブ弾道 速度/角度 | 相手(高さ1.8m,旗手前1.2m)越え |');
console.log('|---|---|---|---|');
const e0: Array<Record<string, number | string>> = [];
for (let d = 1.5; d <= 6.01; d += 0.75) {
  const flat = solveShot(d, 1.5, DEFAULT_PARAMS.rag, false);
  const lob = solveShot(d, 1.5, DEFAULT_PARAMS.rag, true);
  let clear = '-';
  if (lob) {
    const prof = heightProfile(lob.speed, lob.angleRad, DEFAULT_PARAMS.rag, d, 0.1);
    const idx = Math.round((d - 1.2) / 0.1);
    const hAtObs = prof[Math.max(0, Math.min(prof.length - 1, idx))];
    // 砲口1.5m + プロファイル高さ vs 相手上限1.8m
    clear = hAtObs !== undefined && 1.5 + hAtObs > 1.85 ? '可' : '不可';
  }
  console.log(
    `| ${d.toFixed(2)}m | ${flat ? `${flat.speed.toFixed(1)}m/s / ${((flat.angleRad * 180) / Math.PI).toFixed(0)}° / ${flat.tof.toFixed(2)}s` : '解なし'} | ${lob ? `${lob.speed.toFixed(1)}m/s / ${((lob.angleRad * 180) / Math.PI).toFixed(0)}°` : '解なし'} | ${clear} |`,
  );
  e0.push({ d, flatSpeed: flat?.speed ?? -1, lobSpeed: lob?.speed ?? -1, clear });
}
report.e0_ballistics = e0;

// スーパー雑巾
const sup = solveShot(2.6, 1.5, DEFAULT_PARAMS.superRag, false);
console.log(`\nスーパー雑巾 2.6m→旗: ${sup ? `${sup.speed.toFixed(1)}m/s / ${((sup.angleRad * 180) / Math.PI).toFixed(0)}°` : '解なし'}`);
report.e0_super = sup;

// ---------------------------------------------------------------- E1 サイクルタイム感度
console.log('\n## E1: 射出サイクルの感度 (vs 手動装填チーム, n=' + N + ')');
console.log('| サイクル倍率 | 実効サイクル/枚 | 平均得点 | 未使用弾 |');
console.log('|---|---|---|---|');
const e1: Array<Record<string, number>> = [];
for (const cs of [0.6, 0.8, 1.0, 1.4, 1.8, 2.4]) {
  const a = batch(N, 'manual', 'standard', (p) => {
    p.shooter.aim *= cs;
    p.shooter.feed *= cs;
    p.shooter.recover *= cs;
  });
  const cyc = ((DEFAULT_PARAMS.shooter.aim + DEFAULT_PARAMS.shooter.feed + DEFAULT_PARAMS.shooter.recover) * cs).toFixed(1);
  console.log(`| ×${cs} | ${cyc}s | ${a.mean} | ${a.meanUnused} |`);
  e1.push({ cs, mean: a.mean, unused: a.meanUnused });
}
report.e1_cycle = e1;

// ---------------------------------------------------------------- E2 命中率感度
console.log('\n## E2: 旗命中率の感度 (vs 手動装填チーム)');
console.log('| 旗命中率 | 平均得点 | 勝率 |');
console.log('|---|---|---|');
const e2: Array<Record<string, number>> = [];
for (const fp of [0.4, 0.5, 0.6, 0.72, 0.8, 0.9]) {
  const a = batch(N, 'manual', 'standard', (p) => {
    p.probs.flag = fp;
  });
  console.log(`| ${(fp * 100).toFixed(0)}% | ${a.mean} | ${(a.winRate * 100).toFixed(0)}% |`);
  e2.push({ fp, mean: a.mean, win: a.winRate });
}
report.e2_flagP = e2;

// ---------------------------------------------------------------- E3 戦略×相手
console.log('\n## E3: 戦略バリアント × 相手アーキタイプ (平均得点 / 勝率)');
const variants: StrategyVariant[] = ['standard', 'flagOnly', 'bucketFocus'];
const archetypes: OppArchetype[] = ['manual', 'auto', 'blocker'];
console.log('| 戦略 \\ 相手 | ' + archetypes.join(' | ') + ' |');
console.log('|---|---|---|---|');
const e3: Record<string, Record<string, { mean: number; win: number; red: number }>> = {};
for (const v of variants) {
  const row: string[] = [];
  e3[v] = {};
  for (const ar of archetypes) {
    const a = batch(N, ar, v);
    row.push(`${a.mean} / ${(a.winRate * 100).toFixed(0)}%`);
    e3[v]![ar] = { mean: a.mean, win: a.winRate, red: a.meanRed };
  }
  console.log(`| ${v} | ${row.join(' | ')} |`);
}
report.e3_matrix = e3;

// ---------------------------------------------------------------- E5 標準シナリオ分布
console.log('\n## E5: 標準戦略の得点分布 (vs manual)');
const std = batch(600, 'manual', 'standard');
console.log(
  `平均 ${std.mean} ± ${std.sd} / 勝率 ${(std.winRate * 100).toFixed(1)}% / 旗平均 ${std.meanFlag}枚 / 移動バケツ平均 ${std.meanMoving}枚 / ボーナス率 ${(std.bonusRate * 100).toFixed(0)}%`,
);
report.e5_standard = std;

writeFileSync(new URL('../verify-report.json', import.meta.url), JSON.stringify(report, null, 2));
console.log('\n→ verify-report.json に保存');
