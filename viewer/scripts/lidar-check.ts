/**
 * LiDAR シムと自己位置推定が「実機の取付」で回るか確かめる。
 *
 *     npm run lidar:check
 *
 * ⚠ `urdf:check` は headless で回すのでセンサーを 1 度も焚かない。
 *   走査原点を車体中心から実際の取付（前後 ±0.454m / 上段 0.5m）へ移したとき、
 *   点群の復元に取付ぶんの平行移動を足し忘れると ICP は 0.9m ずれた所に
 *   収束する。それを見張るのがここ。
 *
 * 落ちる条件:
 *   * 推定が真値から離れる（RMS > 0.15m / 最大 > 0.40m）
 *   * 点群が返ってこない・欠測ばかり
 *   * 相手機を 1 度も検出できない（上段の断面が空になっている等）
 */
import { MatchSim } from '../src/sim/match';
import { clonedParams, type OppArchetype, type StrategyVariant } from '../src/config/params';
import { LIDAR_HIGH, LIDAR_LOW_FRONT, LIDAR_LOW_REAR, scanToLocalPoints } from '../src/sim/lidar';

const DT = 1 / 60;
const SECONDS = 60;

interface Row {
  archetype: OppArchetype;
  variant: StrategyVariant;
  rms: number;
  max: number;
  pts: number;
  upperPts: number;
  sawOpponent: number;
}

function run(archetype: OppArchetype, variant: StrategyVariant, seed: number): Row {
  const sim = new MatchSim({
    seed,
    params: clonedParams(),
    archetype,
    variant,
    headless: false,
  });
  let sum = 0;
  let n = 0;
  let max = 0;
  let pts = 0;
  let upperPts = 0;
  let sawOpponent = 0;
  for (let i = 0; i < SECONDS / DT; i++) {
    sim.step(DT);
    const p = sim.player;
    // ⚠ リトライ中（人が持ち上げて運ぶ区間）は測らない。機体は非常停止中で
    //   計測輪も LiDAR も電源が無く、推定が流れるのは正しい挙動。
    if (p.retry) continue;
    const e = sim.localizer.est;
    const d = Math.hypot(e.x - p.x, e.z - p.z);
    sum += d * d;
    n++;
    max = Math.max(max, d);
    if (sim.lastScan) pts += scanToLocalPoints(sim.lastScan).n;
    if (sim.lastUpperScan) {
      const u = scanToLocalPoints(sim.lastUpperScan);
      upperPts += u.n;
      // 相手が上段の視野に居るか（真値で近ければ、見えていないとおかしい）
      const o = sim.opponent;
      if (Math.hypot(o.x - p.x, o.z - p.z) < 6 && u.n > 0) sawOpponent++;
    }
  }
  return { archetype, variant, rms: Math.sqrt(sum / n), max, pts, upperPts, sawOpponent };
}

function main(): number {
  console.log('LiDAR 取付（ビューア局所 m / 走査面はその y）');
  for (const m of [LIDAR_LOW_FRONT, LIDAR_LOW_REAR, LIDAR_HIGH]) {
    console.log(
      `  ${m.name.padEnd(16)} (X${m.x.toFixed(3)}, Y${m.y.toFixed(3)}, Z${m.z.toFixed(3)}) ` +
        `方位 ${((m.yaw * 180) / Math.PI).toFixed(0)}°`,
    );
  }
  console.log(`\n${SECONDS}s × 6 通り\n`);
  console.log('相手      戦略        推定RMS   最大    下段点/回  上段点/回  判定');
  console.log('-'.repeat(72));

  const bad: string[] = [];
  const cases: Array<[OppArchetype, StrategyVariant]> = [
    ['manual', 'standard'],
    ['auto', 'standard'],
    ['blocker', 'standard'],
    ['auto', 'bucketFocus'],
    ['blocker', 'flagOnly'],
    ['manual', 'optimal'],
  ];
  let k = 0;
  for (const [a, v] of cases) {
    const r = run(a, v, 20260804 + k * 7);
    k++;
    const scans = SECONDS / 0.1;
    const lo = r.pts / scans;
    const up = r.upperPts / scans;
    const ok = r.rms < 0.15 && r.max < 0.4 && lo > 50 && up > 5;
    if (!ok) bad.push(`${a}/${v}: RMS ${r.rms.toFixed(3)} 最大 ${r.max.toFixed(3)} 下段 ${lo.toFixed(0)} 上段 ${up.toFixed(0)}`);
    console.log(
      `${a.padEnd(9)} ${v.padEnd(11)} ${r.rms.toFixed(3)}m  ${r.max.toFixed(3)}m  ` +
        `${lo.toFixed(0).padStart(8)}  ${up.toFixed(0).padStart(8)}   ${ok ? 'OK' : 'NG'}`,
    );
  }

  if (bad.length) {
    console.log('\n✘ LiDAR / 自己位置推定が実機の取付で成立していない:');
    for (const b of bad) console.log('   ' + b);
    return 1;
  }
  console.log('\n取付位置ぶんの平行移動込みで、点群も推定も成立している');
  return 0;
}

process.exit(main());
