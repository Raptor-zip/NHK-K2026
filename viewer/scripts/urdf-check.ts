/**
 * URDF の全関節が「試合を通して実際に動き、可動域を超えない」ことを確かめる。
 *
 *     npm run urdf:check
 *
 * ⚠ **ブラウザを起こさない。** 関節値の作り方（`src/render/urdf-joints.ts`）は
 *   Three.js に依存しないので、試合シムを Node で 3 分回して数字だけ見れば
 *   足りる。描画の確認と混ぜると、遅くて誰も回さなくなる。
 *
 * 落ちる条件:
 *   * URDF に無い関節を動かしている / URDF にあるのに動かしていない
 *   * 試合を通して 1 度も動かない関節がある（＝そのモーターは制御していない）
 *   * revolute / prismatic が URDF の lower..upper を出る
 */
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { MatchSim } from '../src/sim/match';
import { clonedParams, type OppArchetype, type StrategyVariant } from '../src/config/params';
import {
  GRAB_STROKE,
  JOINT_NAMES,
  mapJoints,
  newAccum,
  type JointName,
} from '../src/render/urdf-joints';
import { DIMS, homePos, resupPose, ROBOT } from '../src/config/field';
import { sectionRects } from '../src/config/robot-shape';
import { LIDAR_HIGH, LIDAR_LOW_FRONT, LIDAR_LOW_REAR } from '../src/sim/lidar';
import type { RobotState } from '../src/sim/types';
import { readLidarMounts, readRobotBoxes, renderModule } from './urdf-shape';

const HERE = dirname(fileURLToPath(import.meta.url));
const URDF = resolve(HERE, '..', '..', 'cad', 'urdf', 'tr.urdf');
const SHAPE = resolve(HERE, '..', 'src', 'config', 'robot-shape.ts');

interface Limit {
  type: string;
  lower: number;
  upper: number;
}

/** URDF から関節の型と可動域を読む（機械生成の平たい XML なので正規表現で足りる） */
function readLimits(xml: string): Map<string, Limit> {
  const out = new Map<string, Limit>();
  const re = /<joint\s+name="([^"]+)"\s+type="([^"]+)"([\s\S]*?)<\/joint>/g;
  for (let m = re.exec(xml); m; m = re.exec(xml)) {
    const [, name, type, body] = m;
    const lim = /<limit([^>]*)\/>/.exec(body ?? '');
    const num = (k: string, d: number): number => {
      const v = new RegExp(`${k}="([^"]+)"`).exec(lim?.[1] ?? '');
      return v ? Number(v[1]) : d;
    };
    // fixed はセンサーの取付フレーム（走査原点）で、モーターではない。
    // 「URDF にあるのに制御していない」の対象から外す。
    if (type === 'fixed') continue;
    const cont = type === 'continuous';
    out.set(name!, {
      type: type!,
      lower: cont ? -Infinity : num('lower', 0),
      upper: cont ? Infinity : num('upper', 0),
    });
  }
  return out;
}

interface Track {
  min: number;
  max: number;
  moved: boolean;
}

/**
 * 「シムの数字が CAD と合っているか」。関節の動きとは別に、寸法そのものを見る。
 *
 * ⚠ ここが合っていないと、関節は可動域に収まっていても**別の機械の試合**に
 *   なる。実際こうなっていた: 砲口 1.5m（CAD は 1.0）、車輪 φ127（φ100）、
 *   補充で机に正対（実際はフォークが後ろにあるので背を向ける）。
 */
function checkGeometry(limits: Map<string, Limit>): string[] {
  const bad: string[] = [];
  const p = clonedParams();
  const say = (ok: boolean, msg: string): void => {
    console.log(`  ${ok ? 'OK ' : 'NG '} ${msg}`);
    if (!ok) bad.push(msg);
  };
  console.log('\n寸法が CAD と合っているか');

  // 砲塔の可動域（URDF が唯一の情報源）
  const yl = limits.get('turret_yaw');
  say(
    !!yl && Math.abs(p.turret.yawLimit - yl.upper) < 1e-6,
    `砲塔ヨー ±${p.turret.yawLimit.toFixed(6)} vs URDF ±${yl?.upper.toFixed(6)}`,
  );
  const pl = limits.get('shooter_pitch');
  say(
    !!pl && Math.abs(p.turret.pitchMin - pl.lower) < 1e-6 && Math.abs(p.turret.pitchMax - pl.upper) < 1e-6,
    `仰角 ${p.turret.pitchMin.toFixed(6)}..${p.turret.pitchMax.toFixed(6)} vs URDF ` +
      `${pl?.lower.toFixed(6)}..${pl?.upper.toFixed(6)}`,
  );

  // 砲口高さ。⚠ CAD の tr_params.NIP_Z = 1000mm
  say(Math.abs(p.turret.muzzleY - 1.0) < 1e-9, `砲口高さ ${p.turret.muzzleY}m vs CAD NIP_Z 1.0m`);

  // 車輪（cad/src/tr_params.py WHEEL_DIA=100 / cad/control LX=LY=0.30）
  say(Math.abs(p.wheel.r - 0.05) < 1e-9, `車輪半径 ${p.wheel.r}m vs CAD φ100 → 0.05m`);
  say(Math.abs(p.wheel.k - 0.3) < 1e-9, `(a+b)/2 ${p.wheel.k}m vs CAD LX=LY=0.30 → 0.3m`);

  // グラバーの可動域と速度（URDF の limit と一致していること）
  const gs = limits.get('grabber_slide');
  say(!!gs && Math.abs(GRAB_STROKE - gs.upper) < 1e-6, `スライド ${GRAB_STROKE}m vs URDF ${gs?.upper}m`);

  // 補充ポーズ: フォークが机に届き、車体は机に当たらないこと
  for (const team of ['blue', 'red'] as const) {
    const s = team === 'blue' ? 1 : -1;
    const desk = homePos(team, 'resup');
    const pose = resupPose(team);
    const nearEdge = desk.z + (DIMS.resupDesk.wz / 2) * s;
    const farEdge = desk.z - (DIMS.resupDesk.wz / 2) * s;
    const rear = pose.z - ROBOT.rearOverhang * s;
    const tipOpen = pose.z - ROBOT.forkReachMax * s;
    const tipShut = pose.z - ROBOT.forkReachMin * s;
    // 車体は縁より手前（机側へ入っていない）
    say((rear - nearEdge) * s > 0, `${team}: 車体後端 ${rear.toFixed(3)} が机の縁 ${nearEdge.toFixed(3)} より手前`);
    // 全閉の歯先も縁より手前（寄せるときに天板を叩かない）
    say((tipShut - nearEdge) * s > 0, `${team}: 全閉の歯先 ${tipShut.toFixed(3)} が縁より手前`);
    // 全開で机の中央より奥まで届く（山の向こう側に歯が入る）
    say((desk.z - tipOpen) * s > 0, `${team}: 全開の歯先 ${tipOpen.toFixed(3)} が机中央 ${desk.z.toFixed(3)} を越える`);
    // 天板を突き抜けない
    say((tipOpen - farEdge) * s > 0, `${team}: 全開の歯先が机の向こう端 ${farEdge.toFixed(3)} を突き抜けない`);
    // 向き: フォーク（車体 -X）が机を向く
    const fork = { x: -Math.sin(pose.theta), z: -Math.cos(pose.theta) };
    const toDesk = { x: desk.x - pose.x, z: desk.z - pose.z };
    const n = Math.hypot(toDesk.x, toDesk.z) || 1;
    const dot = (fork.x * toDesk.x + fork.z * toDesk.z) / n;
    say(dot > 0.99, `${team}: フォークが机を向いている (cos=${dot.toFixed(3)}, θ=${((pose.theta * 180) / Math.PI).toFixed(0)}°)`);
  }

  // --- 実体（当たり判定）と LiDAR 取付 ---
  // ⚠ `src/config/robot-shape.ts` は生成物。URDF から作り直して一致を見る。
  //   URDF は git に入らないので、ここで見ないと表だけが古くなる。
  const boxes = readRobotBoxes(URDF);
  const mounts = readLidarMounts(URDF);
  say(
    renderModule(boxes, mounts) === readFileSync(SHAPE, 'utf8'),
    `robot-shape.ts が URDF と一致（当たり判定 ${boxes.length} / LiDAR ${mounts.length}）` +
      '  ずれていたら npm run urdf:shape',
  );

  // 移動バケツ: URDF の当たり判定そのものと合っていること
  const bk = boxes.find((b) => Math.abs(b.y - (ROBOT.bucket.topY - 0.1275)) < 0.02 && b.sy > 0.25);
  say(!!bk, '移動バケツの当たり判定が URDF にある');
  if (bk) {
    say(Math.abs(bk.x - ROBOT.bucket.x) < 1e-6, `バケツ X ${ROBOT.bucket.x} vs URDF ${bk.x}`);
    say(Math.abs(bk.z - ROBOT.bucket.z) < 1e-6, `バケツ Z ${ROBOT.bucket.z} vs URDF ${bk.z}`);
    const top = bk.y + bk.sy / 2;
    say(Math.abs(ROBOT.bucket.topY - top) < 1e-6, `バケツ上面 ${ROBOT.bucket.topY}m vs URDF ${top}m`);
    say(top >= 1.2 && top <= 2.1, `バケツ上面が規定 3.2.3b の 1.2..2.1m 内 (${top}m)`);
  }
  say(
    Math.abs(p.bucketTopY.blue - ROBOT.bucket.topY) < 1e-6 &&
      Math.abs(p.bucketTopY.red - ROBOT.bucket.topY) < 1e-6,
    `既定のバケツ高 青 ${p.bucketTopY.blue} / 赤 ${p.bucketTopY.red} が CAD の ${ROBOT.bucket.topY}m`,
  );

  // LiDAR: 走査原点が車体中心でないこと（中心から飛ばすと別の機械の点群になる）
  for (const m of [LIDAR_LOW_FRONT, LIDAR_LOW_REAR, LIDAR_HIGH]) {
    const off = Math.hypot(m.x, m.z);
    say(off > 0.3, `${m.name}: 走査原点が車体中心から ${off.toFixed(3)}m 離れている`);
  }
  // ⚠ 下段は 2026-08-06 のレベリング座で 0.112 → 0.0696 に下がった（横材の上面
  //   135 から固定板 t6・すきま 12・可動板 t6・検出面まで 47.4 を引いた高さ）。
  say(
    Math.abs(LIDAR_LOW_FRONT.y - 0.0696) < 1e-9,
    `下段の走査面 ${LIDAR_LOW_FRONT.y}m vs CAD LIDAR_LOW_Z 0.0696m`,
  );
  say(
    Math.abs(LIDAR_HIGH.y - 0.5) < 1e-9,
    `上段の走査面 ${LIDAR_HIGH.y}m vs CAD LIDAR_HIGH_Z 0.5m`,
  );
  say(
    Math.abs(Math.abs(LIDAR_LOW_REAR.yaw) - Math.PI) < 1e-4,
    `後ろ向き下段の据付方位 ${((LIDAR_LOW_REAR.yaw * 180) / Math.PI).toFixed(0)}°`,
  );

  // 相手機の断面が「正方形」で無いこと（= URDF から切り出せている）
  for (const h of [LIDAR_LOW_FRONT.y, LIDAR_HIGH.y]) {
    const rects = sectionRects(h);
    let lox = Infinity;
    let hix = -Infinity;
    let loz = Infinity;
    let hiz = -Infinity;
    for (const r of rects) {
      lox = Math.min(lox, r.cx - r.hw);
      hix = Math.max(hix, r.cx + r.hw);
      loz = Math.min(loz, r.cz - r.hd);
      hiz = Math.max(hiz, r.cz + r.hd);
    }
    say(
      rects.length >= 2 && hix - lox > 0.3 && hiz - loz > 0.3,
      `走査面 ${h.toFixed(3)}m の相手機断面 ${rects.length} 枚 ` +
        `${(hix - lox).toFixed(3)} x ${(hiz - loz).toFixed(3)}m`,
    );
  }
  return bad;
}

function main(): number {
  const limits = readLimits(readFileSync(URDF, 'utf8'));
  console.log(`URDF: ${URDF}\n関節 ${limits.size} 個 / 制御している関節 ${JOINT_NAMES.length} 個\n`);

  const bad: string[] = [];
  for (const n of JOINT_NAMES) if (!limits.has(n)) bad.push(`${n}: URDF に無い関節を動かしている`);
  for (const n of limits.keys()) {
    if (!(JOINT_NAMES as readonly string[]).includes(n)) bad.push(`${n}: URDF にあるのに制御していない`);
  }

  // ⚠ **1 試合では足りない。** 相手の型と自陣の戦略で砲塔の振り方は変わる。
  //   全組み合わせ（3 × 5）を回して、いちばん振れたところで判定する。
  const ARCHETYPES: OppArchetype[] = ['manual', 'auto', 'blocker'];
  const VARIANTS: StrategyVariant[] = ['standard', 'flagOnly', 'bucketFocus', 'fixedOnly', 'optimal'];
  const DT = 1 / 60;
  const MATCH_LEN = 180;
  const track = new Map<JointName, Track>();
  for (const n of JOINT_NAMES) track.set(n, { min: Infinity, max: -Infinity, moved: false });
  const worst = new Map<JointName, string>();

  let games = 0;
  let steps = 0;
  let scoreSum = { blue: 0, red: 0 };
  for (const archetype of ARCHETYPES) {
    for (const variant of VARIANTS) {
      const sim = new MatchSim({
        seed: 20260803 + games,
        archetype,
        variant,
        headless: true,
        params: clonedParams(),
      });
      const accs = { blue: newAccum(), red: newAccum() };
      const prev = new Map<string, number>();
      // ⚠ `runToEnd()` は使わない。1 ステップごとに関節を覗きたいので自分で回す。
      //   終了条件は runToEnd と同じ（時間切れ＋飛翔中の弾が落ちるまで）。
      let n0 = 0;
      while (n0 < 20000 && sim.t <= MATCH_LEN + 10) {
        if (sim.over && sim.projectiles.length === 0) break;
        sim.step(DT);
        n0++;
        // 両陣とも見る。相手側にも同じ機体が立っている前提の競技なので、
        // 「こちらだけ可動域に収まっている」では検査にならない
        for (const team of ['blue', 'red'] as const) {
          const r = sim[team] as RobotState;
          const q = mapJoints(r, accs[team], DT);
          for (const n of JOINT_NAMES) {
            const t = track.get(n)!;
            const v = q[n];
            if (v < t.min || v > t.max) worst.set(n, `${archetype}/${variant}/${team}`);
            t.min = Math.min(t.min, v);
            t.max = Math.max(t.max, v);
            const key = `${team}:${n}`;
            const p = prev.get(key);
            if (p !== undefined && Math.abs(v - p) > 1e-9) t.moved = true;
            prev.set(key, v);
          }
        }
      }
      steps += n0;
      scoreSum.blue += sim.score.blue;
      scoreSum.red += sim.score.red;
      games++;
    }
  }

  console.log(
    `${'関節'.padEnd(16)}${'型'.padEnd(12)}${'最小'.padStart(10)}${'最大'.padStart(10)}` +
      `${'可動域'.padStart(20)}  判定`,
  );
  console.log('-'.repeat(86));
  for (const n of JOINT_NAMES) {
    const t = track.get(n)!;
    const l = limits.get(n);
    const notes: string[] = [];
    if (!t.moved) notes.push('一度も動いていない');
    if (l && t.min < l.lower - 1e-6) notes.push(`下限 ${l.lower.toFixed(3)} を ${(l.lower - t.min).toFixed(4)} 超過`);
    if (l && t.max > l.upper + 1e-6) notes.push(`上限 ${l.upper.toFixed(3)} を ${(t.max - l.upper).toFixed(4)} 超過`);
    if (notes.length) bad.push(`${n}: ${notes.join(' / ')}（最悪は ${worst.get(n) ?? '?'}）`);
    const range = l
      ? l.type === 'continuous'
        ? '連続回転'
        : `${l.lower.toFixed(3)}..${l.upper.toFixed(3)}`
      : '—';
    console.log(
      `${n.padEnd(16)}${(l?.type ?? '?').padEnd(12)}${t.min.toFixed(3).padStart(10)}` +
        `${t.max.toFixed(3).padStart(10)}${range.padStart(20)}  ${notes.length ? 'NG ' + notes.join(' / ') : 'OK'}`,
    );
  }

  console.log(
    `\n${games} 試合 / ${steps} ステップ・平均得点 青 ${(scoreSum.blue / games).toFixed(0)}` +
      ` - 赤 ${(scoreSum.red / games).toFixed(0)}`,
  );

  bad.push(...checkGeometry(limits));
  if (bad.length) {
    console.log(`\n⚠ ${bad.length} 件の指摘`);
    for (const b of bad) console.log(`  - ${b}`);
    return 1;
  }
  console.log('\n全関節が試合中に動き、可動域も超えていない');
  return 0;
}

process.exit(main());
