import { waypoints, type TeamId } from '../config/field';
import { DEFAULT_PARAMS, type OppArchetype, type SimParams, type StrategyVariant } from '../config/params';
import type { Action } from './types';

/** strategy.md §2.3 の試合プランを行動キューに落としたもの (公式フィールド図0415準拠) */

const st = (text: string): Action => ({ t: 'status', text });
const call = (text: string): Action => ({ t: 'call', text });
const strategyVariants: StrategyVariant[] = ['standard', 'flagOnly', 'bucketFocus', 'fixedOnly', 'optimal'];

function isStrategyVariant(v: OppArchetype | StrategyVariant): v is StrategyVariant {
  return strategyVariants.includes(v as StrategyVariant);
}

/** 固定バケツ(b1/b2/b3)と机(desk1/desk2)だけを繰り返し狙う戦略 */
function buildFixedOnlyScript(team: TeamId): Action[] {
  const wp = waypoints(team);
  const a: Action[] = [];
  const sweep = (): Action[] => [
    st('固定バケツ・机狙い'),
    { t: 'goto', ...wp.fireDesk1 },
    { t: 'throw', goal: 'desk1', count: 2 },
    { t: 'goto', ...wp.fireB2 },
    { t: 'throw', goal: 'b2', count: 2 },
    { t: 'goto', ...wp.fireR },
    { t: 'throw', goal: 'b1', count: 2 },
    { t: 'goto', ...wp.fireB3 },
    { t: 'throw', goal: 'b3', count: 2 },
    { t: 'goto', ...wp.fireDesk2 },
    { t: 'throw', goal: 'desk2', count: 2 },
  ];
  a.push(st('補充スポットへ'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 8 });
  a.push(...sweep());
  for (const time of [31, 61] as const) {
    a.push({ t: 'waitUntil', time, label: '補充待ち' });
    a.push(st('補充'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 8 });
    a.push(...sweep());
  }
  // スーパー雑巾も机 (40点) に使う
  a.push({ t: 'waitUntil', time: 121, label: 'スーパー雑巾待ち' });
  a.push(st('スーパー雑巾回収'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 2, dur: 6, superRag: true });
  a.push({ t: 'goto', ...wp.fireDesk1 }, { t: 'throw', goal: 'desk1', superRag: true, count: 1 });
  a.push({ t: 'goto', ...wp.fireDesk2 }, { t: 'throw', goal: 'desk2', superRag: true, count: 1 });
  a.push(...sweep());
  return a;
}

/**
 * 期待値最適(動的)。補充のたびに {t:'replan'} を置き、match 側がその時点のライブ状態
 * (残り時間・残弾・既得点・旗の掛かり本数・射点距離)で planOptimal を呼んで投擲キューを
 * 都度生成する。移動時間(残り時間で投げ切れる枚数)・距離依存命中率・旗の確率的落下を織り込む。
 */
function buildOptimalScript(team: TeamId): Action[] {
  const wp = waypoints(team);
  const a: Action[] = [];
  const supply = (): Action[] => [
    st('最適戦略: 補充'),
    { t: 'goto', ...wp.resupFront },
    { t: 'pickup', n: 10, dur: 8 },
    { t: 'replan' },
  ];
  a.push(...supply()); // 初回 (初期在庫)
  for (const time of [31, 61] as const) {
    a.push({ t: 'waitUntil', time, label: '補充待ち' });
    a.push(...supply());
  }
  // スーパー雑巾 (120秒ブザー)
  a.push({ t: 'waitUntil', time: 121, label: 'スーパー雑巾待ち' });
  a.push(
    st('スーパー雑巾回収'),
    { t: 'goto', ...wp.resupFront },
    { t: 'pickup', n: 2, dur: 6, superRag: true },
    { t: 'replan' },
  );
  return a;
}

export function buildStrategyScript(
  team: TeamId,
  variant: StrategyVariant,
  par: SimParams = DEFAULT_PARAMS,
): Action[] {
  if (variant === 'fixedOnly') return buildFixedOnlyScript(team);
  if (variant === 'optimal') return buildOptimalScript(team);
  const wp = waypoints(team);
  const a: Action[] = [];
  // Phase 1: 一括ピックアップ (補充机はx=+4.95縁。横付け回収)
  a.push(st('補充スポットへ'), { t: 'goto', ...wp.resupFront }, {
    t: 'pickup', n: 10, dur: 8,
  });

  if (variant === 'flagOnly' || variant === 'bucketFocus') {
    a.push(st('前進'), { t: 'goto', ...wp.fireR });
    const goal = variant === 'flagOnly' ? 'flag' : 'moving';
    a.push(st(variant === 'flagOnly' ? '旗連射' : '移動バケツ狙い'), { t: 'throw', goal, count: 10 });
  } else {
    // ボーナス走: x+側 (机①→バケツ②) → 中央 (バケツ①) → x−側 (バケツ③→机②) と掃く
    a.push(st('ボーナス走'), { t: 'goto', ...wp.fireDesk1 }, { t: 'throw', goal: 'desk1' });
    a.push({ t: 'goto', ...wp.fireB2 }, { t: 'throw', goal: 'b2' });
    a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'b1' });
    a.push({ t: 'goto', ...wp.fireB3 }, { t: 'throw', goal: 'b3' });
    a.push({ t: 'goto', ...wp.fireDesk2 }, { t: 'throw', goal: 'desk2' });
    a.push(st('旗連射'), { t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', count: 5 });
  }

  // Phase 2: 0:30 / 1:00 補充と旗刈り (smart = 相手静止時のみ移動バケツ)
  const midGoal = variant === 'bucketFocus' ? 'moving' : variant === 'flagOnly' ? 'flag' : 'smart';
  a.push({ t: 'waitUntil', time: 31, label: '補充待ち' });
  a.push(st('補充(0:30分)'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 8 });
  a.push({ t: 'goto', ...wp.fireR }, st('旗刈り'), { t: 'throw', goal: midGoal, count: 10 });

  a.push({ t: 'waitUntil', time: 61, label: '補充待ち' });
  a.push(st('補充(1:00分)'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 8 });
  a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: midGoal, count: 8 });

  // Phase 3: スーパー雑巾
  a.push({ t: 'waitUntil', time: 121, label: 'スーパー雑巾待ち' });
  a.push(st('スーパー雑巾回収'), { t: 'goto', ...wp.resupFront }, {
    t: 'pickup', n: 2, dur: 6, superRag: true,
  });
  a.push({ t: 'goto', ...wp.fireR }, call('スーパー→旗'), {
    t: 'throw', goal: variant === 'bucketFocus' ? 'moving' : 'flag', superRag: true, count: 2,
  });

  // 2:30以降: 残弾を撃ち切る (敵雑巾の回収機構による回収は不可 → デナイアル走は廃止)
  a.push({ t: 'throw', goal: midGoal, count: 2 });
  a.push(st('最終攻撃'), { t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', count: 3 });
  return a;
}

export function buildBlueScript(variant: StrategyVariant, par?: SimParams): Action[] {
  return buildStrategyScript('blue', variant, par);
}

export function buildRedScript(archetype: OppArchetype | StrategyVariant, par?: SimParams): Action[] {
  if (isStrategyVariant(archetype)) return buildStrategyScript('red', archetype, par);
  const wp = waypoints('red');
  const a: Action[] = [];
  if (archetype === 'auto') {
    a.push(st('補充スポットへ'), { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 9 });
    a.push(st('ボーナス走'), { t: 'goto', ...wp.fireDesk1 }, { t: 'throw', goal: 'desk1' });
    a.push({ t: 'goto', ...wp.fireB2 }, { t: 'throw', goal: 'b2' });
    a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'b1' });
    a.push({ t: 'goto', ...wp.fireB3 }, { t: 'throw', goal: 'b3' });
    a.push({ t: 'goto', ...wp.fireDesk2 }, { t: 'throw', goal: 'desk2' });
    a.push(st('旗連射'), { t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', count: 5 });
    a.push({ t: 'waitUntil', time: 32 }, { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 9 });
    a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', count: 10 });
    a.push({ t: 'waitUntil', time: 62 }, { t: 'goto', ...wp.resupFront }, { t: 'pickup', n: 10, dur: 9 });
    a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', count: 8 });
    a.push({ t: 'waitUntil', time: 122 }, { t: 'goto', ...wp.resupFront }, {
      t: 'pickup', n: 2, dur: 7, superRag: true,
    });
    a.push({ t: 'goto', ...wp.fireR }, { t: 'throw', goal: 'flag', superRag: true, count: 2 });
    a.push({ t: 'throw', goal: 'flag', count: 4 });
    return a;
  }

  // manual / blocker: 手動装填 (スタートゾーンで電源OFF静止 + 人が装填 = pickup)
  // blocker は自陣旗 (x=-0.55) と相手射点の間に立つ
  const firePos = archetype === 'blocker' ? { x: -0.55, z: -1.0 } : wp.fireR;
  const manualLoad = (dur: number, n: number): Action[] => [
    { t: 'power', on: false },
    { t: 'wait', dur: dur - 1, label: '手動装填中(電源OFF・審判許可)' },
    { t: 'pickup', n, dur: 1 },
    { t: 'power', on: true },
  ];
  a.push(st('手動装填(電源OFF)'), ...manualLoad(14, 10));
  a.push(st(archetype === 'blocker' ? '旗前ブロック位置へ' : '前進'), { t: 'goto', ...firePos });
  a.push({ t: 'throw', goal: 'flag', count: 2 });
  a.push({ t: 'throw', goal: 'b1' });
  a.push({ t: 'throw', goal: 'flag', count: 3 });
  a.push(st('装填に戻る'), { t: 'goto', ...wp.start }, ...manualLoad(25, 10));
  a.push({ t: 'goto', ...firePos }, { t: 'throw', goal: 'flag', count: 6 });
  a.push(st('装填に戻る'), { t: 'goto', ...wp.start }, ...manualLoad(22, 12));
  a.push({ t: 'goto', ...firePos }, { t: 'throw', goal: 'flag', count: 6 });
  return a;
}

export function initialPose(team: TeamId): { x: number; z: number; theta: number } {
  const wp = waypoints(team);
  // 車体は最初から攻撃方位 (相手陣向き) で待機。マスコットは椅子ごと補充スポットへ
  // わずかに振って搭載し、FAQ 4.1 Q10 の「マスコット正面=補充スポット正対」に合わせる。
  // 補充は x=+4.95 の机へ横付け (青機の車体-x側面グラバー) — 車体旋回ゼロは維持。
  return { x: wp.start.x, z: wp.start.z, theta: team === 'blue' ? Math.PI : 0 };
}
