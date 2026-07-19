import { POINTS, type ThrowTargetKey } from '../config/field';
import type { SimParams } from '../config/params';

/**
 * 期待値最大化の割当ソルバ (strategy.md の解析: 各ゴールの「限界期待値」に対する
 * water-filling)。旗の飽和・容量、固定ゴールの全制覇ボーナス(結合)を織り込む。
 *
 * 前提(モデルの割り切り):
 *   - 供給は最大32枚 (通常30 + スーパー2)。スループットもほぼ同程度なので全枚数を投げ切る。
 *   - 命中確率は近距離での par.probs を用いる (距離係数 ~1.0)。
 *   - 移動バケツは相手依存なので既定では候補に含めない (含めるオプションあり)。
 *   - 旗は掛かり容量 flagCap を超えると古い1枚が落ちて実質加点0 (シムの挙動と一致)。
 */

const FIXED_FOR_BONUS: ThrowTargetKey[] = ['flag', 'b1', 'b2', 'b3', 'desk1', 'desk2'];
const CANDIDATES: ThrowTargetKey[] = ['flag', 'desk1', 'desk2', 'b2', 'b3', 'b1'];

function baseProb(par: SimParams, g: ThrowTargetKey): number {
  const p = par.probs;
  switch (g) {
    case 'flag':
      return p.flag;
    case 'desk1':
    case 'desk2':
      return p.desk;
    case 'b2':
    case 'b3':
      return p.fixedB;
    case 'b1':
      return p.fixedB;
    case 'moving':
      return p.bucketStill;
  }
}

/** ゴール g に既に successes 回入れているときの、次の1枚の限界期待値 (通常/スーパー) */
function marginal(par: SimParams, g: ThrowTargetKey, successes: number, superRag: boolean): number {
  const mult = superRag ? 2 : 1;
  if (g === 'flag') {
    if (successes >= par.flagCapacity) return 0; // 容量超過は古い1枚が落ちて実質0
    const sat = Math.max(0.45, 1 - 0.02 * successes);
    return POINTS.flag * mult * par.probs.flag * sat;
  }
  return POINTS[g] * mult * baseProb(par, g);
}

export interface OptimalResult {
  /** 投げる順のゴール列 (通常弾) */
  order: ThrowTargetKey[];
  /** スーパー弾の割当 */
  superOrder: ThrowTargetKey[];
  /** ゴール別の割当枚数 */
  alloc: Record<string, number>;
  /** 期待総得点 (ボーナス込み) */
  expected: number;
  /** ボーナス走 (5固定を1枚ずつ) を含めた方が良いか */
  bonusWorth: boolean;
}

/**
 * 期待値最適な割当を計算する。
 * @param normalRags 通常弾の総数 (既定30)
 * @param superRags スーパー弾の総数 (既定2)
 */
export function solveOptimal(par: SimParams, normalRags = 30, superRags = 2): OptimalResult {
  const runGreedy = (forceBonus: boolean): { order: ThrowTargetKey[]; superOrder: ThrowTargetKey[]; ev: number } => {
    const succ: Record<string, number> = {};
    for (const g of CANDIDATES) succ[g] = 0;
    const order: ThrowTargetKey[] = [];
    const superOrder: ThrowTargetKey[] = [];
    let ev = 0;
    // 命中を「期待的」に扱う: 1枚投げると期待 prob 回成功する。飽和/容量は期待successで進める。
    const expSucc: Record<string, number> = {};
    for (const g of CANDIDATES) expSucc[g] = 0;

    // ボーナス確保: 5固定+旗を1枚ずつ最初に確保
    let bonusSecured = false;
    if (forceBonus) {
      for (const g of FIXED_FOR_BONUS) {
        order.push(g);
        ev += marginal(par, g, expSucc[g]!, false);
        expSucc[g]! += baseProb(par, g);
        succ[g]! += 1;
      }
      ev += POINTS.flag; // 全ゴール制覇ボーナス +100
      bonusSecured = true;
    }

    // スーパー弾: 旗が容量に余裕あれば旗(200)、無ければ机(40)へ
    for (let i = 0; i < superRags; i++) {
      let best: ThrowTargetKey = 'desk1';
      let bestV = -1;
      for (const g of CANDIDATES) {
        const v = marginal(par, g, expSucc[g]!, true);
        if (v > bestV) {
          bestV = v;
          best = g;
        }
      }
      superOrder.push(best);
      ev += bestV;
      expSucc[best]! += baseProb(par, best);
    }

    // 通常弾の water-filling
    const used = forceBonus ? FIXED_FOR_BONUS.length : 0;
    for (let i = 0; i < normalRags - used; i++) {
      let best: ThrowTargetKey = 'desk1';
      let bestV = -1;
      for (const g of CANDIDATES) {
        const v = marginal(par, g, expSucc[g]!, false);
        if (v > bestV) {
          bestV = v;
          best = g;
        }
      }
      order.push(best);
      ev += bestV;
      expSucc[best]! += baseProb(par, best);
    }
    void bonusSecured;
    void succ;
    return { order, superOrder, ev };
  };

  const withB = runGreedy(true);
  const withoutB = runGreedy(false);
  const bonusWorth = withB.ev >= withoutB.ev;
  const chosen = bonusWorth ? withB : withoutB;

  const alloc: Record<string, number> = {};
  for (const g of [...chosen.order, ...chosen.superOrder]) alloc[g] = (alloc[g] ?? 0) + 1;

  return {
    order: chosen.order,
    superOrder: chosen.superOrder,
    alloc,
    expected: Math.round(chosen.ev),
    bonusWorth,
  };
}

// ===================== 動的(受動ホライズン)ソルバ =====================

/** planOptimal に渡すライブ状態。試合中に補充ごとへ都度再計算する。 */
export interface OptimalContext {
  par: SimParams;
  /** 手持ちの通常弾・スーパー弾 (このバッチで割り当てる) */
  normalRags: number;
  superRags: number;
  /** 残り時間(秒)。これで投げ切れる枚数の上限が決まる */
  remainingSec: number;
  /** これまでの各ゴール成功数 (flag は累計ヒット)。飽和/ボーナス判定に使う */
  scored: Record<ThrowTargetKey, number>;
  /** 現在の旗の掛かり本数 (落下モデルの価値減に使う) */
  flagOcc: number;
  /** 各ゴールを撃つ射点距離(m)。距離依存命中率に使う */
  shotDist: Record<ThrowTargetKey, number>;
  /** 1投にかかる実時間(秒)。残り時間から投げられる枚数を出す */
  perThrow: number;
}

export interface OptimalPlan {
  order: ThrowTargetKey[];
  superOrder: ThrowTargetKey[];
}

function halfWidth(g: ThrowTargetKey): number {
  return g === 'flag' ? 0.3 : g === 'desk1' || g === 'desk2' ? 0.32 : 0.14;
}

/** 距離依存の命中率 (resolveThrowSpec と同じモデル) */
function distProb(ctx: OptimalContext, g: ThrowTargetKey): number {
  const sigma = (ctx.par.aimSpreadDeg * Math.PI) / 180;
  const d = ctx.shotDist[g] ?? 2.5;
  const f = Math.exp(-0.5 * ((d * sigma) / halfWidth(g)) ** 2);
  return baseProb(ctx.par, g) * f;
}

/**
 * 受動ホライズン最適: 手持ち弾を、残り時間で投げ切れる範囲だけ、距離依存命中率と
 * 旗の確率的落下(掛かり本数が多いほど価値減)を織り込んで、限界期待値の高いゴールへ配分する。
 * 全ゴール制覇ボーナスも考慮する。補充ごとにライブ状態で呼び直すことで動的に最良を追う。
 */
export function planOptimal(ctx: OptimalContext): OptimalPlan {
  const affordable = Math.max(0, Math.floor(ctx.remainingSec / Math.max(0.3, ctx.perThrow)));
  const nSuper = Math.min(ctx.superRags, affordable);
  const nNormal = Math.min(ctx.normalRags, Math.max(0, affordable - nSuper));
  // 旗の落下を織り込んだ価値保持係数 (掛かり本数が多いほど下がるが0にはしない)
  const retention = (occ: number): number => Math.max(0.15, 1 - ctx.par.flagFallPerSec * occ * 12);
  const bonusEarned = FIXED_FOR_BONUS.every((g) => (ctx.scored[g] ?? 0) > 0);

  const runGreedy = (forceBonus: boolean): { order: ThrowTargetKey[]; superOrder: ThrowTargetKey[]; ev: number } => {
    const succ: Record<string, number> = {};
    for (const g of CANDIDATES) succ[g] = ctx.scored[g] ?? 0;
    let flagOcc = ctx.flagOcc;
    const order: ThrowTargetKey[] = [];
    const superOrder: ThrowTargetKey[] = [];
    let ev = 0;
    let remNormal = nNormal;

    const marginal = (g: ThrowTargetKey, superRag: boolean): number => {
      const mult = superRag ? 2 : 1;
      const prob = distProb(ctx, g);
      if (g === 'flag') {
        const sat = Math.max(0.45, 1 - 0.02 * (succ.flag ?? 0));
        return POINTS.flag * mult * prob * sat * retention(flagOcc);
      }
      return POINTS[g] * mult * prob;
    };
    const applyThrow = (g: ThrowTargetKey, superRag: boolean): void => {
      ev += marginal(g, superRag);
      const prob = distProb(ctx, g);
      succ[g] = (succ[g] ?? 0) + prob;
      if (g === 'flag') flagOcc += prob;
    };

    // ボーナス確保: まだ未達成 (scored==0) の固定ゴールを1枚ずつ通常弾で押さえる
    if (forceBonus && !bonusEarned) {
      for (const g of FIXED_FOR_BONUS) {
        if ((ctx.scored[g] ?? 0) > 0 || remNormal <= 0) continue;
        order.push(g);
        applyThrow(g, false);
        remNormal--;
      }
      if (FIXED_FOR_BONUS.every((g) => (succ[g] ?? 0) > 0)) ev += 100; // 期待的に全制覇
    }

    for (let i = 0; i < nSuper; i++) {
      let best: ThrowTargetKey = 'desk1';
      let bv = -1;
      for (const g of CANDIDATES) {
        const v = marginal(g, true);
        if (v > bv) {
          bv = v;
          best = g;
        }
      }
      superOrder.push(best);
      applyThrow(best, true);
    }
    for (let i = 0; i < remNormal; i++) {
      let best: ThrowTargetKey = 'desk1';
      let bv = -1;
      for (const g of CANDIDATES) {
        const v = marginal(g, false);
        if (v > bv) {
          bv = v;
          best = g;
        }
      }
      order.push(best);
      applyThrow(best, false);
    }
    return { order, superOrder, ev };
  };

  const withB = runGreedy(true);
  const withoutB = runGreedy(false);
  const chosen = withB.ev >= withoutB.ev ? withB : withoutB;
  return { order: chosen.order, superOrder: chosen.superOrder };
}
