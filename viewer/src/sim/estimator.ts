import type { Rng } from './rng';
import type { Seg } from './collision';
import {
  getMapSegs,
  getUpperSegs,
  localToWorld,
  scanToLocalPoints,
  type LidarScan,
  type Pose2,
} from './lidar';

/**
 * 本物の自己位置推定 (strategy.md §4.5.1):
 *   予測: 計測輪オドメトリ (2軸オムニ+エンコーダ) + ジャイロ — ノイズ入り
 *   補正: 点-線分 ICP。フィールドは四方をフェンスに囲まれ、既知地図とのマッチングに最適。
 * 表示される推定値・誤差はすべてこのパイプラインの実出力 (演出値ではない)。
 */

export interface LocalizerDiag {
  rmse: number;
  matched: number;
  iters: number;
  /** 直近スキャンでの補正量 (振動診断用) */
  corrMm: number;
  corrDeg: number;
}

const ODOM = {
  velNoise: 0.006, // 計測輪: 速度比例ノイズ (0.6%)
  velBias: 0.002, // m/s 定常
  gyroNoise: 0.004, // rad/s
};

function wrap(a: number): number {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

function closestOnSeg(px: number, pz: number, s: Seg): [number, number, number] {
  const ex = s.x2 - s.x1;
  const ez = s.z2 - s.z1;
  const len2 = ex * ex + ez * ez;
  let u = len2 > 1e-12 ? ((px - s.x1) * ex + (pz - s.z1) * ez) / len2 : 0;
  u = Math.max(0, Math.min(1, u));
  const cx = s.x1 + ex * u;
  const cz = s.z1 + ez * u;
  const dx = px - cx;
  const dz = pz - cz;
  return [cx, cz, dx * dx + dz * dz];
}

export class Localizer {
  est: Pose2;
  diag: LocalizerDiag = { rmse: 0, matched: 0, iters: 0, corrMm: 0, corrDeg: 0 };

  constructor(init: Pose2) {
    this.est = { ...init };
  }

  reset(p: Pose2): void {
    this.est = { ...p };
    this.diag = { rmse: 0, matched: 0, iters: 0, corrMm: 0, corrDeg: 0 };
  }

  /** 計測輪+ジャイロによるデッドレコニング (真の速度にセンサーノイズを加えて積分) */
  predict(vx: number, vz: number, omega: number, dt: number, rng: Rng): void {
    const g = (): number => (rng() + rng() + rng() - 1.5) / 0.6124;
    const sp = Math.hypot(vx, vz);
    const nx = vx + g() * (ODOM.velNoise * sp + ODOM.velBias);
    const nz = vz + g() * (ODOM.velNoise * sp + ODOM.velBias);
    const nw = omega + g() * ODOM.gyroNoise;
    this.est.x += nx * dt;
    this.est.z += nz * dt;
    this.est.theta = wrap(this.est.theta + nw * dt);
  }

  /**
   * ICP 補正: スキャン点を推定ポーズで世界へ置き、既知地図線分への最近点で
   * 剛体変換 (R, t) を閉形式で解く。3〜6回反復。外れ点 (相手ロボ等) はゲートで除外。
   * 前後2台のスキャンをまとめて1回のICPにかける (360°の拘束で縮退に強い)。
   */
  correct(...scans: LidarScan[]): LocalizerDiag {
    // 予測ポーズを保存。ICPは作業ポーズ (this.est) 上で収束させ、最後に
    // 「予測→ICP解」への部分更新でブレンドする (スキャンノイズ起因の振動抑制)
    const pred = { ...this.est };
    let total = 0;
    const parts = scans.map((s) => {
      const p = scanToLocalPoints(s);
      total += p.n;
      return p;
    });
    const pts = new Float32Array(total * 2);
    let n = 0;
    for (const p of parts) {
      pts.set(p.pts.subarray(0, p.n * 2), n * 2);
      n += p.n;
    }
    const segs = getMapSegs();
    const step = n > 160 ? Math.ceil(n / 160) : 1; // 最大160点に間引き
    const GATE2 = 0.45 * 0.45;
    let iters = 0;
    let rmse = 0;
    let matched = 0;
    for (let iter = 0; iter < 6; iter++) {
      iters++;
      const pairs: Array<[number, number, number, number]> = [];
      for (let i = 0; i < n; i += step) {
        const [wx, wz] = localToWorld(pts[i * 2]!, pts[i * 2 + 1]!, this.est);
        let best = GATE2;
        let bx = 0;
        let bz = 0;
        let found = false;
        for (const s of segs) {
          const [cx, cz, d2] = closestOnSeg(wx, wz, s);
          if (d2 < best) {
            best = d2;
            bx = cx;
            bz = cz;
            found = true;
          }
        }
        if (found) pairs.push([wx, wz, bx, bz]);
      }
      matched = pairs.length;
      if (matched < 8) break;
      let max = 0;
      let maz = 0;
      let mbx = 0;
      let mbz = 0;
      for (const [ax, az, bx, bz] of pairs) {
        max += ax;
        maz += az;
        mbx += bx;
        mbz += bz;
      }
      max /= matched;
      maz /= matched;
      mbx /= matched;
      mbz /= matched;
      let num = 0;
      let den = 0;
      let err = 0;
      for (const [ax, az, bx, bz] of pairs) {
        const dax = ax - max;
        const daz = az - maz;
        const dbx = bx - mbx;
        const dbz = bz - mbz;
        num += dax * dbz - daz * dbx; // Σ a×b
        den += dax * dbx + daz * dbz; // Σ a·b
        const ex = ax - bx;
        const ez = az - bz;
        err += ex * ex + ez * ez;
      }
      rmse = Math.sqrt(err / matched);
      // 剛体変換 T: 点群A(スキャン)→B(地図)。φ は (x,z) 平面の標準CCW回転
      const phi = Math.atan2(num, den);
      const c = Math.cos(phi);
      const s = Math.sin(phi);
      const tx = mbx - (c * max - s * maz);
      const tz = mbz - (s * max + c * maz);
      // ポーズに T を適用。前方=(sinθ,cosθ) 規約では θ' = θ − φ
      const nx = c * this.est.x - s * this.est.z + tx;
      const nz = s * this.est.x + c * this.est.z + tz;
      this.est.x = nx;
      this.est.z = nz;
      this.est.theta = wrap(this.est.theta - phi);
      if (rmse < 0.015 && Math.abs(phi) < 0.001) break;
    }

    // ---- 平滑化 (EKF的な部分更新) ----
    // 毎スキャンのICP最適解はノイズで数mm〜cm振れる。全跳びせず予測とブレンドする。
    const dx = this.est.x - pred.x;
    const dz = this.est.z - pred.z;
    const dth = wrap(this.est.theta - pred.theta);
    const mag = Math.hypot(dx, dz);
    let gPos = 0.5;
    let gTh = 0.6;
    if (matched < 8) {
      // マッチ不足: 補正しない (予測のみ)
      gPos = 0;
      gTh = 0;
    } else if (mag > 0.35 || Math.abs(dth) > 0.15) {
      // 外れスキャン (歪み大・遮蔽など): 弱く効かせる
      gPos = 0.15;
      gTh = 0.2;
    }
    this.est.x = pred.x + dx * gPos;
    this.est.z = pred.z + dz * gPos;
    this.est.theta = wrap(pred.theta + dth * gTh);

    this.diag = {
      rmse,
      matched,
      iters,
      corrMm: mag * gPos * 1000,
      corrDeg: ((Math.abs(dth) * gTh) * 180) / Math.PI,
    };
    return this.diag;
  }
}

/**
 * 相手ロボット推定 (strategy.md §4.5.5):
 * 上段LiDAR (0.5m) のスキャンから、既知静物にマッチしない点をクラスタリング。
 * 下段(0.12m)は教壇(H200)に遮られ相手が見えない — 2台目が必要な理由 (シムで実証)。
 * YOLO 単眼より距離が直接取れて堅い。ゴースト表示はこの実出力。
 */
export class OpponentTracker {
  est: { x: number; z: number } | null = null;
  private missT = 0;

  update(scan: LidarScan, selfEst: Pose2, dt: number): void {
    const { pts, n } = scanToLocalPoints(scan);
    const segs = getUpperSegs();
    const outliers: Array<[number, number]> = [];
    const GATE2 = 0.3 * 0.3;
    // 遠距離 (6m超) では相手車体に当たるレイが数本しかないため全点を使う
    for (let i = 0; i < n; i++) {
      const [wx, wz] = localToWorld(pts[i * 2]!, pts[i * 2 + 1]!, selfEst);
      let minD = Infinity;
      for (const s of segs) {
        const [, , d2] = closestOnSeg(wx, wz, s);
        if (d2 < minD) minD = d2;
        if (minD < GATE2) break;
      }
      if (minD >= GATE2) outliers.push([wx, wz]);
    }
    if (outliers.length >= 3) {
      // 最大クラスタ: 中央値中心の0.8m箱に入る点の平均
      const xs = outliers.map((p) => p[0]).sort((a, b) => a - b);
      const zs = outliers.map((p) => p[1]).sort((a, b) => a - b);
      const medX = xs[Math.floor(xs.length / 2)]!;
      const medZ = zs[Math.floor(zs.length / 2)]!;
      let sx = 0;
      let sz = 0;
      let m = 0;
      for (const [x, z] of outliers) {
        if (Math.abs(x - medX) < 0.8 && Math.abs(z - medZ) < 0.8) {
          sx += x;
          sz += z;
          m++;
        }
      }
      if (m >= 3) {
        const cx = sx / m;
        const cz = sz / m;
        // 車体表面点の重心 → 中心へ0.2m押し込む (観測は手前面に偏るため)
        const dx = cx - selfEst.x;
        const dz = cz - selfEst.z;
        const dl = Math.hypot(dx, dz) || 1;
        const px = cx + (dx / dl) * 0.2;
        const pz = cz + (dz / dl) * 0.2;
        this.est = this.est
          ? { x: this.est.x * 0.55 + px * 0.45, z: this.est.z * 0.55 + pz * 0.45 }
          : { x: px, z: pz };
        this.missT = 0;
        return;
      }
    }
    this.missT += dt;
    if (this.missT > 1.2) this.est = null;
  }
}
