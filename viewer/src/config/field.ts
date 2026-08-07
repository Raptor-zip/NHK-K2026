/**
 * フィールド座標系 — 公式フィールド図 (0415版 PDF) を実測読取りした値 (2026-07-04 修正)
 *
 *   z: 教壇と直交する方向 (青陣 z>0 / 赤陣 z<0)。教壇は z∈[-0.3, +0.3]
 *   x: 教壇と平行な方向 (全幅10.5m)。スタートゾーン/補充スポット/コントロールSTは x=+ 側の縁
 *   y: 高さ
 *
 * 図面の要点 (300dpi実測精査 2026-07-04、寸法連鎖 5400/5400/10500 で完全に閉じることを確認):
 *   - フィールドは左右「鏡映」対称 (点対称ではない): 赤陣 = 青陣の z 反転、x は同値
 *   - 各陣の床の奥行き 5.4m (フェンス内 z=±5.7)、幅 10.5m (x=±5.25)。フェンスは L字断面 H150×D150
 *   - x=-0.55 の行 (中心線から550mm南寄り): バケツ①(z=0.87) → 旗(z=3.025) → 椅子(z=4.98)
 *     寸法連鎖は「シート端-端」: 490+460+1500+450+1760+340+400=5400
 *   - 固定バケツ②/③: z=±1.48 (シート教壇側端→教壇端=1000)、x=+1.27 / -2.37
 *   - 机①(x=+2.295)/机②(x=-3.395): z=±3.855。机は全て W650 が x方向・D450 が z方向
 *   - 机①②の棚開口は教壇側 / 補充机はスタートゾーン側 / CS机はサイドフェンス側
 *   - コントロールステーションは「フェンス外」ではなくフィールド内コーナーの障害物 (北・東フェンス内縁に接触)
 *   - 旗だけ鏡映でなく「z反転+ヨー180°」: 布・横棒は 赤=+x向き / 青=-x向き
 */

export type TeamId = 'blue' | 'red';
export type GoalKey = 'flag' | 'b1' | 'b2' | 'b3' | 'desk1' | 'desk2';
export type ThrowTargetKey = GoalKey | 'moving';

export interface Vec2 {
  x: number;
  z: number;
}

export const FIELD = {
  /** x方向 内寸 (教壇と平行) */
  w: 10.5,
  /** z方向 内寸 (5.4 + 0.6 + 5.4) */
  l: 11.4,
  halfL: 5.4,
  podium: { w: 10.5, d: 0.6, h: 0.2 },
  fenceH: 0.15,
} as const;

/**
 * 赤陣 (z<0) 側の配置。青陣は z 反転の「鏡映」(mirror)。
 * 値は図面の寸法連鎖から算出 (中心座標, 単位m)。
 */
export const RED_SIDE = {
  flag: { x: -0.55, z: -3.025 },
  b1: { x: -0.55, z: -0.87 },
  b2: { x: 1.27, z: -1.48 },
  b3: { x: -2.37, z: -1.48 },
  desk1: { x: 2.295, z: -3.855 },
  desk2: { x: -3.395, z: -3.855 },
  chair: { x: -0.55, z: -4.98 },
  start: { x: 4.75, z: -1.8 },
  resup: { x: 4.75, z: -1.075 },
  control: { x: 4.925, z: -5.475 },
} as const;

/** 旗布・横棒の突き出し方向 (x符号)。図面で旗だけ鏡映でなく 180° 回っている */
export const FLAG_CLOTH_DIR: Record<TeamId, 1 | -1> = { red: 1, blue: -1 };

export const DIMS = {
  flagBarY: 3.0,
  flagBarHalfW: 0.3,
  bucketH: 0.255,
  bucketRimR: 0.137,
  pedestal: { b1: 0, b2: 0.6, b3: 0.3 } as Record<'b1' | 'b2' | 'b3', number>,
  /** 机: W650 が x方向 (教壇と平行)、D450 が z方向。机①②の棚開口は教壇向き */
  desk: { wx: 0.65, wz: 0.45, h: 0.76, shelfY: 0.5 },
  chair: { wx: 0.36, wz: 0.4, seatY: 0.46, totalH: 0.807 },
  /** 補充スポット机: 同一品 (W650=x)。開口はスタートゾーン側 */
  resupDesk: { wx: 0.65, wz: 0.45, h: 0.76 },
} as const;

export const POINTS: Record<ThrowTargetKey, number> = {
  flag: 100,
  b1: 1,
  b2: 5,
  b3: 5,
  desk1: 20,
  desk2: 20,
  moving: 100,
};

export const GOAL_LABEL: Record<ThrowTargetKey, string> = {
  flag: '旗',
  b1: '固定バケツ①',
  b2: '固定バケツ②',
  b3: '固定バケツ③',
  desk1: '机①',
  desk2: '机②',
  moving: '移動バケツ',
};

/** 鏡映: 赤陣⇔青陣は z 反転のみ (x はそのまま) */
/**
 * Q2: コントロールステーションの3隅に立てる反射材マーカー柱 (赤陣)。
 * CS机 (中心 x=4.925, z=-5.475) の隅 ±(0.29, 0.19) に配置し、フェンス隅を埋める4隅目は除外。
 * L字型の3点配置で ICP / 上段クラスタ照合の対応付けを一意にする。
 * 柱 r0.03 の外縁は +x隅で 5.245 < フェンス内縁5.25、-z隅で -5.695 > -5.7 に収まりはみ出さない。
 * 高さ 0.78m — 下段 (0.12m)・上段 (0.50m) 両方のスキャン面にかかる。
 */
export const REFLECTORS_RED: readonly Vec2[] = [
  { x: RED_SIDE.control.x - 0.29, z: RED_SIDE.control.z + 0.19 }, // 場内側の隅
  { x: RED_SIDE.control.x + 0.29, z: RED_SIDE.control.z + 0.19 }, // +x フェンス寄りの隅
  { x: RED_SIDE.control.x - 0.29, z: RED_SIDE.control.z - 0.19 }, // -z フェンス寄りの隅
];

export function mirror<T extends Vec2>(p: T): T {
  return { ...p, z: -p.z };
}

/** team が攻撃する側の平面座標 (blue は赤陣そのまま、red は鏡映=青陣) */
export function attackPos(team: TeamId, key: keyof typeof RED_SIDE): Vec2 {
  const p = RED_SIDE[key];
  return team === 'blue' ? { x: p.x, z: p.z } : { x: p.x, z: -p.z };
}

/** team の自陣側の平面座標 */
export function homePos(team: TeamId, key: keyof typeof RED_SIDE): Vec2 {
  return attackPos(team === 'blue' ? 'red' : 'blue', key);
}

/** 射撃時に狙う 3D 点 */
export function goalAimPoint(team: TeamId, key: GoalKey): { x: number; y: number; z: number } {
  const p = attackPos(team, key);
  switch (key) {
    case 'flag': {
      // 横棒はポールから片持ち — 相手陣の旗の布方向の中点を狙う
      const owner: TeamId = team === 'blue' ? 'red' : 'blue';
      return { x: p.x + FLAG_CLOTH_DIR[owner] * 0.31, y: DIMS.flagBarY, z: p.z };
    }
    case 'b1':
    case 'b2':
    case 'b3':
      return { x: p.x, y: DIMS.pedestal[key] + DIMS.bucketH + 0.02, z: p.z };
    case 'desk1':
    case 'desk2': {
      // 机下棚: 開口は教壇側 → 教壇方向へ手前0.28mを狙う
      const dz = p.z > 0 ? -0.28 : 0.28;
      return { x: p.x, y: DIMS.desk.shelfY - 0.1, z: p.z + dz };
    }
  }
}

/**
 * 機体の寸法 [m]。⚠ **すべて CAD から来た値**で、ここで丸めない。
 * 出どころは `cad/src/tr_params.py` と `cad/urdf/tr.urdf`。
 */
export const ROBOT = {
  /** 車体後端（base_link の -X 端）。cad の chassis collision -421mm */
  rearOverhang: 0.421,
  /** 車体前端（+X 端）。同 +421mm */
  frontOverhang: 0.421,
  /** 櫛歯の先端が届く範囲（base_link の -X 側）。
   *  全閉 = RAIL_X0 - FORK_TINE_EXT = 275.4 + 89 = 364.4mm
   *  全開 = さらに GRAB_STROKE 316mm = 680.4mm（規定の「山の遠端 -680」に届く） */
  forkReachMin: 0.3644,
  forkReachMax: 0.6804,
  /** 櫛歯の上面高さ。歯下面 761mm = 机上面 760 + 1mm */
  forkTopY: 0.763,
  /** 机に寄せるときのすきま。レーザ測距の分解能と操縦の余裕 */
  deskClearance: 0.03,
  /**
   * 移動バケツ（自陣が的として担ぐ方）。⚠ **車体中心ではない。**
   * CAD は `tr_assembly.BUCKET_X = -70mm` / `y = 0`、マスト上端に座る。
   * 局所座標はビューア流儀（X 左 / Z 前）なので、URDF の (x=-70, y=0) は
   * ここでは (x=0, z=-0.070)＝**中心線上の 70mm 後ろ**になる。
   * 以前は (0.24, -0.24) と斜め後ろに置いていて、CAD 実形状で描くと
   * 弾がバケツの脇 340mm を素通りしていた。
   */
  bucket: {
    x: 0,
    z: -0.07,
    /** 上面 = BUCKET_SEAT_Z(1175) + BUCKET_H(255)。規定 3.2.3b の 1200..2100 内 */
    topY: 1.43,
    /** エンテック PO-24A φ273 */
    r: 0.1365,
  },
} as const;

/**
 * 補充机に付けるときの機体ポーズ。
 *
 * ⚠ **フォークは車体の後ろ（base_link の -X）にある。** 砲塔は +X なので、
 *   補充のときは**攻撃方位とは逆を向く**（机に背を向ける）。
 *   「側面グラバーだから車体旋回ゼロ」は CAD と違う。実際 CAD の
 *   `grabber_slide` は軸 (-1,0,0) で、櫛歯は -X へ 316mm 出る。
 *
 * 位置は「車体後端が机の縁の手前 30mm」。このとき櫛歯の先端は
 * 机の縁から 680.4 - 421 - 30 = 229mm 入った所まで届く（机の奥行 450mm の
 * 中央 225mm を越える＝山の向こう側に歯が入る）。
 */
export function resupPose(team: TeamId): { x: number; z: number; theta: number } {
  const s = team === 'blue' ? 1 : -1;
  const desk = homePos(team, 'resup');
  const nearEdge = desk.z + (DIMS.resupDesk.wz / 2) * s; // スタートゾーン側の縁
  return {
    x: desk.x,
    z: nearEdge + (ROBOT.rearOverhang + ROBOT.deskClearance) * s,
    // 前方 = (sinθ, cosθ)。後ろを机（-s 方向）へ向ける → 前方は +s
    theta: s > 0 ? 0 : Math.PI,
  };
}

/** 櫛歯の先端が机の縁から何 m 入るか（全開時）。負なら届いていない */
export function forkPenetration(): number {
  return ROBOT.forkReachMax - ROBOT.rearOverhang - ROBOT.deskClearance;
}

/** 主要ウェイポイント (team の自陣座標系: blue は z>0) */
export function waypoints(team: TeamId) {
  const s = team === 'blue' ? 1 : -1;
  return {
    start: { x: 4.75, z: 1.8 * s },
    /** 補充机の開口 (スタートゾーン側) 正面。⚠ CAD 由来 (`resupPose`) */
    resupFront: { x: resupPose(team).x, z: resupPose(team).z },
    /** ボーナス走の射撃点 (教壇際に各ゴール正面) */
    fireDesk1: { x: 2.295, z: 0.95 * s },
    fireB2: { x: 1.27, z: 0.95 * s },
    /** 旗+固定バケツ①正面 (x=-0.55 の行)。主戦場 */
    fireR: { x: -0.55, z: 0.88 * s },
    fireB3: { x: -2.37, z: 0.95 * s },
    fireDesk2: { x: -3.395, z: 0.95 * s },
  };
}
