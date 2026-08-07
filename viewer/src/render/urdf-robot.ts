import * as THREE from 'three';
import type { TeamId } from '../config/field';
import type { RobotState } from '../sim/types';
import { loadUrdf, type UrdfModel } from './urdf';
import { JOINT_NAMES, mapJoints, newAccum, type JointAccum } from './urdf-joints';

/**
 * CAD そのもの（`cad/urdf/tr.urdf`）を読んで、試合シムの状態で**全関節を動かす**。
 *
 * ⚠ ここは「見た目を CAD に差し替える」だけの層ではない。関節の**上下限は
 *   URDF が持っている**ので、シムが出した角度が機構の可動域を超えていれば
 *   ここで頭打ちになる。図と食い違ったら、直すのは CAD かシムであって
 *   この層ではない。
 *
 * 関節（11 個）:
 *   wheel_fl/fr/rl/rr  continuous  メカナム 4 輪（M3508 ×4）
 *   turret_yaw         revolute    砲塔ヨー ±30°（M3508）
 *   shooter_pitch      revolute    仰角 20〜70°（M2006 + ウォーム）
 *   roller_upper/lower continuous  射出ローラー 対向 2 本（M3508 ×2）
 *   singulator         continuous  送給の分離ローラー（M2006）
 *   grabber_slide      prismatic   グラバー前後 0〜316mm（M3508 + ベルト）
 *   grabber_press      prismatic   上押さえ 0〜138mm（サーボ）
 *   fork_tilt          revolute    フォーク傾き 0〜22°（サーボ）
 */

const URDF_URL = `${import.meta.env.BASE_URL}urdf/tr.urdf`;
// ⚠ URDF の `filename` は **URDF から見た相対パス**（`meshes/base_link.stl`）。
//   ここに `urdf/meshes/` を渡すと `urdf/meshes/meshes/…` になり、Vite の
//   SPA フォールバックが index.html を 200 で返す。STLLoader はそれを
//   バイナリ STL と読んで「三角形 17 億個」を確保しようとして落ちる。
const MESH_BASE = `${import.meta.env.BASE_URL}urdf/`;

/**
 * 材質ごとの質感。⚠ **色は CAD（`tr_lib.MAT_COLOR`）が持っている**ので
 * ここでは触らない。決めるのは金属かどうか・つやの有無だけ。
 * 色まで持つと、CAD で材質を変えたのにビューアだけ前の色、が起きる。
 */
const FINISH: Record<
  string,
  { metalness: number; roughness: number; emissive?: number }
> = {
  tr_A5052: { metalness: 0.82, roughness: 0.34 }, // 切削アルミ（やや光る）
  tr_A6005C: { metalness: 0.72, roughness: 0.45 }, // アルマイトのフレーム
  tr_ADC12: { metalness: 0.68, roughness: 0.55 }, // ダイカスト（梨地）
  tr_SUS304: { metalness: 0.88, roughness: 0.28 }, // ステンレス板金
  tr_STEEL: { metalness: 0.9, roughness: 0.35 }, // スライドレール
  tr_MOTOR_SHAFT: { metalness: 0.95, roughness: 0.22 }, // 金メッキのフランジ
  tr_PETG: { metalness: 0.0, roughness: 0.72 }, // 3Dプリント（積層で粗い）
  tr_POM: { metalness: 0.0, roughness: 0.5 },
  tr_PC: { metalness: 0.0, roughness: 0.15 }, // ポリカ（つや）
  // 表示器の画面。⚠ **自分で光る**。周りの照明だけだと、地の色が黒紺なので
  // 「電源の入っていないモニタ」に見える。映している画そのものはプロモ側
  // （`cad/promo/tex/screen.png`）が持つので、ここでは点いていることだけ出す。
  tr_SCREEN: { metalness: 0.0, roughness: 0.06, emissive: 1.2 },
  tr_PP_DANPLA: { metalness: 0.0, roughness: 0.85 },
  tr_TEKCELL: { metalness: 0.0, roughness: 0.85 },
  tr_PLYWOOD: { metalness: 0.0, roughness: 0.9 },
  tr_URETHANE: { metalness: 0.0, roughness: 0.95 }, // 射出ローラー（食いつく）
  tr_SILICONE: { metalness: 0.0, roughness: 0.95 },
  tr_RUBBER: { metalness: 0.0, roughness: 0.95 },
  tr_SPONGE: { metalness: 0.0, roughness: 1.0 },
  tr_CABLE: { metalness: 0.0, roughness: 0.8 },
  tr_MOTOR: { metalness: 0.35, roughness: 0.5 }, // 黒アルマイトのケース
  tr_PCB: { metalness: 0.1, roughness: 0.6 },
  tr_BATTERY: { metalness: 0.05, roughness: 0.7 },
  tr_SENSOR: { metalness: 0.3, roughness: 0.55 },
  tr_ESTOP: { metalness: 0.0, roughness: 0.45 },
  tr_MASCOT: { metalness: 0.0, roughness: 0.95 }, // フェルト
  tr_MASCOT_SUIT: { metalness: 0.0, roughness: 0.95 },
  tr_MASCOT_TRIM: { metalness: 0.0, roughness: 0.95 },
  tr_MASCOT_RAG: { metalness: 0.0, roughness: 1.0 },
  tr_MASCOT_DARK: { metalness: 0.0, roughness: 0.8 },
};
const FINISH_DEFAULT = { metalness: 0.5, roughness: 0.5 };

/**
 * チーム色。⚠ **機体そのものは塗らない**（CAD の材質色をつぶさない）。
 * 青赤の区別は、地の色に**ごくわずか**寄せるだけにする。
 */
const TEAM_TINT: Record<TeamId, THREE.Color> = {
  blue: new THREE.Color(0.88, 0.93, 1.0),
  red: new THREE.Color(1.0, 0.9, 0.88),
};

export interface UrdfRobotOptions {
  team: TeamId;
  /** 読み込み進捗 (0..1)。UI のローディング表示用 */
  onProgress?: (frac: number) => void;
}

/** `RobotVisual` と同じ形で使える、CAD 実形状のロボット。 */
export class UrdfRobotVisual {
  readonly root = new THREE.Group();
  readonly team: TeamId;
  private model: UrdfModel | null = null;
  /** continuous 関節の積算角。⚠ 関節値の作り方は `urdf-joints.ts` が持つ */
  private acc: JointAccum = newAccum();
  private ready = false;

  constructor(opts: UrdfRobotOptions) {
    this.team = opts.team;
    this.root.name = `urdf-robot:${opts.team}`;
  }

  /** URDF とメッシュを読む。**完成してから** root に足す（歯抜けを見せない） */
  async load(onProgress?: (frac: number) => void): Promise<void> {
    const tint = TEAM_TINT[this.team];
    const model = await loadUrdf(URDF_URL, {
      meshBase: MESH_BASE,
      // 色は URDF（＝CAD の材質色）から。質感だけ材質名で決める
      makeMaterial: ([r, g, b, a], name) => {
        const f = FINISH[name] ?? FINISH_DEFAULT;
        const c = new THREE.Color(r, g, b).multiply(tint);
        return new THREE.MeshStandardMaterial({
          color: c,
          metalness: f.metalness,
          roughness: f.roughness,
          emissive: f.emissive ? c.clone() : new THREE.Color(0, 0, 0),
          emissiveIntensity: f.emissive ?? 0,
          transparent: a < 1,
          opacity: a,
        });
      },
      onProgress: (done, total) => onProgress?.(total ? done / total : 1),
    });
    this.model = model;
    this.root.add(model.root);
    this.ready = true;
  }

  get loaded(): boolean {
    return this.ready;
  }

  get triangles(): number {
    return this.model?.triangles ?? 0;
  }

  /** 関節名 → いまの値（デバッグ表示・検証用） */
  jointValues(): Record<string, number> {
    const out: Record<string, number> = {};
    for (const j of this.model?.joints.values() ?? []) out[j.name] = j.value;
    return out;
  }

  update(r: RobotState, dt: number, _matchT = 999): void {
    this.root.position.set(r.x, r.liftY, r.z);
    this.root.rotation.y = r.theta;
    const m = this.model;
    if (!m) return;
    const q = mapJoints(r, this.acc, dt);
    for (const n of JOINT_NAMES) m.setJoint(n, q[n]);
  }

  dispose(): void {
    this.model?.dispose();
    this.model = null;
    this.ready = false;
    this.root.clear();
  }
}
