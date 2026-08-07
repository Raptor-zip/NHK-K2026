import * as THREE from 'three';
import { homePos, ROBOT, waypoints, type TeamId } from '../config/field';
import type { RobotState } from '../sim/types';
import { mascotTexture, ragTexture } from './textures';

/**
 * ロボットの見た目 (strategy.md の FLAGSHIP 構成)。
 * 20mm角アルミフレーム構造 + ポリカパネルで実機らしく組む:
 * メカナム4輪(M3508モーター付き・逆運動学で実回転) / 旋回砲塔+対向ローラー /
 * ホッパー・送給 / 補充机側スタックグラバー / 椅子+マスコット / 移動バケツ / LED
 */

const DEG = Math.PI / 180;

function lam(color: number): THREE.MeshLambertMaterial {
  return new THREE.MeshLambertMaterial({ color });
}

const ALU = new THREE.MeshStandardMaterial({ color: 0xc9ced6, metalness: 0.75, roughness: 0.38 });
const PANEL = new THREE.MeshLambertMaterial({
  color: 0x1d2430,
  transparent: true,
  opacity: 0.35,
  side: THREE.DoubleSide,
});
const PCB = lam(0x1c6b3a);
const CABLE = lam(0x111318);

interface Wheel {
  hub: THREE.Group;
  rollers: THREE.Mesh[];
  steer?: THREE.Group;
  swerve?: boolean;
  spinDir?: number;
  x: number;
  z: number;
}

function wrapAngle(a: number): number {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

function initialChassisYaw(team: TeamId): number {
  return team === 'blue' ? Math.PI : 0;
}

function mascotYawToResupply(team: TeamId): number {
  const start = waypoints(team).start;
  const resup = homePos(team, 'resup');
  const theta = initialChassisYaw(team);
  const dx = resup.x - start.x;
  const dz = resup.z - start.z;
  const lx = dx * Math.cos(theta) - dz * Math.sin(theta);
  const lz = dx * Math.sin(theta) + dz * Math.cos(theta);
  return Math.atan2(lx, lz);
}

export class RobotVisual {
  readonly root = new THREE.Group();
  private wheels: Wheel[] = [];
  // 足回りは両方式を構築しておき、r.swerve に応じて可視/アニメを切り替える (途中変更に即応)
  private readonly driveMecanum = new THREE.Group();
  private readonly driveSwerve = new THREE.Group();
  private mecanumWheels: Wheel[] = [];
  private swerveWheels: Wheel[] = [];
  private curSwerve: boolean | null = null;
  private turretYawG: THREE.Group | null = null;
  private turretPitchG: THREE.Group | null = null;
  private shooterRollers: THREE.Mesh[] = [];
  private feedRag: THREE.Mesh | null = null;
  private grabber: THREE.Group | null = null;
  private grabRag: THREE.Mesh | null = null;
  private led: THREE.Mesh;
  private rollerAngle = 0;
  private measureL: THREE.Mesh | null = null;
  private measureR: THREE.Mesh | null = null;
  private measureS: THREE.Mesh | null = null;
  private readonly bucketDeployG = new THREE.Group();
  private readonly shooterDeployG = new THREE.Group();
  private readonly telescopingPosts: Array<{ mesh: THREE.Mesh; baseY: number; height: number; stowedTopY: number }> = [];
  private readonly mastTopParts: THREE.Object3D[] = [];
  private readonly bucketStowOffsetY: number;
  private readonly shooterStowOffsetY: number;
  readonly pipCamera: THREE.PerspectiveCamera | null = null;

  /** 20mm角アルミ材 */
  private alu(lx: number, ly: number, lz: number, x: number, y: number, z: number): THREE.Mesh {
    const m = new THREE.Mesh(new THREE.BoxGeometry(lx, ly, lz), ALU);
    m.position.set(x, y, z);
    m.castShadow = true;
    this.root.add(m);
    return m;
  }

  private aluBetween(
    a: [number, number, number],
    b: [number, number, number],
    t = 0.016,
  ): THREE.Mesh {
    const va = new THREE.Vector3(...a);
    const vb = new THREE.Vector3(...b);
    const mid = va.clone().add(vb).multiplyScalar(0.5);
    const dir = vb.clone().sub(va);
    const m = new THREE.Mesh(new THREE.BoxGeometry(t, t, dir.length()), ALU);
    m.position.copy(mid);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir.normalize());
    m.castShadow = true;
    this.root.add(m);
    return m;
  }

  /** 立方フレーム (下段・上段の四辺 + 4隅の柱) */
  private frameBox(half: number, yBot: number, yTop: number): void {
    const t = 0.02;
    const len = half * 2 + t;
    for (const y of [yBot, yTop]) {
      this.alu(len, t, t, 0, y, -half);
      this.alu(len, t, t, 0, y, half);
      this.alu(t, t, len - t * 2, -half, y, 0);
      this.alu(t, t, len - t * 2, half, y, 0);
    }
    const h = yTop - yBot - t;
    for (const [px, pz] of [
      [-half, -half],
      [half, -half],
      [-half, half],
      [half, half],
    ] as const) {
      this.alu(t, h, t, px, (yBot + yTop) / 2, pz);
    }
  }

  private deploymentProgress(matchT: number): number {
    const p = Math.max(0, Math.min(1, (matchT - 0.2) / 2.6));
    return p * p * (3 - 2 * p);
  }

  private applyDeployment(matchT: number): void {
    const p = this.deploymentProgress(matchT);
    this.bucketDeployG.position.y = this.bucketStowOffsetY * (1 - p);
    this.shooterDeployG.position.y = this.shooterStowOffsetY * (1 - p);
    for (const post of this.telescopingPosts) {
      const deployedTopY = post.baseY + post.height;
      const stowedTopY = Math.max(post.baseY + 0.08, Math.min(post.stowedTopY, deployedTopY));
      const topY = stowedTopY + (deployedTopY - stowedTopY) * p;
      const scaleY = Math.max(0.01, (topY - post.baseY) / post.height);
      post.mesh.scale.y = scaleY;
      post.mesh.position.y = post.baseY + (post.height * scaleY) / 2;
    }
    for (const part of this.mastTopParts) part.visible = p > 0.86;
  }

  constructor(team: TeamId, bucketTopY: number, detailed: boolean) {
    const teamCol = team === 'blue' ? 0x1f5fb0 : 0xb52731;
    this.bucketStowOffsetY = -Math.max(0, bucketTopY - 1.16);
    this.shooterStowOffsetY = detailed ? -0.24 : 0;
    this.root.add(this.bucketDeployG, this.shooterDeployG);
    this.root.add(this.driveMecanum, this.driveSwerve);

    // ===== アルミフレームシャシ =====
    this.frameBox(0.31, 0.09, 0.4);
    // 側面・後面の筋交い。ポリカの奥に実機らしい三角構造を見せる。
    for (const sx of [-1, 1] as const) {
      this.aluBetween([sx * 0.315, 0.11, -0.29], [sx * 0.315, 0.39, 0.29]);
      this.aluBetween([sx * 0.315, 0.39, -0.29], [sx * 0.315, 0.11, 0.29]);
    }
    this.aluBetween([-0.29, 0.11, -0.315], [0.29, 0.39, -0.315]);
    this.aluBetween([-0.29, 0.39, -0.315], [0.29, 0.11, -0.315]);
    // ポリカパネル (側面2面 + 前面)
    for (const sx of [-1, 1] as const) {
      const p = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.27), PANEL);
      p.rotation.y = Math.PI / 2;
      p.position.set(sx * 0.315, 0.245, 0);
      this.root.add(p);
    }
    const front = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.27), PANEL);
    front.position.set(0, 0.245, 0.315);
    this.root.add(front);
    // チームカラーのバンパーパネル
    const bumper = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.06, 0.015), lam(teamCol));
    bumper.position.set(0, 0.075, 0.325);
    this.root.add(bumper);
    const bumper2 = bumper.clone();
    bumper2.position.z = -0.325;
    this.root.add(bumper2);
    // トップデッキ (アルミ複合板)
    const deck = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.012, 0.62), lam(0x272d38));
    deck.position.set(0, 0.415, 0);
    deck.castShadow = true;
    this.root.add(deck);

    // ===== 電装 (デッキ上) =====
    const battery = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.065, 0.08), lam(0x2c3a4e));
    battery.position.set(-0.2, 0.455, 0.18);
    battery.castShadow = true;
    this.root.add(battery);
    for (const dz of [-0.02, 0.02]) {
      const strap = new THREE.Mesh(new THREE.BoxGeometry(0.152, 0.067, 0.015), lam(0xd9b23a));
      strap.position.set(-0.2, 0.455, 0.18 + dz);
      this.root.add(strap);
    }
    const devBoard = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.01, 0.07), PCB);
    devBoard.position.set(-0.18, 0.425, -0.15);
    this.root.add(devBoard);
    const rpi = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.01, 0.05), PCB);
    rpi.position.set(-0.02, 0.425, -0.18);
    this.root.add(rpi);
    for (const x of [0.08, 0.16, 0.24]) {
      const esc = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.014, 0.09), lam(0x202632));
      esc.position.set(x, 0.428, -0.17);
      this.root.add(esc);
    }
    for (const x of [-0.15, -0.05, 0.05, 0.15]) {
      const cable = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.34, 8), CABLE);
      cable.rotation.z = Math.PI / 2;
      cable.position.set(x, 0.445, 0.02);
      this.root.add(cable);
    }

    // 非常停止スイッチ。黄色ベース+赤ボタンを前後に配置。
    for (const z of [0.345, -0.345]) {
      const base = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.014, 16), lam(0xffd23e));
      base.rotation.x = Math.PI / 2;
      base.position.set(-0.24, 0.2, z);
      this.root.add(base);
      const knob = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.018, 16), lam(0xd7000f));
      knob.rotation.x = Math.PI / 2;
      knob.position.set(-0.24, 0.2, z + Math.sign(z) * 0.012);
      this.root.add(knob);
    }

    // ===== 駆動輪 =====
    const wheelPos: Array<[number, number, number]> = [
      [-0.345, 0.0635, 0.27],
      [0.345, 0.0635, 0.27],
      [-0.345, 0.0635, -0.27],
      [0.345, 0.0635, -0.27],
    ];
    wheelPos.forEach(([x, y, z], i) => {
      // 両方式を構築して driveMecanum / driveSwerve に振り分ける。r.swerve で可視/アニメを切替。
      // --- メカナムホイール + M3508 ---
      {
        const mount = new THREE.Group();
        mount.position.set(x, y, z);
        mount.rotation.z = Math.PI / 2;
        const hub = new THREE.Group();
        const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.05, 16), lam(0x2b2b31));
        tire.castShadow = true;
        hub.add(tire);
        const hubcap = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.028, 0.054, 12), ALU);
        hub.add(hubcap);
        const rollers: THREE.Mesh[] = [];
        const tilt = (i === 0 || i === 3 ? 45 : -45) * DEG;
        for (let k = 0; k < 8; k++) {
          const a = (k / 8) * Math.PI * 2;
          const rl = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.055, 8), lam(0x8a8f98));
          const holder = new THREE.Group();
          holder.rotation.y = a;
          const arm = new THREE.Group();
          arm.position.set(0.0635 - 0.014, 0, 0);
          arm.rotation.z = tilt;
          arm.add(rl);
          holder.add(arm);
          hub.add(holder);
          rollers.push(rl);
        }
        mount.add(hub);
        this.driveMecanum.add(mount);
        this.mecanumWheels.push({ hub, rollers, x, z });
        const mx = x - Math.sign(x) * 0.055;
        const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.021, 0.021, 0.05, 12), lam(0x15171c));
        motor.rotation.z = Math.PI / 2;
        motor.position.set(mx, y, z);
        motor.castShadow = true;
        this.driveMecanum.add(motor);
        const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.008, 12), ALU);
        cap.rotation.z = Math.PI / 2;
        cap.position.set(mx - Math.sign(x) * 0.026, y, z);
        this.driveMecanum.add(cap);
      }
      // --- 独立ステア(スワーブ)モジュール ---
      {
        const steer = new THREE.Group();
        steer.position.set(x, y + 0.018, z);
        const turntable = new THREE.Mesh(new THREE.CylinderGeometry(0.058, 0.064, 0.028, 18), lam(0x222834));
        turntable.position.y = 0.02;
        steer.add(turntable);
        const forkL = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.065, 0.018), ALU);
        forkL.position.set(-0.038, -0.005, 0);
        const forkR = forkL.clone();
        forkR.position.x = 0.038;
        steer.add(forkL, forkR);
        const hub = new THREE.Group();
        hub.rotation.z = Math.PI / 2;
        const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.052, 0.05, 18), lam(0x24262d));
        tire.castShadow = true;
        hub.add(tire);
        const hubcap = new THREE.Mesh(new THREE.CylinderGeometry(0.023, 0.023, 0.054, 12), ALU);
        hub.add(hubcap);
        // ステア方向がひと目で分かる指示フィン (モジュールと一緒に回る)
        const pointer = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.014, 0.09), lam(0xffd23e));
        pointer.position.set(0, 0.045, 0.052);
        steer.add(pointer);
        steer.add(hub);
        this.driveSwerve.add(steer);
        this.swerveWheels.push({ hub, rollers: [], steer, swerve: true, x, z });
        // ステアモーター (小さく、モジュール直上に控えめに)
        const steerMotor = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.035, 12), lam(0x111318));
        steerMotor.position.set(x, y + 0.11, z);
        this.driveSwerve.add(steerMotor);
      }
    });

    {
      // ===== 計測輪 ×3 コの字配置 (メカナムの外観。driveMecanum にまとめて可視切替)
      //       前後計測×2 (左右対称・軸=x) + 横計測×1 (後方中央・軸=z)
      //       前後2輪の差動からヨーレートも直接計測でき、ジャイロと相互補正できる =====
      const mkMeasure = (px: number, pz: number, axisAlongX: boolean): THREE.Mesh => {
        const arm = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.05, 0.012), ALU);
        arm.position.set(px, 0.062, pz);
        this.driveMecanum.add(arm);
        const pivot = new THREE.Group();
        pivot.position.set(px, 0.026, pz);
        if (axisAlongX) pivot.rotation.z = Math.PI / 2;
        else pivot.rotation.x = Math.PI / 2;
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.012, 14), lam(0xd9dee6));
        // 樽ローラー風の刻み (オムニ表現)
        for (let k = 0; k < 6; k++) {
          const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.014, 6), lam(0x555b66));
          const a = (k / 6) * Math.PI * 2;
          pin.position.set(Math.cos(a) * 0.024, 0, Math.sin(a) * 0.024);
          pin.rotation.x = Math.PI / 2;
          pin.rotation.z = a;
          wheel.add(pin);
        }
        pivot.add(wheel);
        this.driveMecanum.add(pivot);
        return wheel;
      };
      this.measureL = mkMeasure(-0.15, 0, true); // 前後計測・左 (軸=x)
      this.measureR = mkMeasure(0.15, 0, true); // 前後計測・右 (軸=x)
      this.measureS = mkMeasure(0, -0.15, false); // 横計測・後方中央 (軸=z)

      // ===== 雑巾巻き込み防止 (T20): 全周スカート + ホイールフェンダー =====
      const skirtMat = lam(0x14181f);
      for (const sz of [-1, 1] as const) {
        const skirt = new THREE.Mesh(new THREE.BoxGeometry(0.66, 0.075, 0.008), skirtMat);
        skirt.position.set(0, 0.052, sz * 0.334);
        this.driveMecanum.add(skirt);
      }
      for (const sx of [-1, 1] as const) {
        const skirt = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.075, 0.5), skirtMat);
        skirt.position.set(sx * 0.318, 0.052, 0);
        this.driveMecanum.add(skirt);
      }
      for (const [wx, wz] of [
        [-0.345, 0.27],
        [0.345, 0.27],
        [-0.345, -0.27],
        [0.345, -0.27],
      ] as const) {
        const fender = new THREE.Mesh(
          new THREE.CylinderGeometry(0.072, 0.072, 0.062, 14, 1, true, -Math.PI / 2, Math.PI),
          new THREE.MeshLambertMaterial({ color: 0x20242c, side: THREE.DoubleSide }),
        );
        fender.rotation.z = Math.PI / 2;
        fender.position.set(wx, 0.0635, wz);
        this.driveMecanum.add(fender);
      }
    }
    // 初期表示はチーム既定 (青=メカナム / 赤=独ステ)。以降は update() が r.swerve で切替。
    this.setDrivetrain(team === 'red');

    // ===== 椅子 (5号・脚切断で低搭載) + マスコット =====
    // FAQ 4.1 Q10: スタンバイ時はマスコット正面を補充スポットへ正対させる。
    // 車体は攻撃方位のままなので、椅子ごと補充スポット方向へわずかに振って搭載する。
    const chairG = new THREE.Group();
    chairG.position.set(0, 0, -0.17);
    chairG.rotation.y = mascotYawToResupply(team);
    this.root.add(chairG);
    const chairMat = lam(0xd9c9a3);
    for (const [lx, lz] of [
      [-0.12, 0.11],
      [0.12, 0.11],
      [-0.12, -0.11],
      [0.12, -0.11],
    ] as const) {
      const stub = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.05, 8), lam(0x777c84));
      stub.position.set(lx, 0.445, lz);
      chairG.add(stub);
    }
    const seat = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.025, 0.3), chairMat);
    seat.position.set(0, 0.48, 0);
    seat.castShadow = true;
    chairG.add(seat);
    const backrest = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.26, 0.025), chairMat);
    backrest.position.set(0, 0.63, -0.15);
    backrest.castShadow = true;
    chairG.add(backrest);
    const mascotBody = new THREE.Mesh(
      new THREE.BoxGeometry(0.22, 0.3, 0.18),
      lam(team === 'blue' ? 0x2b7fd4 : 0xd44a52),
    );
    mascotBody.position.set(0, 0.64, 0.02);
    mascotBody.castShadow = true;
    chairG.add(mascotBody);
    // 顔はローカル +z。chairG の取付角で補充スポットへ向ける。
    const headSkin = lam(0xffd75e);
    const faceMat = new THREE.MeshLambertMaterial({ map: mascotTexture() });
    const head = new THREE.Mesh(
      new THREE.BoxGeometry(0.2, 0.2, 0.18),
      [headSkin, headSkin, headSkin, headSkin, faceMat, headSkin],
    );
    head.position.set(0, 0.91, 0.02);
    head.castShadow = true;
    chairG.add(head);

    // ===== マスト + 移動バケツ =====
    // ⚠ **バケツは車体中心線の 70mm 後ろ**（CAD `BUCKET_X = -70`）。マストは
    //   側面トラスの延長で |X| = 0.35 の 2 本、Z = -0.275。以前は片側の角
    //   (0.24, -0.24) に 1 本だけ立てて斜めに担いでいて、CAD 実形状に
    //   切り替えるとバケツが 340mm 横へ飛んでいた。
    const bx = ROBOT.bucket.x;
    const bz = ROBOT.bucket.z;
    const mastZ = -0.275;
    const mastH = bucketTopY - 0.255;
    const mastA = this.alu(0.02, mastH - 0.09, 0.02, 0.35, (mastH + 0.09) / 2, mastZ);
    const mastB = this.alu(0.02, mastH - 0.09, 0.02, -0.35, (mastH + 0.09) / 2, mastZ);
    // 片持ちの受け梁（-X 側の柱から前へ出て、中心線でバケツを座らせる）
    const mastTop = this.alu(0.344, 0.02, 0.435, 0, mastH - 0.01, (mastZ + 0.15) / 2);
    const mastBraceA = this.aluBetween([0.35, 0.4, mastZ], [0.35, bucketTopY - 0.28, mastZ - 0.04], 0.014);
    const mastBraceB = this.aluBetween([-0.35, 0.4, mastZ], [-0.35, bucketTopY - 0.28, mastZ - 0.04], 0.014);
    this.telescopingPosts.push(
      { mesh: mastA, baseY: 0.09, height: mastH - 0.09, stowedTopY: 1.12 },
      { mesh: mastB, baseY: 0.09, height: mastH - 0.09, stowedTopY: 1.12 },
    );
    this.mastTopParts.push(mastTop, mastBraceA, mastBraceB);
    const bucket = new THREE.Mesh(
      new THREE.CylinderGeometry(ROBOT.bucket.r, 0.108, 0.255, 24, 1, true),
      new THREE.MeshPhongMaterial({ color: 0xdff1ff, transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
    );
    bucket.position.set(bx, bucketTopY - 0.1275, bz);
    this.root.add(bucket);
    const rim = new THREE.Mesh(new THREE.TorusGeometry(ROBOT.bucket.r, 0.008, 8, 24), lam(0xffffff));
    rim.rotation.x = Math.PI / 2;
    rim.position.set(bx, bucketTopY, bz);
    this.root.add(rim);
    const line2L = new THREE.Mesh(new THREE.TorusGeometry(0.12, 0.004, 6, 24), lam(0xd7000f));
    line2L.rotation.x = Math.PI / 2;
    line2L.position.set(bx, bucketTopY - 0.12, bz);
    this.root.add(line2L);
    // 取っ手 (下ろした状態・ルール5.3)
    const handle = new THREE.Mesh(new THREE.TorusGeometry(0.13, 0.006, 6, 20, Math.PI), lam(0xbfc7d1));
    handle.rotation.z = -Math.PI / 2;
    handle.position.set(bx - 0.13, bucketTopY - 0.18, bz);
    this.root.add(handle);
    this.bucketDeployG.add(bucket, rim, line2L, handle);
    const liftHandle = new THREE.Mesh(new THREE.TorusGeometry(0.18, 0.008, 6, 24, Math.PI), lam(0x8f98a8));
    liftHandle.rotation.x = Math.PI / 2;
    liftHandle.position.set(0, 0.43, -0.34);
    this.root.add(liftHandle);

    // ===== LED (駆動電源インジケーター) =====
    this.led = new THREE.Mesh(
      new THREE.BoxGeometry(0.56, 0.025, 0.025),
      new THREE.MeshBasicMaterial({ color: 0x27ff5a }),
    );
    this.led.position.set(0, 0.105, 0.34);
    this.root.add(this.led);

    if (detailed) {
      // ===== 砲塔ヨーク (アルミ柱2本 + 天板) =====
      const yokeL = this.alu(0.02, 0.46, 0.02, -0.15, 0.64, 0.05);
      const yokeR = this.alu(0.02, 0.46, 0.02, 0.15, 0.64, 0.05);
      const yokeTop = this.alu(0.32, 0.02, 0.02, 0, 0.86, 0.05);
      const yokeBraceA = this.aluBetween([-0.15, 0.42, 0.05], [0.15, 0.86, 0.05], 0.014);
      const yokeBraceB = this.aluBetween([0.15, 0.42, 0.05], [-0.15, 0.86, 0.05], 0.014);
      const turretPlate = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.012, 0.3), lam(0x272d38));
      turretPlate.position.set(0, 0.88, 0.05);
      this.root.add(turretPlate);
      this.shooterDeployG.add(yokeL, yokeR, yokeTop, yokeBraceA, yokeBraceB, turretPlate);

      // ===== 旋回砲塔 =====
      const yawG = new THREE.Group();
      yawG.position.set(0, 0.95, 0.05);
      const turretBase = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.18, 0.1, 20), lam(0x3a4250));
      turretBase.castShadow = true;
      yawG.add(turretBase);
      const pitchG = new THREE.Group();
      pitchG.position.set(0, 0.12, 0);
      const housing = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.16, 0.42), lam(0x4a5568));
      housing.position.z = 0.05;
      housing.castShadow = true;
      pitchG.add(housing);
      // 対向ローラー (幅650: スーパー雑巾対応) + M3508ギアボックスレス
      const rollerGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.62, 16);
      const rollerMat = new THREE.MeshPhongMaterial({ color: 0xd57a2a, shininess: 40 });
      const top = new THREE.Mesh(rollerGeo, rollerMat);
      top.rotation.z = Math.PI / 2;
      top.position.set(0, 0.065, 0.3);
      const bot = top.clone();
      bot.position.y = -0.065;
      pitchG.add(top, bot);
      this.shooterRollers = [top, bot];
      for (const sx of [-1, 1] as const) {
        const rm = new THREE.Mesh(new THREE.CylinderGeometry(0.021, 0.021, 0.05, 12), lam(0x15171c));
        rm.rotation.z = Math.PI / 2;
        rm.position.set(sx * 0.34, 0.065, 0.3);
        pitchG.add(rm);
      }
      const guard = new THREE.Mesh(new THREE.BoxGeometry(0.64, 0.02, 0.2), lam(0x333a45));
      guard.position.set(0, 0.13, 0.28);
      pitchG.add(guard);
      const feed = new THREE.Mesh(
        new THREE.PlaneGeometry(0.3, 0.2),
        new THREE.MeshLambertMaterial({ map: ragTexture(false), side: THREE.DoubleSide }),
      );
      feed.rotation.x = -Math.PI / 2;
      feed.visible = false;
      pitchG.add(feed);
      this.feedRag = feed;
      // 照準カメラ: ガードより上・レンズはガード前縁より前
      const cam = new THREE.PerspectiveCamera(48, 16 / 9, 0.05, 40);
      cam.position.set(0, 0.215, 0.31);
      cam.rotation.y = Math.PI;
      pitchG.add(cam);
      (this as { pipCamera: THREE.PerspectiveCamera | null }).pipCamera = cam;
      const camBox = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.05, 0.08), lam(0x20242c));
      camBox.position.set(0, 0.215, 0.25);
      pitchG.add(camBox);
      yawG.add(pitchG);
      this.shooterDeployG.add(yawG);
      this.turretYawG = yawG;
      this.turretPitchG = pitchG;

      // ===== LiDAR (前面下部・スキャン面0.12m) =====
      const lidarMount = this.alu(0.02, 0.02, 0.08, 0, 0.1, 0.36);
      lidarMount.castShadow = false;
      const lidar = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.045, 16), lam(0x111318));
      lidar.position.set(0, 0.135, 0.37);
      this.root.add(lidar);
      const lidarWin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.036, 0.036, 0.014, 16),
        new THREE.MeshBasicMaterial({ color: 0x2a3a55 }),
      );
      lidarWin.position.set(0, 0.13, 0.37);
      this.root.add(lidarWin);

      // ===== 下段LiDAR 後ろ向き2台目: 前後で完全360° → 横付け補充時もフェンス拘束を維持 =====
      const lidarRearMount = this.alu(0.02, 0.02, 0.08, 0, 0.1, -0.36);
      lidarRearMount.castShadow = false;
      const lidarRear = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.045, 16), lam(0x111318));
      lidarRear.position.set(0, 0.135, -0.37);
      this.root.add(lidarRear);
      const lidarRearWin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.036, 0.036, 0.014, 16),
        new THREE.MeshBasicMaterial({ color: 0x2a3a55 }),
      );
      lidarRearWin.position.set(0, 0.13, -0.37);
      this.root.add(lidarRearWin);

      // ===== 上段LiDAR (相手検出用・スキャン面0.5m): 下段は教壇に遮られ相手が見えない =====
      this.alu(0.02, 0.1, 0.02, 0.2, 0.45, 0.3);
      const lidarUp = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.045, 16), lam(0x111318));
      lidarUp.position.set(0.2, 0.515, 0.3);
      this.root.add(lidarUp);
      const lidarUpWin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.036, 0.036, 0.014, 16),
        new THREE.MeshBasicMaterial({ color: 0x553a2a }),
      );
      lidarUpWin.position.set(0.2, 0.505, 0.3);
      this.root.add(lidarUpWin);

      // ===== スタックグラバー (車体 前面: 補充机へ正対approachして山を回収) =====
      // 機体は補充机に正対(前方=+z=攻撃方位のまま)して山を掴むため、フォークは前方(+z)へ伸びる。
      this.alu(0.02, 0.02, 0.26, -0.12, 0.75, 0.28);
      this.alu(0.02, 0.02, 0.26, 0.12, 0.75, 0.28);
      const grab = new THREE.Group();
      grab.position.set(0, 0.78, 0.18); // 前面。フォークは +z へ伸長
      const forkMat = lam(0xb8bec8);
      for (const fx of [-0.09, -0.03, 0.03, 0.09]) {
        const tine = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.008, 0.34), forkMat);
        tine.position.set(fx, 0, 0.17);
        grab.add(tine);
      }
      const presser = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.012, 0.26), lam(0x6b7280));
      presser.position.set(0, 0.12, 0.18);
      presser.name = 'presser';
      grab.add(presser);
      const bundle = new THREE.Mesh(
        new THREE.BoxGeometry(0.28, 0.055, 0.2),
        new THREE.MeshLambertMaterial({ map: ragTexture(false) }),
      );
      bundle.position.set(0, 0.04, 0.18);
      bundle.visible = false;
      grab.add(bundle);
      this.grabRag = bundle;
      this.root.add(grab);
      this.grabber = grab;
    } else {
      // 相手ロボット: スワーブ駆動 + ベルト式直動投射機構
      this.frameBox(0.22, 0.43, 0.82);
      const beltBase = new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.035, 0.78), lam(0x34303a));
      beltBase.position.set(0, 0.84, 0.08);
      beltBase.rotation.x = -0.18;
      beltBase.castShadow = true;
      this.root.add(beltBase);
      for (const sx of [-1, 1] as const) {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.04, 0.82), ALU);
        rail.position.set(sx * 0.19, 0.885, 0.08);
        rail.rotation.x = -0.18;
        this.root.add(rail);
      }
      const beltMat = new THREE.MeshLambertMaterial({ color: 0x111318 });
      for (const sx of [-1, 1] as const) {
        const belt = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.012, 0.72), beltMat);
        belt.position.set(sx * 0.075, 0.91, 0.1);
        belt.rotation.x = -0.18;
        this.root.add(belt);
        for (const z of [-0.26, 0.46]) {
          const pulley = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.13, 14), lam(0x7f8794));
          pulley.rotation.z = Math.PI / 2;
          pulley.rotation.y = 0.18;
          pulley.position.set(sx * 0.075, 0.91 + z * Math.sin(0.18), z);
          this.root.add(pulley);
        }
      }
      const pusher = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.055, 0.035), lam(teamCol));
      pusher.position.set(0, 0.935, -0.14);
      pusher.rotation.x = -0.18;
      this.root.add(pusher);
      const loadedRag = new THREE.Mesh(
        new THREE.PlaneGeometry(0.28, 0.18),
        new THREE.MeshLambertMaterial({ map: ragTexture(false), side: THREE.DoubleSide }),
      );
      loadedRag.position.set(0, 0.96, 0.08);
      loadedRag.rotation.x = -Math.PI / 2 - 0.18;
      this.root.add(loadedRag);
      const linearGuide = new THREE.Mesh(new THREE.BoxGeometry(0.38, 0.018, 0.018), ALU);
      linearGuide.position.set(0, 1.02, 0.48);
      this.root.add(linearGuide);
    }
    this.applyDeployment(0);
  }

  /** 足回りの見た目を独ステ/メカナムへ切り替える (途中変更に即応) */
  private setDrivetrain(swerve: boolean): void {
    if (this.curSwerve === swerve) return;
    this.curSwerve = swerve;
    this.driveMecanum.visible = !swerve;
    this.driveSwerve.visible = swerve;
    this.wheels = swerve ? this.swerveWheels : this.mecanumWheels;
  }

  update(r: RobotState, dt: number, matchT = 999): void {
    this.root.position.set(r.x, r.liftY, r.z);
    this.root.rotation.y = r.theta;
    this.applyDeployment(matchT);
    this.setDrivetrain(r.swerve);

    for (let i = 0; i < 4; i++) {
      const w = this.wheels[i];
      if (!w) continue;
      if (w.swerve && w.steer) {
        const c = Math.cos(r.theta);
        const s = Math.sin(r.theta);
        const lx = r.vx * c - r.vz * s;
        const lz = r.vx * s + r.vz * c;
        const wx = lx + r.omega * w.z;
        const wz = lz - r.omega * w.x;
        const speed = Math.hypot(wx, wz);
        if (speed > 0.03) {
          let target = Math.atan2(wx, wz);
          let diff = wrapAngle(target - w.steer.rotation.y);
          w.spinDir = 1;
          if (Math.abs(diff) > Math.PI / 2) {
            target = wrapAngle(target + Math.PI);
            diff = wrapAngle(target - w.steer.rotation.y);
            w.spinDir = -1;
          }
          w.steer.rotation.y += diff * Math.min(1, dt * 16);
        } else {
          // 静止時ロック: 中心向き(X字)から90°回した接線方向 (ひし形の各辺に沿う向き)。
          // 各モジュールが円の接線を向く=回転にも並進にも抗して静止精度を上げる
          let target = Math.atan2(-w.x, -w.z) + Math.PI / 2;
          let diff = wrapAngle(target - w.steer.rotation.y);
          if (Math.abs(diff) > Math.PI / 2) {
            target = wrapAngle(target + Math.PI);
            diff = wrapAngle(target - w.steer.rotation.y);
          }
          w.steer.rotation.y += diff * Math.min(1, dt * 8);
        }
        const spin = (speed / 0.052) * (w.spinDir ?? 1);
        w.hub.rotation.x -= spin * dt;
        continue;
      }
      w.hub.rotation.y = -(r.anim.wheels[i] ?? 0);
      const spin = r.anim.strafe * 30 * dt;
      for (const rl of w.rollers) rl.rotation.y += spin;
    }

    // 計測輪の回転 (コの字3輪: 前後2輪は差動でヨー成分も乗る)。メカナム表示のときだけ
    if ((this.measureL || this.measureS) && !r.swerve) {
      const vf = r.vx * Math.sin(r.theta) + r.vz * Math.cos(r.theta);
      const vs = r.anim.strafe;
      if (this.measureL) this.measureL.rotation.y -= ((vf + r.omega * 0.15) / 0.024) * dt;
      if (this.measureR) this.measureR.rotation.y -= ((vf - r.omega * 0.15) / 0.024) * dt;
      if (this.measureS) this.measureS.rotation.y -= ((vs - r.omega * 0.15) / 0.024) * dt;
    }

    (this.led.material as THREE.MeshBasicMaterial).color.set(r.powered ? 0x27ff5a : 0x30363f);

    if (this.turretYawG && this.turretPitchG) {
      this.turretYawG.rotation.y = r.turretYaw;
      this.turretPitchG.rotation.x = -r.turretPitch;
      this.rollerAngle += (r.rollerRpm / 60) * Math.PI * 2 * dt;
      const [top, bot] = this.shooterRollers;
      if (top && bot) {
        top.rotation.x = this.rollerAngle;
        bot.rotation.x = -this.rollerAngle;
      }
      if (this.feedRag) {
        if (r.anim.feed >= 0) {
          this.feedRag.visible = true;
          this.feedRag.position.set(0, 0.0, -0.25 + r.anim.feed * 0.55);
        } else {
          this.feedRag.visible = false;
        }
      }
    }

    if (this.grabber) {
      const p = r.anim.grab;
      const presser = this.grabber.getObjectByName('presser');
      if (p >= 0) {
        // 0-0.3 前方(+z)展開+下降 / 0.3-0.5 押さえ / 0.5-0.75 リフト / 0.75-1 引込
        const ext = p < 0.3 ? p / 0.3 : p < 0.75 ? 1 : 1 - (p - 0.75) / 0.25;
        const down = p < 0.3 ? p / 0.3 : p < 0.5 ? 1 : p < 0.75 ? 1 - (p - 0.5) / 0.25 : 0;
        const press = p < 0.3 ? 0 : p < 0.5 ? (p - 0.3) / 0.2 : 1;
        this.grabber.position.z = 0.18 + ext * 0.34;
        this.grabber.position.y = 0.78 + 0.06 - down * 0.06;
        if (presser) presser.position.y = 0.12 - press * 0.095;
        if (this.grabRag) this.grabRag.visible = p > 0.45;
      } else {
        this.grabber.position.set(0, 0.84, 0.18);
        if (presser) presser.position.y = 0.12;
        if (this.grabRag) this.grabRag.visible = false;
      }
    }
  }
}
