import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { MatchSim } from '../sim/match';
import { FIELD, RED_SIDE, homePos, waypoints, type TeamId } from '../config/field';
import {
  LIDAR_SCAN_HEIGHT,
  LIDAR_UPPER_HEIGHT,
  localToWorld,
  scanToLocalPoints,
} from '../sim/lidar';
import { heightProfile } from '../sim/ballistics';
import type { HungRag, Projectile } from '../sim/types';
import { buildField, type FieldRefs } from './field-mesh';
import { FaceCam } from './face-cam';
import { planPath } from '../sim/pathfind';
import { RobotVisual } from './robot-mesh';
import { ragTexture } from './textures';
import type { AudioEngine } from './audio';

export interface HudSnapshot {
  t: number;
  remain: number;
  over: boolean;
  score: { blue: number; red: number };
  breakdown: MatchSim['breakdown'];
  status: { blue: string; red: string };
  events: Array<{ t: number; text: string; cls: string }>;
  metrics: ReturnType<MatchSim['metrics']>;
  manualPower: number;
  fireReady: boolean;
  /** 各ロボットが保持している雑巾枚数 (スコアパネル表示用) */
  held: { blue: number; blueSuper: number; red: number; redSuper: number };
}

export interface PerfSnapshot {
  fps: number;
  frameMs: number;
  simMs: number;
  visualMs: number;
  renderMs: number;
  maxFrameMs: number;
  budgetPct: number;
  headroomMs: number;
  drawCalls: number;
  triangles: number;
  points: number;
}

export interface ViewerElems {
  container: HTMLElement;
  pipBox: HTMLElement;
  pipOverlay: HTMLCanvasElement;
}

interface FieldPlayerVisual {
  team: TeamId;
  slot: number;
  root: THREE.Group;
  body: THREE.Mesh;
  head: THREE.Mesh;
  armL: THREE.Mesh;
  armR: THREE.Mesh;
  legL: THREE.Mesh;
  legR: THREE.Mesh;
  rag: THREE.Mesh;
  phone: THREE.Mesh | null;
  pos: THREE.Vector3;
  /** リトライ接近時に競技物品を避けて歩くA*経路 (と現在の追従インデックス) */
  walkPath?: { x: number; z: number }[];
  walkIdx?: number;
}

/** WebGL コンテキストを1つも張れなかった (GPUプロセス側の問題)。呼び出し側でリトライする。 */
export class WebGLInitError extends Error {
  constructor(cause: unknown) {
    super(`WebGL コンテキストを生成できませんでした: ${String(cause)}`);
    this.name = 'WebGLInitError';
  }
}

/**
 * WebGLRenderer を生成する。
 *
 * Chrome の GPU プロセスがまだ起動しきっていない/再起動中だと、リロード直後に
 * "Could not create a WebGL context ... BindToCurrentSequence failed" で生成が失敗することがある
 * (再現性は低いがゼロではない)。一度コンテキスト生成に失敗した canvas は以後も使えないので、
 * 試行ごとに新しい canvas を用意し、2回目は高性能GPU要求とMSAAを外した控えめな設定で試す。
 */
function createRenderer(): THREE.WebGLRenderer {
  const attempts: THREE.WebGLRendererParameters[] = [
    { antialias: true, powerPreference: 'high-performance' },
    { antialias: false, powerPreference: 'default' },
  ];
  let last: unknown = null;
  for (const opts of attempts) {
    try {
      return new THREE.WebGLRenderer({ ...opts, canvas: document.createElement('canvas') });
    } catch (e) {
      last = e;
    }
  }
  throw new WebGLInitError(last);
}

interface RefereeVisual {
  root: THREE.Group;
  arm: THREE.Group;
  flag: THREE.Mesh;
  /** 主審のみ: 左腕 (青旗) */
  arm2?: THREE.Group;
  timer: number;
  chief?: boolean;
}

export class SimViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private pipCamera = new THREE.PerspectiveCamera(55, 16 / 9, 0.05, 40);
  private controls: OrbitControls;
  private field: FieldRefs;
  private blueVis: RobotVisual;
  private redVis: RobotVisual;
  // バケツ高さは構築時にメッシュへ焼き込まれるため、変更を検知して描画を作り直す
  private builtBucketTopY = { blue: 0, red: 0 };
  private projMeshes = new Map<number, THREE.Mesh>();
  private hungMeshes = new Map<number, THREE.Object3D>();
  private hungCount = 0;
  private hungGroup = new THREE.Group();
  private lidarPts: THREE.Points;
  private lidarPos: THREE.BufferAttribute;
  private upperPts!: THREE.Points;
  private upperPos!: THREE.BufferAttribute;
  private pathMesh!: THREE.Mesh;
  private pathPos!: THREE.BufferAttribute;
  private pathCol!: THREE.BufferAttribute;
  private fieldPlayers: FieldPlayerVisual[] = [];
  private referees: RefereeVisual[] = [];
  /** 放送: Scorpio カメラクレーン */
  private crane!: {
    pan: THREE.Group;
    boom: THREE.Group;
    head: THREE.Group;
    base: THREE.Vector3;
    arm2: THREE.Mesh;
  };
  private craneExt = 0.4; // ブーム伸縮量 (0=縮/1=伸)
  /** 放送: 肩担ぎカメラマン */
  private cameramen: Array<{
    root: THREE.Group;
    aim: THREE.Group;
    home: THREE.Vector3;
    pos: THREE.Vector3;
    phase: number;
    cam: THREE.PerspectiveCamera;
  }> = [];
  /** マルチカム分割表示モード (操縦なし・全カメラ映像を同時表示) */
  multiCamActive = false;
  private refEventCursor = 0;
  private elems: ViewerElems;
  private ragTexN: THREE.Texture;
  private ragTexS: THREE.Texture;
  private clockT = 0;
  private cameraMode: 'broadcast' | 'onboard' | 'top' = 'broadcast';
  private raf = 0;
  private lastNow = 0;
  // 推定可視化 (T8/T21): ICP推定ポーズ矢印 + 不確かさリング + 相手ゴースト
  private estArrow!: THREE.Group;
  private estRing!: THREE.Mesh;
  private ghost!: THREE.Group;
  private fpsCam = new THREE.PerspectiveCamera(72, 16 / 9, 0.05, 60);
  fpsActive = false;
  /** TPS操縦: 追従カメラ + カーソル狙い点射出 */
  tpsActive = false;
  /** 選手操作モード: 選手を歩かせる (f=前後, s=左右) */
  playerWalkActive = false;
  private playerWalk = { f: 0, s: 0, jump: false };
  private playerBody = { vx: 0, vz: 0, y: 0, vy: 0 };
  private playerCam = { yaw: Math.PI, pitch: 0.42, dist: 5.2 };
  /** クレーン操作モード: 先端カメラのPOVで操縦 */
  craneControlActive = false;
  private craneInput = { pan: 0, lift: 0 };
  private craneHead = { yaw: 0, pitch: 0.42, fov: 42 };
  private craneCam = new THREE.PerspectiveCamera(42, 16 / 9, 0.05, 200);
  // 6台目: 観客席上部からフィールド全体を捉える俯瞰カメラ (固定ワイド)
  private overheadCam = new THREE.PerspectiveCamera(52, 16 / 9, 0.1, 300);
  // 投擲雑巾視点: 発射直前に赤/青どちらかへ憑依し、飛ぶ雑巾をTPSで追う (操作なし)
  ragCamActive = false;
  private ragCam = new THREE.PerspectiveCamera(58, 16 / 9, 0.03, 200);
  private ragCamFollowId: number | null = null;
  private ragCamPos = new THREE.Vector3();
  private ragCamLook = new THREE.Vector3();
  private ragCamLastRag = new THREE.Vector3();
  private ragCamLinger = 0;
  private ragCamInit = false;
  // 選手モードの静的当たり判定 (フィールド物品+放送機材)。{x,z,半径,越えられる高さ}
  private playerObstacles: Array<{ x: number; z: number; r: number; h: number }> | null = null;
  // 自分の顔モード: インカメの顔を操縦者(slot1)の顔にマッピング
  faceCamActive = false;
  private faceCam: FaceCam | null = null;
  private faceRigs: THREE.Group[] = [];
  private faceMat: THREE.MeshBasicMaterial | null = null;
  private facePreviewEl: HTMLElement | null = null;
  private tpsCursor = { x: 0, y: 0, has: false };
  private raycaster = new THREE.Raycaster();
  private aimRing!: THREE.Mesh;
  private aimLine!: THREE.Line;
  private aimPos!: THREE.BufferAttribute;
  // Q17: 俯瞰は平行投影
  private topCam = new THREE.OrthographicCamera(-8, 8, 8, -8, 0.1, 60);
  private topHalf = 7.6;
  /** 俯瞰(平行投影)のパン(平行移動)オフセット。ドラッグで移動 */
  private topPan = { x: 0, z: 0 };
  // Q24: 真値ポーズの半透明ゴースト (描画は推定ポーズ主体)
  private trueGhost!: THREE.LineSegments;
  // Q21: 勝利演出
  private confetti!: THREE.InstancedMesh;
  private confettiVel: Float32Array | null = null;
  private confettiRot: Float32Array | null = null;
  private confettiT = -1;
  private celebrated = false;
  // 自動リピート: 競技終了から一定秒後にリセット再開する (App が handleReset を渡す)
  onAutoRepeat: (() => void) | null = null;
  private repeatT = -1;
  teamNames = { blue: '青チーム', red: '赤チーム' };
  private tvTimer = 0;

  match: MatchSim;
  playing = false;
  speed = 1;
  showLidar = true;
  showCam = true;
  showPath = true;
  audio: AudioEngine | null = null;
  onHud: ((s: HudSnapshot) => void) | null = null;
  onPerf: ((s: PerfSnapshot) => void) | null = null;
  onToast: ((msg: string, ok: boolean) => void) | null = null;
  private faceCamNotified = false;
  private hudTimer = 0;
  private perfWindow = 0;
  private perfFrames = 0;
  private perfFrameMs = 0;
  private perfSimMs = 0;
  private perfVisualMs = 0;
  private perfRenderMs = 0;
  private perfMaxFrameMs = 0;

  constructor(elems: ViewerElems, match: MatchSim) {
    this.elems = elems;
    this.match = match;
    const w = elems.container.clientWidth;
    const h = elems.container.clientHeight;
    this.renderer = createRenderer();
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(w, h);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // 会場照明のハイライトを飛ばさず暗部も潰さないフィルミックなトーン
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.18;
    elems.container.appendChild(this.renderer.domElement);
    this.renderer.domElement.addEventListener('webglcontextlost', this.onContextLost);
    this.renderer.domElement.addEventListener('webglcontextrestored', this.onContextRestored);

    this.camera = new THREE.PerspectiveCamera(48, w / h, 0.1, 200);
    this.camera.position.set(-5.8, 8.0, 8.6);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.4, 0);
    this.controls.maxPolarAngle = Math.PI / 2 - 0.03;
    this.controls.enableDamping = true;

    this.scene.background = new THREE.Color(0x14161d);
    this.scene.fog = new THREE.Fog(0x14161d, 34, 70);
    // Q29: 国技館風 — 会場は薄暗く、フィールドへ真上+四隅斜め上から強い光を集中させる
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.34));
    this.scene.add(new THREE.HemisphereLight(0xfff6e6, 0x2a2d38, 0.72));
    // 主光源: 吊り照明の真上トップライト (影の主源)。
    // 注意: intensity=0 だと影が完全に消える (過去に消灯したままになっていた)
    const top = new THREE.SpotLight(0xfff3dc, 150, 100, 0.66, 0.45, 1.6);
    top.position.set(0, 16, 0);
    top.target.position.set(0, 0, 0);
    top.castShadow = true;
    top.shadow.mapSize.set(2048, 2048);
    top.shadow.bias = -0.0004;
    this.scene.add(top, top.target);
    // 四隅の照明タワーからの斜めスポット (影なし、明るさの底上げ)
    for (const [sx, sz] of [
      [9, 9],
      [-9, 9],
      [9, -9],
      [-9, -9],
    ] as const) {
      const spot = new THREE.SpotLight(0xffedd2, 40, 45, 0.5, 0.55, 1.7);
      spot.position.set(sx, 13, sz);
      spot.target.position.set(0, 0, 0);
      this.scene.add(spot, spot.target);
    }
    // 輪郭用の弱い指向光 (旧sunの名残 — 影は持たせない)
    const sun = new THREE.DirectionalLight(0xfff4e0, 0.35);
    sun.position.set(9, 15, 7);
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xcfe0ff, 0.3);
    fill.position.set(-8, 10, -9);
    this.scene.add(fill);

    this.field = buildField();
    this.scene.add(this.field.group);
    this.scene.add(this.hungGroup);

    this.blueVis = new RobotVisual('blue', match.par.bucketTopY.blue, true);
    this.redVis = new RobotVisual('red', match.par.bucketTopY.red, true);
    this.scene.add(this.blueVis.root, this.redVis.root);
    this.builtBucketTopY = { blue: match.par.bucketTopY.blue, red: match.par.bucketTopY.red };
    for (const team of ['blue', 'red'] as const) {
      for (let slot = 0; slot < 3; slot++) this.fieldPlayers.push(this.makeFieldPlayer(team, slot));
    }
    this.scene.add(...this.fieldPlayers.map((p) => p.root));
    this.referees = [
      this.makeReferee(FIELD.w / 2 + 0.95, 0, 0xd7000f, true), // 主審: 看板前・両手に赤青旗
      this.makeReferee(0, FIELD.l / 2 + 0.62, 0x1f5fb0), // 青副審 (青チーム側)
      this.makeReferee(0, -FIELD.l / 2 - 0.62, 0xb52731), // 赤副審 (赤チーム側)
    ];
    this.scene.add(...this.referees.map((r) => r.root));
    this.buildBroadcast();

    const MAXP = 620;
    this.lidarPos = new THREE.BufferAttribute(new Float32Array(MAXP * 3), 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', this.lidarPos);
    geo.setDrawRange(0, 0);
    this.lidarPts = new THREE.Points(
      geo,
      new THREE.PointsMaterial({ color: 0x35ff7a, size: 0.07, sizeAttenuation: true }),
    );
    this.lidarPts.frustumCulled = false;
    this.scene.add(this.lidarPts);

    // 上段LiDAR (相手検出用 0.5m) の点群 — 橙色
    this.upperPos = new THREE.BufferAttribute(new Float32Array(MAXP * 3), 3);
    const ugeo = new THREE.BufferGeometry();
    ugeo.setAttribute('position', this.upperPos);
    ugeo.setDrawRange(0, 0);
    this.upperPts = new THREE.Points(
      ugeo,
      new THREE.PointsMaterial({ color: 0xffa03e, size: 0.075, sizeAttenuation: true }),
    );
    this.upperPts.frustumCulled = false;
    this.scene.add(this.upperPts);

    // A* 経路の可視化 (操作チーム)。目標速度で色分け (速い=緑 / 遅い=赤)
    this.pathPos = new THREE.BufferAttribute(new Float32Array(63 * 6 * 3), 3);
    this.pathCol = new THREE.BufferAttribute(new Float32Array(63 * 6 * 3), 3);
    const pgeo = new THREE.BufferGeometry();
    pgeo.setAttribute('position', this.pathPos);
    pgeo.setAttribute('color', this.pathCol);
    pgeo.setDrawRange(0, 0);
    this.pathMesh = new THREE.Mesh(
      pgeo,
      new THREE.MeshBasicMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.92,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    this.pathMesh.frustumCulled = false;
    this.scene.add(this.pathMesh);

    this.ragTexN = ragTexture(false);
    this.ragTexS = ragTexture(true);

    // ICP推定ポーズの矢印 (実推定値を3Dフィールドに直接表示)
    this.estArrow = new THREE.Group();
    const arrowMat = new THREE.MeshBasicMaterial({ color: 0x4da3ff, transparent: true, opacity: 0.9 });
    const shaft = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.012, 0.5), arrowMat);
    shaft.position.set(0, 0, 0.05);
    const head = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.2, 12), arrowMat);
    head.rotation.x = Math.PI / 2;
    head.position.set(0, 0, 0.38);
    this.estArrow.add(shaft, head);
    this.estArrow.position.y = 0.04;
    this.scene.add(this.estArrow);
    this.estRing = new THREE.Mesh(
      new THREE.RingGeometry(0.86, 1, 40),
      new THREE.MeshBasicMaterial({ color: 0x4da3ff, transparent: true, opacity: 0.4, side: THREE.DoubleSide }),
    );
    this.estRing.rotation.x = -Math.PI / 2;
    this.estRing.position.y = 0.035;
    this.scene.add(this.estRing);

    // 相手ロボットのゴースト (LiDARクラスタ推定の実出力)
    this.ghost = new THREE.Group();
    const gMat = new THREE.MeshBasicMaterial({
      color: 0xff5566,
      transparent: true,
      opacity: 0.22,
      depthWrite: false,
    });
    const gBox = new THREE.Mesh(new THREE.BoxGeometry(0.76, 1.0, 0.76), gMat);
    gBox.position.y = 0.5;
    const gEdge = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.76, 1.0, 0.76)),
      new THREE.LineBasicMaterial({ color: 0xff7788, transparent: true, opacity: 0.8 }),
    );
    gEdge.position.y = 0.5;
    const gBucket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.14, 0.11, 0.26, 16, 1, true),
      new THREE.MeshBasicMaterial({ color: 0xff8899, transparent: true, opacity: 0.3, side: THREE.DoubleSide }),
    );
    gBucket.position.y = 1.45;
    this.ghost.add(gBox, gEdge, gBucket);
    this.ghost.visible = false;
    this.scene.add(this.ghost);

    // TPS照準マーカー + 予測軌道
    this.aimRing = new THREE.Mesh(
      new THREE.RingGeometry(0.16, 0.22, 28),
      new THREE.MeshBasicMaterial({ color: 0x7dffa5, transparent: true, opacity: 0.85, side: THREE.DoubleSide }),
    );
    this.aimRing.rotation.x = -Math.PI / 2;
    this.aimRing.visible = false;
    this.scene.add(this.aimRing);
    this.aimPos = new THREE.BufferAttribute(new Float32Array(40 * 3), 3);
    const ageo = new THREE.BufferGeometry();
    ageo.setAttribute('position', this.aimPos);
    this.aimLine = new THREE.Line(
      ageo,
      new THREE.LineBasicMaterial({ color: 0x7dffa5, transparent: true, opacity: 0.55 }),
    );
    this.aimLine.frustumCulled = false;
    this.aimLine.visible = false;
    this.scene.add(this.aimLine);

    // Q24: 真値ポーズの半透明ワイヤ枠
    this.trueGhost = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.72, 1.05, 0.72)),
      new THREE.LineBasicMaterial({ color: 0x9fd0ff, transparent: true, opacity: 0.35 }),
    );
    this.trueGhost.position.y = 0.55;
    this.scene.add(this.trueGhost);

    // Q21: 紙吹雪 (キラテープ)
    const cGeo = new THREE.PlaneGeometry(0.14, 0.045);
    const cMat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, vertexColors: false });
    this.confetti = new THREE.InstancedMesh(cGeo, cMat, 420);
    this.confetti.visible = false;
    this.scene.add(this.confetti);

    addEventListener('resize', this.onResize);
    this.lastNow = performance.now();
    this.loop(this.lastNow);
  }

  /** TPS: 画面座標のカーソル位置を保存 (毎フレーム、シーンにレイキャストして狙い点を更新) */
  setTpsCursor(clientX: number, clientY: number): void {
    const w = this.elems.container.clientWidth;
    const h = this.elems.container.clientHeight;
    this.tpsCursor = { x: (clientX / w) * 2 - 1, y: -(clientY / h) * 2 + 1, has: true };
  }

  tpsFire(): { ok: boolean; reason?: string } {
    return this.match.manualFire();
  }

  private updateTpsAim(): void {
    if (!this.tpsActive || !this.tpsCursor.has) {
      this.aimRing.visible = false;
      this.aimLine.visible = false;
      if (!this.tpsActive) this.match.manual.target = null;
      return;
    }
    this.raycaster.setFromCamera(
      new THREE.Vector2(this.tpsCursor.x, this.tpsCursor.y),
      this.camera,
    );
    // 狙える対象: フィールド造作物 + 相手ロボット + 着地済み雑巾。マーカー類は除外
    const opponentVis = this.match.playerTeam === 'blue' ? this.redVis.root : this.blueVis.root;
    const hits = this.raycaster.intersectObjects(
      [this.field.group, opponentVis, this.hungGroup],
      true,
    );
    // フィールド外 (バックドロップ・会場床) は狙い対象外
    const hit = hits.find(
      (h) => h.object.visible && Math.abs(h.point.x) <= 5.4 && Math.abs(h.point.z) <= 6.0,
    );
    if (!hit) {
      this.aimRing.visible = false;
      this.aimLine.visible = false;
      this.match.manual.target = null;
      return;
    }
    const p = hit.point;
    this.match.manual.target = { x: p.x, y: Math.max(0.02, p.y), z: p.z };
    // マーカー
    const ok = this.match.manual.sol.ok;
    this.aimRing.visible = true;
    this.aimRing.position.set(p.x, Math.max(0.03, p.y) + 0.02, p.z);
    (this.aimRing.material as THREE.MeshBasicMaterial).color.set(ok ? 0x7dffa5 : 0xff6a6a);
    // 予測軌道 (現在の弾道解を2D積分して描く)
    const b = this.match.player;
    if (ok) {
      const sol = this.match.manual.sol;
      const dx = p.x - b.x;
      const dz = p.z - b.z;
      const dHor = Math.max(0.1, Math.hypot(dx, dz));
      const ux = dx / dHor;
      const uz = dz / dHor;
      const prof = heightProfile(sol.speed, sol.angle, this.match.par.rag, dHor, dHor / 30);
      const n = Math.min(prof.length, 40);
      for (let i = 0; i < 40; i++) {
        const j = Math.min(i, n - 1);
        const s = (j / 30) * dHor;
        this.aimPos.setXYZ(i, b.x + ux * (0.4 + s), b.muzzleY + prof[j]!, b.z + uz * (0.4 + s));
      }
      this.aimPos.needsUpdate = true;
      this.aimLine.geometry.setDrawRange(0, n);
      this.aimLine.visible = true;
    } else {
      this.aimLine.visible = false;
    }
  }

  private updateTpsCam(dt: number): void {
    const r = this.match.player;
    const fx = Math.sin(r.theta);
    const fz = Math.cos(r.theta);
    const k = Math.min(1, dt * 6);
    const desired = new THREE.Vector3(r.x - fx * 3.1, 2.35, r.z - fz * 3.1);
    this.camera.position.lerp(desired, k);
    this.controls.target.lerp(new THREE.Vector3(r.x + fx * 3.2, 0.9, r.z + fz * 3.2), k);
  }

  // GPUプロセスが落ちる/リセットされるとコンテキストが失われる。preventDefault しておくと
  // ブラウザは復帰時に webglcontextrestored を発火し、three が GL 資源を作り直してくれる。
  // preventDefault しないと二度と復帰せず、黒画面のまま固まる。
  private onContextLost = (e: Event): void => {
    e.preventDefault();
    cancelAnimationFrame(this.raf);
    this.raf = 0;
    this.onToast?.('GPUコンテキストを失いました。復帰を待っています…', false);
  };

  private onContextRestored = (): void => {
    this.lastNow = performance.now();
    this.onToast?.('GPUコンテキストが復帰しました', true);
    if (!this.raf) this.loop(this.lastNow);
  };

  dispose(): void {
    cancelAnimationFrame(this.raf);
    this.raf = 0;
    removeEventListener('resize', this.onResize);
    this.faceCam?.stop();
    const canvas = this.renderer.domElement;
    canvas.removeEventListener('webglcontextlost', this.onContextLost);
    canvas.removeEventListener('webglcontextrestored', this.onContextRestored);
    this.renderer.dispose();
    // コンテキストを明示的に解放する。放置すると (HMR やページ遷移の連続で) ブラウザの
    // 同時コンテキスト数上限に当たり、次の生成が失敗する原因になる。
    this.renderer.forceContextLoss();
    canvas.remove();
  }

  private onResize = (): void => {
    const w = this.elems.container.clientWidth;
    const h = this.elems.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  resetMatch(match: MatchSim): void {
    this.match = match;
    this.playing = false;
    for (const m of this.projMeshes.values()) this.scene.remove(m);
    this.projMeshes.clear();
    this.hungMeshes.clear();
    this.hungGroup.clear();
    this.hungCount = 0;
    this.ghost.visible = false;
    for (const p of this.fieldPlayers) p.pos.copy(this.fieldPlayerTarget(p, 0));
    this.refEventCursor = 0;
    this.celebrated = false;
    this.repeatT = -1;
    this.confetti.visible = false;
    this.confettiT = -1;
    this.pushHud(true);
  }

  cameraPreset(which: 'broadcast' | 'onboard' | 'top'): void {
    this.cameraMode = which;
    if (which === 'broadcast') {
      this.camera.position.set(-5.8, 8.0, 8.6);
      this.controls.target.set(0, 0.3, -0.2);
    } else if (which === 'top') {
      this.camera.position.set(0, 17, 0.01);
      this.controls.target.set(0, 0, 0);
      this.topPan.x = 0;
      this.topPan.z = 0;
    } else {
      const r = this.match.player;
      this.camera.position.set(r.x + 1.6, 2.6, r.z + 2.6);
      this.controls.target.set(r.x, 1.2, r.z - 2);
    }
  }

  private loop = (now: number): void => {
    this.raf = requestAnimationFrame(this.loop);
    const frameStart = performance.now();
    const rawDt = (now - this.lastNow) / 1000;
    const dt = Math.min(0.05, rawDt);
    this.lastNow = now;
    this.clockT += dt;
    // Q22: 看板TVは0.5秒ごとに描き換え (Canvas描画コスト削減)
    this.tvTimer -= dt;
    if (this.tvTimer <= 0) {
      this.tvTimer = 0.5;
      this.updateTvScreen();
    }

    let simMs = 0;
    if (this.playing) {
      const simStart = performance.now();
      let sim = dt * this.speed;
      while (sim > 1e-6) {
        const h = Math.min(1 / 60, sim);
        this.match.step(h);
        sim -= h;
      }
      simMs = performance.now() - simStart;
    }
    this.audio?.update(this.match, dt, this.playing);

    const visualStart = performance.now();
    this.syncVisuals(dt);
    const visualMs = performance.now() - visualStart;
    this.hudTimer -= dt;
    if (this.hudTimer <= 0) {
      this.hudTimer = 0.1;
      this.pushHud(false);
    }

    // 旗の揺らぎ
    this.field.flagCloths.forEach((m, i) => {
      m.rotation.y = Math.sin(this.clockT * 1.1 + i * 2.1) * 0.09;
      m.rotation.z = Math.sin(this.clockT * 1.7 + i) * 0.02;
    });

    this.updateFollowCamera(dt);
    this.updateFaceCam();
    if (this.playerWalkActive) this.updatePlayerCam();
    this.controls.enabled =
      !this.fpsActive &&
      !this.tpsActive &&
      !this.playerWalkActive &&
      !this.craneControlActive &&
      !this.multiCamActive &&
      !this.ragCamActive &&
      this.cameraMode !== 'onboard' &&
      this.cameraMode !== 'top';
    if (this.tpsActive) this.updateTpsCam(dt);
    this.updateTpsAim();
    this.controls.update();
    const renderStart = performance.now();
    this.renderer.setScissorTest(false);
    this.renderer.setViewport(0, 0, this.elems.container.clientWidth, this.elems.container.clientHeight);
    if (this.multiCamActive) {
      this.updateMultiCamCameras();
      this.renderMultiCam();
    } else if (this.ragCamActive) {
      this.updateRagCam(dt);
      this.renderer.render(this.scene, this.ragCam);
    } else if (this.craneControlActive) {
      this.updateCraneCam();
      this.renderer.render(this.scene, this.craneCam);
      this.renderPip();
    } else if (this.fpsActive) {
      this.updateFpsCam();
      this.renderer.render(this.scene, this.fpsCam);
    } else if (this.cameraMode === 'top') {
      this.updateTopCam();
      this.renderer.render(this.scene, this.topCam);
      this.renderPip();
    } else {
      this.renderer.render(this.scene, this.camera);
      this.renderPip();
    }
    const renderMs = performance.now() - renderStart;
    this.recordPerf(rawDt, performance.now() - frameStart, simMs, visualMs, renderMs);
  };

  /** 選手操作: 前後(f)・左右(s) の入力 (-1..1) */
  setPlayerWalk(f: number, s: number): void {
    this.playerWalk.f = f;
    this.playerWalk.s = s;
  }

  triggerPlayerJump(): void {
    this.playerWalk.jump = true;
  }

  /** 選手モードのマウス視点 (ポインタロック時。ドラッグ不要でカメラをオービット) */
  lookPlayer(mx: number, my: number): void {
    this.playerCam.yaw -= mx * 0.0024;
    this.playerCam.pitch = Math.min(1.25, Math.max(0.05, this.playerCam.pitch + my * 0.0024));
  }

  /** クレーン操作: A/D=旋回(pan), W/S=俯仰(lift) の入力 (-1..1) */
  setCraneInput(pan: number, lift: number): void {
    this.craneInput.pan = pan;
    this.craneInput.lift = lift;
  }

  /** クレーン操作: マウスでヘッドのパン/チルト */
  lookCrane(mx: number, my: number): void {
    this.craneHead.yaw -= mx * 0.0022;
    this.craneHead.pitch = Math.min(1.3, Math.max(-0.3, this.craneHead.pitch + my * 0.0022));
  }

  /** クレーン操作: ホイールでズーム (FOV) */
  craneZoom(deltaY: number): void {
    this.craneHead.fov = Math.min(64, Math.max(16, this.craneHead.fov + Math.sign(deltaY) * 3));
  }

  /** クレーン先端カメラを頭部の実ワールド姿勢に合わせる (POV) */
  private updateCraneCam(): void {
    if (!this.crane) return;
    // 直前に更新した pan/boom/head の回転を確実に反映させてから姿勢を読む
    this.crane.head.updateWorldMatrix(true, false);
    const wp = new THREE.Vector3();
    this.crane.head.getWorldPosition(wp);
    const q = new THREE.Quaternion();
    this.crane.head.getWorldQuaternion(q);
    const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
    this.craneCam.position.copy(wp).add(fwd.clone().multiplyScalar(0.35));
    this.craneCam.lookAt(wp.x + fwd.x, wp.y + fwd.y, wp.z + fwd.z);
    const w = this.elems.container.clientWidth;
    const h = this.elems.container.clientHeight;
    this.craneCam.aspect = w / Math.max(1, h);
    this.craneCam.fov = this.craneHead.fov;
    this.craneCam.updateProjectionMatrix();
  }

  /**
   * 投擲雑巾視点 (操作なし)。飛行中の雑巾があれば最新弾に憑依してTPSで追い、
   * 無ければ発射直前 (aim あり・未発射) のロボットのマズル後方に構えて次の投擲を待つ。
   */
  private updateRagCam(dt: number): void {
    const m = this.match;
    const live = m.projectiles.filter((p) => !p.landed);
    // 追従中の弾がまだ生きているか。無ければ最新の弾へ憑依。
    let pr = this.ragCamFollowId != null ? live.find((p) => p.id === this.ragCamFollowId) : undefined;
    if (!pr && live.length) pr = live[live.length - 1];
    const newPossession = pr != null && pr.id !== this.ragCamFollowId;
    if (pr) this.ragCamFollowId = pr.id;
    else if (live.length === 0) this.ragCamFollowId = null;

    const desiredPos = new THREE.Vector3();
    const desiredLook = new THREE.Vector3();
    let fast = false;

    if (pr) {
      // 飛行中: 雑巾の後方やや上から進行方向を見る
      fast = true;
      this.ragCamLinger = 0.7;
      this.ragCamLastRag.set(pr.x, pr.y, pr.z);
      const vel = new THREE.Vector3(pr.vx, pr.vy, pr.vz);
      const sp = vel.length();
      const vhat = sp > 0.2 ? vel.multiplyScalar(1 / sp) : new THREE.Vector3(0, 0, pr.team === 'blue' ? -1 : 1);
      desiredPos.set(pr.x, pr.y, pr.z).addScaledVector(vhat, -2.0);
      desiredPos.y += 0.85;
      desiredLook.set(pr.x, pr.y, pr.z).addScaledVector(vhat, Math.min(3, sp * 0.18));
    } else if (this.ragCamLinger > 0) {
      // 着弾直後の余韻: その場で着地点を見る
      this.ragCamLinger -= dt;
      desiredPos.copy(this.ragCamPos);
      desiredLook.copy(this.ragCamLastRag);
    } else {
      // 発射直前の憑依先を探す (aim あり・未発射)
      const shooter = [m.blue, m.red].find((r) => r.aim && !r.aim.fired);
      if (shooter && shooter.aim) {
        const t = shooter.aim.target;
        const dx = t.x - shooter.x;
        const dz = t.z - shooter.z;
        const d = Math.hypot(dx, dz) || 1;
        const ax = dx / d;
        const az = dz / d;
        desiredPos.set(shooter.x - ax * 1.9, shooter.muzzleY + 0.85, shooter.z - az * 1.9);
        desiredLook.set(t.x, Math.max(0.4, t.y), t.z);
      } else {
        // 待機: 二機の中間をゆったり俯瞰
        const mx = (m.blue.x + m.red.x) / 2;
        const mz = (m.blue.z + m.red.z) / 2;
        desiredPos.set(mx, 3.2, mz + 4.2);
        desiredLook.set(mx, 0.7, mz);
      }
    }

    const snap = newPossession || !this.ragCamInit;
    if (snap) {
      this.ragCamPos.copy(desiredPos);
      this.ragCamLook.copy(desiredLook);
      this.ragCamInit = true;
    } else {
      const k = Math.min(1, dt * (fast ? 14 : 4));
      this.ragCamPos.lerp(desiredPos, k);
      this.ragCamLook.lerp(desiredLook, k);
    }
    this.ragCam.position.copy(this.ragCamPos);
    this.ragCam.up.set(0, 1, 0);
    this.ragCam.lookAt(this.ragCamLook);
    this.ragCam.aspect = this.elems.container.clientWidth / Math.max(1, this.elems.container.clientHeight);
    this.ragCam.fov = fast ? 60 : 48;
    this.ragCam.updateProjectionMatrix();
  }

  private get controlledPlayer(): FieldPlayerVisual | undefined {
    return this.fieldPlayers.find((p) => p.team === this.match.playerTeam && p.slot === 1);
  }

  /** 操作中の選手を物理付きで動かす (カメラ相対移動・重力/ジャンプ・ロボット/範囲との当たり判定) */
  private updateControlledPlayer(cp: FieldPlayerVisual, dt: number): void {
    const spd = 3.4;
    const yaw = this.playerCam.yaw;
    // カメラ相対: 前=画面奥、右=画面右
    const fx = Math.sin(yaw);
    const fz = Math.cos(yaw);
    // 画面右 = forward×up ((-cosθ, sinθ))。ロボット操作と同じ右ベクトル規約に合わせる
    const rx = -Math.cos(yaw);
    const rz = Math.sin(yaw);
    const desX = (fx * this.playerWalk.f + rx * this.playerWalk.s) * spd;
    const desZ = (fz * this.playerWalk.f + rz * this.playerWalk.s) * spd;
    // 即応: 目標速度へ素早く追従 (入力ラグを無くす)
    const k = Math.min(1, dt * 20);
    this.playerBody.vx += (desX - this.playerBody.vx) * k;
    this.playerBody.vz += (desZ - this.playerBody.vz) * k;
    cp.pos.x += this.playerBody.vx * dt;
    cp.pos.z += this.playerBody.vz * dt;
    // ジャンプ + 重力
    if (this.playerWalk.jump && this.playerBody.y <= 0.001) this.playerBody.vy = 4.4;
    this.playerWalk.jump = false;
    this.playerBody.vy -= 13 * dt;
    this.playerBody.y = Math.max(0, this.playerBody.y + this.playerBody.vy * dt);
    if (this.playerBody.y <= 0 && this.playerBody.vy < 0) this.playerBody.vy = 0;
    // ロボットと当たり判定 (押し合う)
    for (const r of [this.match.blue, this.match.red]) {
      const dx = cp.pos.x - r.x;
      const dz = cp.pos.z - r.z;
      const d = Math.hypot(dx, dz);
      const minD = 0.5 + 0.28;
      if (d < minD && d > 1e-3 && this.playerBody.y < 0.6) {
        const push = minD - d;
        const nx = dx / d;
        const nz = dz / d;
        cp.pos.x += nx * push * 0.65;
        cp.pos.z += nz * push * 0.65;
        // ロボットも押される (サンドボックスの遊び)
        if (!r.retry) {
          r.x -= nx * push * 0.35;
          r.z -= nz * push * 0.35;
        }
      }
    }
    // フィールド物品・放送機材・審判との当たり判定 (低い物はジャンプで越えられる)
    if (!this.playerObstacles) this.playerObstacles = this.buildPlayerObstacles();
    const pr = 0.26; // 選手の半径
    const collide = (ox: number, oz: number, orad: number, oh: number): void => {
      if (this.playerBody.y > oh) return; // 高さを越えていればすり抜け (ジャンプ)
      const dx = cp.pos.x - ox;
      const dz = cp.pos.z - oz;
      const d = Math.hypot(dx, dz);
      const minD = orad + pr;
      if (d < minD && d > 1e-3) {
        const push = minD - d;
        cp.pos.x += (dx / d) * push;
        cp.pos.z += (dz / d) * push;
      }
    };
    for (const o of this.playerObstacles) collide(o.x, o.z, o.r, o.h);
    for (const cm of this.cameramen) collide(cm.pos.x, cm.pos.z, 0.32, 1.7); // カメラマン (移動する)
    for (const ref of this.referees) collide(ref.root.position.x, ref.root.position.z, 0.3, 1.8); // 審判
    // 教壇 (中央の低い壁): ジャンプで越えられる高さ以下なら通さない
    if (this.playerBody.y < FIELD.podium.h + 0.16 && Math.abs(cp.pos.x) < FIELD.podium.w / 2) {
      const zEdge = FIELD.podium.d / 2 + pr;
      if (Math.abs(cp.pos.z) < zEdge) cp.pos.z = cp.pos.z >= 0 ? zEdge : -zEdge;
    }
    // 会場の歩ける範囲 (スタンド手前) にクランプ
    cp.pos.x = Math.max(-13.5, Math.min(13.5, cp.pos.x));
    cp.pos.z = Math.max(-13.5, Math.min(13.5, cp.pos.z));
    cp.root.position.set(cp.pos.x, this.playerBody.y, cp.pos.z);
    const mv = Math.hypot(this.playerBody.vx, this.playerBody.vz);
    if (mv > 0.15) cp.root.rotation.y = Math.atan2(this.playerBody.vx, this.playerBody.vz);
    // 歩行 / ジャンプの姿勢
    const airborne = this.playerBody.y > 0.02;
    const g = airborne ? 0.6 : Math.sin(this.clockT * (mv > 0.3 ? 13 : 2));
    cp.legL.rotation.x = g * 0.42;
    cp.legR.rotation.x = -g * 0.42;
    cp.armL.rotation.x = -g * 0.34;
    cp.armR.rotation.x = g * 0.34;
    cp.armL.rotation.z = airborne ? -0.7 : 0;
    cp.armR.rotation.z = airborne ? 0.7 : 0;
    if (cp.phone) cp.phone.visible = false;
  }

  /**
   * 自分の顔モード: 有効なら FaceCam を起動し、全選手の頭の前後両面へインカメの顔を貼る
   * (すぐ後ろを向くため前後2枚)。無効化で停止・片付け。映像は端末内処理のみ。
   */
  private updateFaceCam(): void {
    if (this.faceCamActive) {
      if (!this.faceCam) {
        this.faceCam = new FaceCam();
        this.faceCamNotified = false;
        void this.faceCam.start();
        this.onToast?.('自分の顔モード: カメラを起動中…', true);
      }
      const fc = this.faceCam;
      if (!this.faceCamNotified) {
        if (fc.error) {
          this.onToast?.(`顔モード失敗: ${fc.error}`, false);
          this.faceCamNotified = true;
        } else if (fc.ready) {
          this.onToast?.('顔モード: 全選手の顔に反映中', true);
          this.faceCamNotified = true;
        }
      }
      fc.update(performance.now());
      // 確認用: 切り出した自分の顔を画面隅にプレビュー表示
      if (fc.ready && !this.facePreviewEl) {
        const el = fc.canvas;
        el.id = 'k2026-face-preview';
        el.style.cssText =
          'position:fixed;right:12px;top:172px;width:132px;height:132px;z-index:30;' +
          'border:2px solid #2a3140;border-radius:10px;background:#04060a;' +
          'pointer-events:none;box-shadow:0 4px 14px rgba(0,0,0,0.5);';
        document.body.appendChild(el);
        this.facePreviewEl = el;
      }
      // 全選手の頭に前後2枚の顔を貼る (一度だけ生成)
      if (fc.ready && this.faceRigs.length === 0) {
        this.faceMat = new THREE.MeshBasicMaterial({
          map: fc.texture,
          transparent: true,
          depthWrite: false,
          side: THREE.DoubleSide,
        });
        const geo = new THREE.PlaneGeometry(0.2, 0.23);
        for (const p of this.fieldPlayers) {
          const rig = new THREE.Group();
          const front = new THREE.Mesh(geo, this.faceMat);
          front.position.set(0, 1.06, 0.11);
          const back = new THREE.Mesh(geo, this.faceMat);
          back.position.set(0, 1.06, -0.11);
          back.rotation.y = Math.PI;
          rig.add(front, back);
          rig.renderOrder = 3;
          p.root.add(rig);
          this.faceRigs.push(rig);
        }
      }
    } else if (this.faceCam) {
      this.faceCam.stop();
      this.faceCam = null;
      this.faceCamNotified = false;
      for (const rig of this.faceRigs) rig.parent?.remove(rig);
      this.faceRigs = [];
      this.faceMat?.dispose();
      this.faceMat = null;
      // プレビューは id 指定で確実に消す (旧インスタンス由来の残留も掃除)
      document.getElementById('k2026-face-preview')?.remove();
      if (this.facePreviewEl?.parentElement) this.facePreviewEl.parentElement.removeChild(this.facePreviewEl);
      this.facePreviewEl = null;
    }
  }

  /** ロボットの描画を現在の bucketTopY で作り直す (バケツ高さ変更の反映用) */
  private rebuildRobotVisual(team: TeamId): void {
    const old = team === 'blue' ? this.blueVis : this.redVis;
    this.scene.remove(old.root);
    old.root.traverse((o) => {
      if (o instanceof THREE.Mesh) {
        o.geometry.dispose();
        const mat = o.material as THREE.Material | THREE.Material[];
        if (Array.isArray(mat)) mat.forEach((mm) => mm.dispose());
        else mat.dispose();
      }
    });
    const fresh = new RobotVisual(team, this.match.par.bucketTopY[team], true);
    this.scene.add(fresh.root);
    if (team === 'blue') this.blueVis = fresh;
    else this.redVis = fresh;
  }

  /** 選手モードの静的当たり判定リストを構築 (フィールド物品 両陣 + Scorpioクレーン土台) */
  private buildPlayerObstacles(): Array<{ x: number; z: number; r: number; h: number }> {
    const list: Array<{ x: number; z: number; r: number; h: number }> = [];
    // [キー, 半径, 越えるのに要る高さ]。机・椅子は低くジャンプで越えられる
    const props: Array<[keyof typeof RED_SIDE, number, number]> = [
      ['desk1', 0.45, 0.82],
      ['desk2', 0.45, 0.82],
      ['b1', 0.34, 2.0],
      ['b2', 0.34, 2.0],
      ['b3', 0.34, 2.0],
      ['flag', 0.3, 3.2],
      ['chair', 0.3, 0.62],
      ['control', 0.4, 1.3],
    ];
    for (const [key, r, h] of props) {
      const p = RED_SIDE[key];
      list.push({ x: p.x, z: p.z, r, h }); // 赤陣 (z<0)
      list.push({ x: p.x, z: -p.z, r, h }); // 青陣 (z>0)
    }
    // Scorpio クレーンの土台 (buildBroadcast と同じ位置)
    list.push({ x: -(FIELD.w / 2 + 2.8), z: FIELD.l / 2 + 2.4, r: 0.95, h: 3 });
    return list;
  }

  /** 操作中の選手を追う3人称オービットカメラ (ドラッグで yaw/pitch) */
  private updatePlayerCam(): void {
    const cp = this.controlledPlayer;
    if (!cp) return;
    const { yaw, pitch, dist } = this.playerCam;
    const cx = cp.pos.x - Math.sin(yaw) * Math.cos(pitch) * dist;
    const cy = this.playerBody.y + 1.15 + Math.sin(pitch) * dist;
    const cz = cp.pos.z - Math.cos(yaw) * Math.cos(pitch) * dist;
    this.camera.position.set(cx, cy, cz);
    this.controls.target.set(cp.pos.x, this.playerBody.y + 0.9, cp.pos.z);
  }

  /** FPSモード: 砲塔視点。マウスで manual.aimYaw/aimPitch を操作 (App側でポインタロック) */
  addAim(dyaw: number, dpitch: number): void {
    const m = this.match.manual;
    m.aimYaw += dyaw;
    while (m.aimYaw > Math.PI) m.aimYaw -= Math.PI * 2;
    while (m.aimYaw < -Math.PI) m.aimYaw += Math.PI * 2;
    m.aimPitch = Math.min(1.2, Math.max(0.03, m.aimPitch + dpitch));
  }

  /** Q22: 会場ビジョン — 残り時間・得点・チーム名をライブ描画 */
  private updateTvScreen(): void {
    const m = this.match;
    const g = this.field.tvCtx;
    const W = 1024;
    const H = 256;
    const fin = m.over || m.timeExpired;
    const remain = Math.max(0, 180 - m.t);
    const mm = Math.floor(remain / 60);
    const ss = Math.floor(remain % 60);
    g.fillStyle = '#05070c';
    g.fillRect(0, 0, W, H);
    // 上帯
    g.fillStyle = '#10141d';
    g.fillRect(0, 0, W, 52);
    g.fillStyle = '#8fa2bd';
    g.font = '700 30px "Noto Sans JP", sans-serif';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText('高専杯 雑巾投擲選手権 2026', W / 2, 27);
    // 中央: 残り時間
    const timeCol = remain <= 10 && !fin ? '#ff5560' : '#eef3fa';
    g.fillStyle = timeCol;
    g.font = '900 96px "Roboto Mono", monospace';
    g.fillText(`${mm}:${String(ss).padStart(2, '0')}`, W / 2, 138);
    g.fillStyle = '#5d6d84';
    g.font = '700 24px "Noto Sans JP", sans-serif';
    g.fillText(fin ? '試合終了' : m.t < 0.02 ? 'まもなく開始' : '残り時間', W / 2, 212);
    // 左右: チーム名と得点
    const drawTeam = (team: 'red' | 'blue', x: number, align: CanvasTextAlign): void => {
      const col = team === 'red' ? '#ff6a72' : '#5aa8ff';
      g.textAlign = align;
      g.fillStyle = col;
      g.font = '700 34px "Noto Sans JP", sans-serif';
      g.fillText(this.teamNames[team].slice(0, 10), x, 96);
      g.fillStyle = '#ffffff';
      g.font = '900 88px "Roboto Mono", monospace';
      g.fillText(String(m.score[team]), x, 178);
    };
    drawTeam('red', 40, 'left');
    drawTeam('blue', W - 40, 'right');
    // 勝者表示
    if (fin) {
      const winner = m.score.blue > m.score.red ? 'blue' : m.score.red > m.score.blue ? 'red' : null;
      if (winner) {
        g.fillStyle = winner === 'red' ? '#ff6a72' : '#5aa8ff';
        g.font = '900 30px "Noto Sans JP", sans-serif';
        g.textAlign = winner === 'red' ? 'left' : 'right';
        g.fillText('WINNER', winner === 'red' ? 40 : W - 40, 40);
      }
    }
    this.field.tvTex.needsUpdate = true;
  }

  /** Q17: 俯瞰 = 平行投影 (真上から) */
  private updateTopCam(): void {
    const w = this.elems.container.clientWidth;
    const h = this.elems.container.clientHeight;
    const aspect = w / Math.max(1, h);
    this.topCam.left = -this.topHalf * aspect;
    this.topCam.right = this.topHalf * aspect;
    this.topCam.top = this.topHalf;
    this.topCam.bottom = -this.topHalf;
    this.topCam.position.set(this.topPan.x, 24, this.topPan.z);
    this.topCam.up.set(0, 0, -1);
    this.topCam.lookAt(this.topPan.x, 0, this.topPan.z);
    this.topCam.updateProjectionMatrix();
  }

  private updateFpsCam(): void {
    const b = this.match.player;
    const yaw = b.theta + b.turretYaw;
    const pitch = b.turretPitch;
    const cw = this.elems.container.clientWidth;
    const ch = this.elems.container.clientHeight;
    this.fpsCam.aspect = cw / ch;
    this.fpsCam.updateProjectionMatrix();
    const px = b.x + Math.sin(yaw) * 0.3;
    const pz = b.z + Math.cos(yaw) * 0.3;
    const py = 1.62;
    this.fpsCam.position.set(px, py, pz);
    this.fpsCam.lookAt(
      px + Math.sin(yaw) * Math.cos(pitch),
      py + Math.sin(pitch),
      pz + Math.cos(yaw) * Math.cos(pitch),
    );
  }

  private recordPerf(
    dt: number,
    frameMs: number,
    simMs: number,
    visualMs: number,
    renderMs: number,
  ): void {
    this.perfWindow += Math.max(0, dt);
    this.perfFrames++;
    this.perfFrameMs += frameMs;
    this.perfSimMs += simMs;
    this.perfVisualMs += visualMs;
    this.perfRenderMs += renderMs;
    this.perfMaxFrameMs = Math.max(this.perfMaxFrameMs, frameMs);
    if (this.perfWindow < 0.5 || !this.onPerf) return;

    const frames = Math.max(1, this.perfFrames);
    const frameAvg = this.perfFrameMs / frames;
    const budget = 1000 / 60;
    const info = this.renderer.info.render;
    this.onPerf({
      fps: frames / this.perfWindow,
      frameMs: frameAvg,
      simMs: this.perfSimMs / frames,
      visualMs: this.perfVisualMs / frames,
      renderMs: this.perfRenderMs / frames,
      maxFrameMs: this.perfMaxFrameMs,
      budgetPct: (frameAvg / budget) * 100,
      headroomMs: budget - frameAvg,
      drawCalls: info.calls,
      triangles: info.triangles,
      points: info.points,
    });
    this.perfWindow = 0;
    this.perfFrames = 0;
    this.perfFrameMs = 0;
    this.perfSimMs = 0;
    this.perfVisualMs = 0;
    this.perfRenderMs = 0;
    this.perfMaxFrameMs = 0;
  }

  /** 自機視点: ロボット相対のオービット。ドラッグで角度、ホイールで距離 (UX改善) */
  private followOff = { yaw: Math.PI, pitch: 0.5, dist: 3.6 };
  private followDrag = false;

  followPointerDown(): void {
    if (this.playerWalkActive || this.cameraMode === 'top') {
      this.followDrag = true;
      return;
    }
    if (this.cameraMode === 'onboard' && !this.tpsActive && !this.fpsActive) this.followDrag = true;
  }

  followPointerUp(): void {
    this.followDrag = false;
  }

  followPointerMove(mx: number, my: number): void {
    if (!this.followDrag) return;
    if (this.playerWalkActive) {
      this.playerCam.yaw -= mx * 0.006;
      this.playerCam.pitch = Math.min(1.25, Math.max(0.05, this.playerCam.pitch + my * 0.004));
      return;
    }
    if (this.cameraMode === 'top') {
      // ドラッグ量(px)をワールド距離に換算してパン (掴んで動かす向き)。
      // 俯瞰は上=-z / 右=+x なので、画面右ドラッグ→内容を右へ=カメラ中心を-x
      const h = Math.max(1, this.elems.container.clientHeight);
      const wpp = (2 * this.topHalf) / h;
      this.topPan.x = Math.min(8, Math.max(-8, this.topPan.x - mx * wpp));
      this.topPan.z = Math.min(8, Math.max(-8, this.topPan.z - my * wpp));
      return;
    }
    this.followOff.yaw -= mx * 0.006;
    this.followOff.pitch = Math.min(1.35, Math.max(0.06, this.followOff.pitch + my * 0.004));
  }

  followWheel(deltaY: number): void {
    if (this.playerWalkActive) {
      this.playerCam.dist = Math.min(10, Math.max(2.5, this.playerCam.dist * (1 + Math.sign(deltaY) * 0.1)));
      return;
    }
    if (this.cameraMode === 'top') {
      this.topHalf = Math.min(11, Math.max(3.2, this.topHalf * (1 + Math.sign(deltaY) * 0.1)));
      return;
    }
    if (this.cameraMode !== 'onboard') return;
    this.followOff.dist = Math.min(9, Math.max(1.6, this.followOff.dist * (1 + Math.sign(deltaY) * 0.12)));
  }

  private updateFollowCamera(dt: number): void {
    if (this.cameraMode !== 'onboard' || this.tpsActive || this.fpsActive) return;
    const r = this.match.player;
    const az = r.theta + this.followOff.yaw;
    const horiz = this.followOff.dist * Math.cos(this.followOff.pitch);
    const desired = new THREE.Vector3(
      r.x + Math.sin(az) * horiz,
      0.9 + this.followOff.dist * Math.sin(this.followOff.pitch),
      r.z + Math.cos(az) * horiz,
    );
    const k = Math.min(1, dt * 8);
    this.camera.position.lerp(desired, k);
    this.controls.target.lerp(new THREE.Vector3(r.x, 1.0, r.z), k);
    this.camera.lookAt(this.controls.target);
  }

  private pushHud(force: boolean): void {
    if (!this.onHud) return;
    const m = this.match;
    this.onHud({
      t: m.t,
      remain: Math.max(0, 180 - m.t),
      over: m.over || m.timeExpired,
      score: { ...m.score },
      breakdown: m.breakdown,
      status: { blue: m.blue.status, red: m.red.status },
      events: force ? [...m.events] : m.events,
      metrics: m.metrics(),
      manualPower: m.manual.power,
      fireReady: m.manual.fireCooldown <= 0 && m.player.ammo > 0,
      held: {
        blue: m.blue.ammo,
        blueSuper: m.blue.superAmmo,
        red: m.red.ammo,
        redSuper: m.red.superAmmo,
      },
    });
  }

  private syncVisuals(dt: number): void {
    const m = this.match;
    // バケツ高さが変わったら、その値で焼き込まれた描画を作り直す
    for (const team of ['blue', 'red'] as const) {
      if (m.par.bucketTopY[team] !== this.builtBucketTopY[team]) {
        this.builtBucketTopY[team] = m.par.bucketTopY[team];
        this.rebuildRobotVisual(team);
      }
    }
    this.blueVis.update(m.blue, dt, m.t);
    this.redVis.update(m.red, dt, m.t);

    // Q24: 操作側ロボットは「推定ポーズ」で描画し、真値は半透明ワイヤ枠で示す。
    // リトライ搬送中はセンサー凍結のため真値表示に戻す。
    {
      const pv = m.playerTeam === 'blue' ? this.blueVis : this.redVis;
      const pl = m.player;
      if (!pl.retry) {
        const est = m.localizer.est;
        pv.root.position.x = est.x;
        pv.root.position.z = est.z;
        pv.root.rotation.y = est.theta;
        this.trueGhost.visible = this.showLidar;
        this.trueGhost.position.set(pl.x, 0.55 + pl.liftY, pl.z);
        this.trueGhost.rotation.y = pl.theta;
      } else {
        this.trueGhost.visible = false;
      }
    }
    this.handleRefereeEvents();
    this.updateReferees(dt);
    this.updateFieldPlayers(dt);
    this.updateBroadcast(dt);

    // 投擲中の雑巾 (バックスピン + はためき + 射出直後に開く)
    const seen = new Set<number>();
    for (const pr of m.projectiles) {
      seen.add(pr.id);
      let mesh = this.projMeshes.get(pr.id);
      if (!mesh) {
        mesh = this.makeRagMesh(pr);
        this.projMeshes.set(pr.id, mesh);
        this.scene.add(mesh);
      }
      mesh.position.set(pr.x, pr.y, pr.z);
      mesh.rotation.x += pr.spin * dt * (pr.outcome ? 1 : 0.55);
      if (!pr.outcome) {
        mesh.rotation.z = Math.sin(pr.t * 13) * 0.6;
        mesh.rotation.y += dt * 2.5;
      }
      mesh.scale.setScalar(Math.min(1, 0.5 + pr.t * 2.2));
      // 頂点はためき
      const geo = mesh.geometry as THREE.PlaneGeometry;
      const pos = geo.getAttribute('position') as THREE.BufferAttribute;
      const base = mesh.userData.base as Float32Array;
      const amp = pr.outcome ? 0.016 : 0.05;
      const ph = (mesh.userData.phase as number) + pr.t * 19;
      for (let i = 0; i < pos.count; i++) {
        const bx = base[i * 3]!;
        const by = base[i * 3 + 1]!;
        pos.setZ(i, Math.sin(bx * 22 + ph) * amp + Math.cos(by * 18 + ph * 1.3) * amp * 0.6);
      }
      pos.needsUpdate = true;
      geo.computeVertexNormals();
    }
    for (const [id, mesh] of this.projMeshes) {
      if (!seen.has(id)) {
        this.scene.remove(mesh);
        this.projMeshes.delete(id);
      }
    }

    // 着地済みの雑巾
    while (this.hungCount < m.hung.length) {
      const hr = m.hung[this.hungCount]!;
      const obj = this.makeHungMesh(hr);
      this.hungMeshes.set(hr.id, obj);
      this.hungGroup.add(obj);
      this.hungCount++;
    }
    this.updateAttachedHungRags();

    // 補充スポットの雑巾の山 (残量で高さが変わる)
    for (const team of ['blue', 'red'] as const) {
      const pile = this.field.ragPiles[team];
      const s = Math.min(1, m.spotStock[team] / 10);
      pile.visible = s > 0.01;
      pile.scale.y = Math.max(0.15, s);
      pile.position.y = 0.762 + 0.025 * Math.max(0.15, s);
    }

    // A* 経路
    const player = m.player;
    if (this.showPath && player.path && player.path.length > 0) {
      const pts = [{ x: player.x, z: player.z }, ...player.path.slice(player.pathIdx)];
      const segments = Math.min(pts.length - 1, 63);
      const halfW = 0.05;
      // 各点の目標速度を追従制御と同じ規則で算出 (終点ブレーキ+コーナー曲率制限)
      const dpar = player.swerve ? m.par.swerveDrive : m.par.drive;
      const vmax = dpar.vmax * Math.max(0.1, player.driveMul);
      const acc = dpar.acc * Math.max(0.1, player.driveMul);
      const np = pts.length;
      const segLen: number[] = [];
      for (let i = 0; i < np - 1; i++) segLen.push(Math.hypot(pts[i + 1]!.x - pts[i]!.x, pts[i + 1]!.z - pts[i]!.z));
      const remain: number[] = new Array(np).fill(0);
      for (let i = np - 2; i >= 0; i--) remain[i] = remain[i + 1]! + segLen[i]!;
      // 累積弧長 (曲率を弧長の窓で安定推定するため)
      const cum: number[] = new Array(np).fill(0);
      for (let i = 1; i < np; i++) cum[i] = cum[i - 1]! + segLen[i - 1]!;
      const safeAcc = 0.88 * acc; // 追従制御の予見ブレーキと同じ旋回予算
      const WIN = 0.5; // 前後この弧長の点で曲率を測る (微小セグメントのスパイク防止)
      const speedAt = (i: number): number => {
        const brakeV = Math.sqrt(2 * acc * Math.max(0, remain[i]! - 0.03));
        let curveV = vmax;
        // i から弧長 WIN だけ前後の点を取り、その間の向き変化で曲率半径を推定する
        let jb = i;
        while (jb > 0 && cum[i]! - cum[jb - 1]! < WIN) jb--;
        let jf = i;
        while (jf < np - 1 && cum[jf + 1]! - cum[i]! < WIN) jf++;
        if (jb < i && jf > i) {
          const a1 = Math.atan2(pts[i]!.x - pts[jb]!.x, pts[i]!.z - pts[jb]!.z);
          const a2 = Math.atan2(pts[jf]!.x - pts[i]!.x, pts[jf]!.z - pts[i]!.z);
          let turn = Math.abs(a2 - a1);
          if (turn > Math.PI) turn = 2 * Math.PI - turn;
          const arcW = cum[jf]! - cum[jb]!;
          if (turn > 0.06 && arcW > 0.05) curveV = Math.sqrt((safeAcc * arcW) / turn); // R = arcW/turn
        }
        return Math.min(vmax, brakeV, curveV);
      };
      // 速度→色 (遅い=赤 / 中=黄 / 速い=緑)
      const col = new THREE.Color();
      const colorAt = (i: number): THREE.Color => {
        const f = Math.max(0, Math.min(1, speedAt(i) / Math.max(0.3, vmax)));
        return col.setRGB(Math.min(1, 2 * (1 - f)) * 0.95 + 0.05, Math.min(1, 2 * f) * 0.9 + 0.1, 0.12);
      };
      let v = 0;
      for (let i = 0; i < segments; i++) {
        const a = pts[i]!;
        const b = pts[i + 1]!;
        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const len = Math.hypot(dx, dz);
        if (len < 1e-4) continue;
        const nx = (-dz / len) * halfW;
        const nz = (dx / len) * halfW;
        const y = 0.045;
        const ca = colorAt(i).clone();
        const cb = colorAt(i + 1).clone();
        const setV = (x: number, z: number, c: THREE.Color): void => {
          this.pathPos.setXYZ(v, x, y, z);
          this.pathCol.setXYZ(v, c.r, c.g, c.b);
          v++;
        };
        setV(a.x + nx, a.z + nz, ca);
        setV(a.x - nx, a.z - nz, ca);
        setV(b.x + nx, b.z + nz, cb);
        setV(b.x + nx, b.z + nz, cb);
        setV(a.x - nx, a.z - nz, ca);
        setV(b.x - nx, b.z - nz, cb);
      }
      this.pathMesh.visible = v > 0;
      this.pathMesh.geometry.setDrawRange(0, v);
      this.pathPos.needsUpdate = true;
      this.pathCol.needsUpdate = true;
    } else {
      this.pathMesh.visible = false;
      this.pathMesh.geometry.setDrawRange(0, 0);
    }

    // LiDAR 点群: センサー出力を「推定ポーズ」で世界へ復元して表示
    // (実機のRvizと同じ。推定がズレれば点群が壁からズレて見える = 正直な表示)
    // 下段は前後2台 (UST-20LX×2) のスキャンをまとめて描画
    const lidarEst = m.localizer.est;
    if (m.lastScan && this.showLidar) {
      this.lidarPts.visible = true;
      let k = 0;
      for (const scan of [m.lastScan, m.lastScanRear]) {
        if (!scan) continue;
        const { pts, n } = scanToLocalPoints(scan);
        for (let i = 0; i < n && k < 600; i++, k++) {
          const [wx, wz] = localToWorld(pts[i * 2]!, pts[i * 2 + 1]!, lidarEst);
          this.lidarPos.setXYZ(k, wx, LIDAR_SCAN_HEIGHT, wz);
        }
      }
      this.lidarPts.geometry.setDrawRange(0, k);
      this.lidarPos.needsUpdate = true;
    } else {
      this.lidarPts.visible = false;
      this.lidarPts.geometry.setDrawRange(0, 0);
    }
    if (m.lastUpperScan && this.showLidar) {
      this.upperPts.visible = true;
      const { pts, n } = scanToLocalPoints(m.lastUpperScan);
      const k = Math.min(n, 280);
      for (let i = 0; i < k; i++) {
        const [wx, wz] = localToWorld(pts[i * 2]!, pts[i * 2 + 1]!, lidarEst);
        this.upperPos.setXYZ(i, wx, LIDAR_UPPER_HEIGHT, wz);
      }
      this.upperPts.geometry.setDrawRange(0, k);
      this.upperPos.needsUpdate = true;
    } else {
      this.upperPts.visible = false;
      this.upperPts.geometry.setDrawRange(0, 0);
    }

    // ICP推定ポーズ矢印 + 不確かさリング (T8)。10Hz更新の段差は視覚のみ軽く補間
    const est = m.localizer.est;
    const kk = Math.min(1, dt * 14);
    this.estArrow.position.x += (est.x - this.estArrow.position.x) * kk;
    this.estArrow.position.z += (est.z - this.estArrow.position.z) * kk;
    let dth = est.theta - this.estArrow.rotation.y;
    while (dth > Math.PI) dth -= Math.PI * 2;
    while (dth < -Math.PI) dth += Math.PI * 2;
    this.estArrow.rotation.y += dth * kk;
    this.estArrow.visible = this.showLidar;
    const unc = Math.min(0.6, 0.06 + m.localizer.diag.rmse * 3);
    this.estRing.scale.setScalar(unc);
    this.estRing.position.x = this.estArrow.position.x;
    this.estRing.position.z = this.estArrow.position.z;
    this.estRing.visible = this.showLidar;

    // 相手ゴースト (LiDARクラスタ推定の実出力, T21)
    const opp = m.oppTracker.est;
    if (opp && this.showLidar) {
      this.ghost.visible = true;
      this.ghost.position.x = opp.x;
      this.ghost.position.z = opp.z;
    } else {
      this.ghost.visible = false;
    }
  }

  private updateAttachedHungRags(): void {
    for (const hr of this.match.hung) {
      if (!hr.attachedTo) continue;
      const obj = this.hungMeshes.get(hr.id);
      if (!obj) continue;
      const carrier = hr.attachedTo === 'blue' ? this.match.blue : this.match.red;
      const lx = hr.localX ?? 0;
      const ly = hr.localY ?? hr.y;
      const lz = hr.localZ ?? 0;
      const c = Math.cos(carrier.theta);
      const s = Math.sin(carrier.theta);
      obj.position.set(
        carrier.x + lx * c + lz * s,
        Math.max(ly, 0.015),
        carrier.z - lx * s + lz * c,
      );
      obj.rotation.y = carrier.theta + hr.yaw;
    }
  }

  private makeReferee(x: number, z: number, flagColor: number, chief = false): RefereeVisual {
    const root = new THREE.Group();
    root.position.set(x, 0, z);
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.46, 0.15), new THREE.MeshLambertMaterial({ color: 0xf5f5f0 }));
    body.position.y = 0.74;
    const pants = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.42, 0.14), new THREE.MeshLambertMaterial({ color: 0x111318 }));
    pants.position.y = 0.29;
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.1, 10, 8), new THREE.MeshLambertMaterial({ color: 0xe8c39e }));
    head.position.y = 1.06;
    const hat = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.1, 0.065, 16), new THREE.MeshLambertMaterial({ color: 0x050608 }));
    hat.position.y = 1.16;
    const brim = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.012, 16), new THREE.MeshLambertMaterial({ color: 0x050608 }));
    brim.position.y = 1.12;
    const arm = new THREE.Group();
    arm.position.set(0.17, 0.89, 0);
    const sleeve = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.32, 0.055), new THREE.MeshLambertMaterial({ color: 0xf5f5f0 }));
    sleeve.position.y = -0.16;
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.42, 8), new THREE.MeshLambertMaterial({ color: 0x20242c }));
    pole.rotation.z = Math.PI / 2;
    pole.position.set(0.18, -0.31, 0);
    const flag = new THREE.Mesh(
      new THREE.PlaneGeometry(0.26, 0.18),
      new THREE.MeshLambertMaterial({ color: flagColor, side: THREE.DoubleSide }),
    );
    flag.position.set(0.38, -0.31, 0);
    arm.add(sleeve, pole, flag);
    arm.rotation.z = -0.25;
    root.add(body, pants, head, hat, brim, arm);
    // 主審: 左腕にも旗 (青)。勝利判定時に該当色の旗を振る
    let arm2: THREE.Group | undefined;
    if (chief) {
      arm2 = new THREE.Group();
      arm2.position.set(-0.17, 0.89, 0);
      const sleeve2 = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.32, 0.055), new THREE.MeshLambertMaterial({ color: 0xf5f5f0 }));
      sleeve2.position.y = -0.16;
      const pole2 = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.42, 8), new THREE.MeshLambertMaterial({ color: 0x20242c }));
      pole2.rotation.z = Math.PI / 2;
      pole2.position.set(-0.18, -0.31, 0);
      const flag2 = new THREE.Mesh(
        new THREE.PlaneGeometry(0.26, 0.18),
        new THREE.MeshLambertMaterial({ color: 0x0068b7, side: THREE.DoubleSide }),
      );
      flag2.position.set(-0.38, -0.31, 0);
      arm2.add(sleeve2, pole2, flag2);
      arm2.rotation.z = 0.25;
      root.add(arm2);
    }
    root.lookAt(0, 0.75, 0);
    root.traverse((o) => {
      if (o instanceof THREE.Mesh) o.castShadow = true;
    });
    return { root, arm, flag, arm2, timer: 0, chief };
  }

  private triggerReferee(index: number, dur = 1.7): void {
    const ref = this.referees[index];
    if (ref) ref.timer = Math.max(ref.timer, dur);
  }

  private handleRefereeEvents(): void {
    const events = this.match.events;
    while (this.refEventCursor < events.length) {
      const e = events[this.refEventCursor]!;
      this.refEventCursor++;
      const score = (e.cls === 'hit' || e.cls === 'opp' || e.cls === 'ev') && /\+\d+/.test(e.text);
      if (e.text.includes('競技スタート') || e.text.includes('リトライ') || e.text.includes('ジャム') || e.text.includes('復旧')) {
        this.triggerReferee(0, 2.0);
      }
      if (score) {
        if (e.text.includes('青チーム')) this.triggerReferee(1, 1.8);
        else if (e.text.includes('赤チーム')) this.triggerReferee(2, 1.8);
        else this.triggerReferee(0, 1.4);
      }
    }
  }

  private updateReferees(dt: number): void {
    const m = this.match;
    const fin = m.over || m.timeExpired;
    const winner = fin ? (m.score.blue > m.score.red ? 'blue' : m.score.red > m.score.blue ? 'red' : null) : null;
    for (const ref of this.referees) {
      ref.timer = Math.max(0, ref.timer - dt);
      if (ref.chief && winner) {
        // 主審: 勝者側の旗を高く上げて振る (右腕=赤, 左腕=青)
        const swing = Math.sin(this.clockT * 6) * 0.35;
        const redUp = winner === 'red';
        const rTarget = redUp ? -2.4 + swing : -0.25;
        const lTarget = !redUp ? 2.4 - swing : 0.25;
        ref.arm.rotation.z += (rTarget - ref.arm.rotation.z) * Math.min(1, dt * 10);
        if (ref.arm2) ref.arm2.rotation.z += (lTarget - ref.arm2.rotation.z) * Math.min(1, dt * 10);
        continue;
      }
      const up = ref.timer > 0 ? 1 : 0;
      const target = -0.25 - up * 1.75;
      ref.arm.rotation.z += (target - ref.arm.rotation.z) * Math.min(1, dt * 12);
      if (ref.arm2) ref.arm2.rotation.z += (0.25 - ref.arm2.rotation.z) * Math.min(1, dt * 12);
      ref.flag.rotation.y = Math.sin(this.clockT * 8) * (0.04 + up * 0.12);
    }
  }

  /** 放送機材: Scorpio 風カメラクレーン1基 + 肩担ぎカメラマン数名 */
  private buildBroadcast(): void {
    const lam = (c: number): THREE.MeshLambertMaterial => new THREE.MeshLambertMaterial({ color: c });
    // ===== Scorpio テレスコピッククレーン (会場外 -x/+z 隅) =====
    const base = new THREE.Vector3(-(FIELD.w / 2 + 2.8), 0, FIELD.l / 2 + 2.4);
    const g = new THREE.Group();
    g.position.copy(base);
    // タイヤ4輪 (黒ゴム + シルバーハブ)。ドリーはこの上に載る
    const wheelR = 0.26;
    for (const [wx, wz] of [
      [-0.62, -0.72],
      [0.62, -0.72],
      [-0.62, 0.72],
      [0.62, 0.72],
    ] as const) {
      const tire = new THREE.Mesh(new THREE.CylinderGeometry(wheelR, wheelR, 0.18, 22), lam(0x0b0c10));
      tire.rotation.z = Math.PI / 2;
      tire.position.set(wx, wheelR, wz);
      tire.castShadow = true;
      g.add(tire);
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.2, 14), lam(0x9aa3b2));
      hub.rotation.z = Math.PI / 2;
      hub.position.set(wx, wheelR, wz);
      g.add(hub);
      // フォーク (脚) でドリーと接続
      const strut = new THREE.Mesh(new THREE.BoxGeometry(0.06, wheelR + 0.1, 0.06), lam(0x2b3038));
      strut.position.set(wx, wheelR + 0.1, wz);
      g.add(strut);
    }
    const dollyY = wheelR * 2 + 0.14; // タイヤの上端 + 半分
    const dolly = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.24, 1.9), lam(0x181b22));
    dolly.position.y = dollyY;
    dolly.castShadow = true;
    g.add(dolly);
    const dollyTop = dollyY + 0.12;
    const col = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.22, 1.6, 16), lam(0x2b3038));
    col.position.y = dollyTop + 0.8;
    col.castShadow = true;
    g.add(col);
    const pan = new THREE.Group();
    pan.position.y = dollyTop + 1.6;
    // 既定でフィールド中心へ向ける (クレーン操縦モードの初期視点が場内を向くように)
    pan.rotation.y = Math.atan2(-base.x, -base.z);
    g.add(pan);
    const boom = new THREE.Group();
    pan.add(boom);
    // カウンターウェイト (近端 -z)
    const cw = new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.44, 0.6), lam(0x101216));
    cw.position.set(0, 0, -1.1);
    boom.add(cw);
    const yoke = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.3, 0.3), lam(0x2b3038));
    boom.add(yoke);
    // テレスコピックアーム (遠端 +z、2段でシルバー→黒)
    const arm1 = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.17, 4.6), lam(0x3a4048));
    arm1.position.set(0, 0, 2.1);
    arm1.castShadow = true;
    boom.add(arm1);
    const arm2 = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 3.6), lam(0x8b93a1));
    arm2.position.set(0, 0, 4.7);
    boom.add(arm2);
    // ヘッド (ジンバル + カメラ) — アーム先端
    const head = new THREE.Group();
    head.position.set(0, 0, 6.4);
    boom.add(head);
    const gimbal = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.22, 0.22), lam(0x14161c));
    head.add(gimbal);
    const camBody = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.28, 0.46), lam(0x0d0e12));
    camBody.position.set(0, -0.16, 0.06);
    camBody.castShadow = true;
    head.add(camBody);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.12, 0.36, 20), lam(0x05060a));
    lens.rotation.x = Math.PI / 2;
    lens.position.set(0, -0.16, 0.34);
    head.add(lens);
    const tally = new THREE.Mesh(
      new THREE.SphereGeometry(0.032, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0xff2020 }),
    );
    tally.position.set(0, 0, 0.3);
    head.add(tally);
    boom.rotation.x = -0.22;
    this.scene.add(g);
    this.crane = { pan, boom, head, base, arm2 };

    // ===== 肩担ぎカメラマン (外周の要所) =====
    const spots: THREE.Vector3[] = [
      new THREE.Vector3(FIELD.w / 2 + 1.15, 0, -(FIELD.l / 2 - 1.2)),
      new THREE.Vector3(-(FIELD.w / 2 + 1.15), 0, -(FIELD.l / 2 - 3.4)),
      new THREE.Vector3(FIELD.w / 2 + 1.15, 0, FIELD.l / 2 - 3.0),
      new THREE.Vector3(0.9, 0, FIELD.l / 2 + 1.25),
    ];
    for (const s of spots) this.cameramen.push(this.makeCameraman(s));
    this.scene.add(...this.cameramen.map((c) => c.root));

    // ===== 俯瞰カメラ (6台目) =====
    // 大看板は +x 側 (z=0 中央) にある。看板が背景に写る -x 側の中央・高所に据え、
    // フィールド全体を斜め上から捉える固定ワイド。
    this.overheadCam.position.set(-(FIELD.w / 2 + 4.0), 9.5, 0);
    this.overheadCam.up.set(0, 1, 0);
    this.overheadCam.lookAt(0.6, 0.7, 0);
  }

  private makeCameraman(pos: THREE.Vector3): {
    root: THREE.Group;
    aim: THREE.Group;
    home: THREE.Vector3;
    pos: THREE.Vector3;
    phase: number;
    cam: THREE.PerspectiveCamera;
  } {
    const lam = (c: number): THREE.MeshLambertMaterial => new THREE.MeshLambertMaterial({ color: c });
    const root = new THREE.Group();
    root.position.copy(pos);
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.5, 0.18), lam(0x2a2e38));
    body.position.y = 0.74;
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.1, 10, 8), lam(0xe8c39e));
    head.position.y = 1.06;
    const cap = new THREE.Mesh(
      new THREE.SphereGeometry(0.108, 10, 8, 0, Math.PI * 2, 0, Math.PI / 2),
      lam(0x14161c),
    );
    cap.position.y = 1.09;
    const legL = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.44, 0.08), lam(0x1b1e26));
    legL.position.set(-0.07, 0.28, 0);
    const legR = legL.clone();
    legR.position.x = 0.07;
    root.add(body, head, cap, legL, legR);
    // 肩担ぎカメラ (ヨー/ピッチする aim グループ)
    const aim = new THREE.Group();
    aim.position.set(0.06, 0.98, 0.05);
    const cbody = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.22, 0.5), lam(0x101216));
    cbody.position.z = 0.12;
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 0.3, 18), lam(0x05060a));
    lens.rotation.x = Math.PI / 2;
    lens.position.z = 0.42;
    const hood = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.1, 0.06, 18), lam(0x1a1d24));
    hood.rotation.x = Math.PI / 2;
    hood.position.z = 0.56;
    const tally = new THREE.Mesh(
      new THREE.SphereGeometry(0.022, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0xff2020 }),
    );
    tally.position.set(0, 0.12, 0.2);
    aim.add(cbody, lens, hood, tally);
    root.add(aim);
    root.traverse((o) => {
      if (o instanceof THREE.Mesh) o.castShadow = true;
    });
    const cam = new THREE.PerspectiveCamera(46, 16 / 9, 0.05, 200);
    return { root, aim, home: pos.clone(), pos: pos.clone(), phase: pos.x + pos.z, cam };
  }

  /** マルチカム: 各カメラ(カメラマン肩カメラ + クレーン先端)を現在の姿勢に合わせる */
  private updateMultiCamCameras(): void {
    const wp = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const m = this.match;
    for (const cm of this.cameramen) {
      cm.aim.getWorldPosition(wp);
      cm.aim.getWorldQuaternion(q);
      const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
      cm.cam.position.copy(wp).add(fwd.clone().multiplyScalar(0.6));
      cm.cam.up.set(0, 1, 0);
      cm.cam.lookAt(wp.x + fwd.x, wp.y + fwd.y, wp.z + fwd.z);
      // 動的ズーム: 追う対象(最寄りロボット)までの距離で FOV を変える (遠い=寄る/狭FOV)
      const db = Math.hypot(cm.cam.position.x - m.blue.x, cm.cam.position.z - m.blue.z);
      const dr = Math.hypot(cm.cam.position.x - m.red.x, cm.cam.position.z - m.red.z);
      const dist = Math.min(db, dr);
      const targetFov = Math.max(24, Math.min(60, 62 - dist * 3.4));
      cm.cam.fov += (targetFov - cm.cam.fov) * 0.1;
    }
    this.updateCraneCam();
  }

  /** マルチカム: 画面を 3×2 グリッドに分割して 4カメラ + クレーン を同時描画 */
  private renderMultiCam(): void {
    const W = this.elems.container.clientWidth;
    const H = this.elems.container.clientHeight;
    const cols = 3;
    const rows = 2;
    const cw = Math.floor(W / cols);
    const ch = Math.floor(H / rows);
    const feeds: (THREE.PerspectiveCamera | null)[] = [
      this.cameramen[0]?.cam ?? null,
      this.cameramen[1]?.cam ?? null,
      this.crane ? this.craneCam : null,
      this.cameramen[2]?.cam ?? null,
      this.cameramen[3]?.cam ?? null,
      this.overheadCam,
    ];
    this.renderer.setScissorTest(true);
    for (let i = 0; i < 6; i++) {
      const col = i % cols;
      const row = Math.floor(i / cols);
      const x = col * cw;
      const y = H - (row + 1) * ch; // WebGL は左下原点
      this.renderer.setViewport(x, y, cw, ch);
      this.renderer.setScissor(x, y, cw, ch);
      const cam = feeds[i];
      if (cam) {
        cam.aspect = cw / ch;
        cam.updateProjectionMatrix();
        // クレーン先端カメラのセルは自分のヘッドが写り込むので隠して描く
        const isCrane = i === 2 && this.crane;
        if (isCrane) this.crane.head.visible = false;
        this.renderer.render(this.scene, cam);
        if (isCrane) this.crane.head.visible = true;
      } else {
        this.renderer.setClearColor(0x04060a, 1);
        this.renderer.clear(true, true, false);
      }
    }
    this.renderer.setScissorTest(false);
  }

  /** クレーンとカメラマンをロボットの動きに合わせて動的に動かす (放送カメラワーク) */
  private updateBroadcast(dt: number): void {
    const m = this.match;
    // 撮る対象 = 二機の中間 (アクションの中心)。時々どちらかへ寄る
    const sway = 0.5 + 0.5 * Math.sin(this.clockT * 0.13);
    const ax = m.blue.x * sway + m.red.x * (1 - sway);
    const az = m.blue.z * sway + m.red.z * (1 - sway);

    // --- Scorpio クレーン ---
    // 操縦中(POV)は自分のカメラ(ヘッド)が写り込むので隠す
    if (this.crane) this.crane.head.visible = !this.craneControlActive;
    if (this.crane && this.craneControlActive) {
      // 手動操縦: A/D=旋回, W/S=俯仰, マウス=ヘッド
      const c = this.crane;
      c.pan.rotation.y += this.craneInput.pan * 0.9 * dt;
      c.boom.rotation.x = Math.max(-0.95, Math.min(0.35, c.boom.rotation.x + this.craneInput.lift * 0.6 * dt));
      c.head.rotation.y = this.craneHead.yaw;
      c.head.rotation.x = this.craneHead.pitch;
    } else if (this.crane) {
      const c = this.crane;
      // 旋回: ブーム(+z)を対象へ向ける
      const desPan = Math.atan2(ax - c.base.x, az - c.base.z);
      let dp = desPan - c.pan.rotation.y;
      while (dp > Math.PI) dp -= Math.PI * 2;
      while (dp < -Math.PI) dp += Math.PI * 2;
      c.pan.rotation.y += dp * Math.min(1, dt * 1.5);
      // 俯仰: ゆっくり上下 + 対象が遠いほど水平寄り
      const horiz = Math.hypot(ax - c.base.x, az - c.base.z);
      const boomTarget = -0.32 + Math.sin(this.clockT * 0.22) * 0.14 + Math.min(0.2, (12 - horiz) * 0.02);
      c.boom.rotation.x += (boomTarget - c.boom.rotation.x) * Math.min(1, dt * 1.2);
      // ヘッド: 対象を画面中心に捉える。ワールドの見下ろし角 = boom + head なので、
      // ヘッド先端の実位置から対象への俯角を求めて head.rotation.x を逆算する。
      c.pan.updateMatrixWorld(true);
      const hwp = new THREE.Vector3();
      c.head.getWorldPosition(hwp);
      const horizH = Math.hypot(ax - hwp.x, az - hwp.z);
      const desiredDown = Math.atan2(Math.max(0, hwp.y - 0.45), Math.max(0.4, horizH));
      const headTarget = desiredDown - c.boom.rotation.x;
      c.head.rotation.x += (headTarget - c.head.rotation.x) * Math.min(1, dt * 4);
      c.head.rotation.y = 0;
    }

    // --- ブームの伸縮 (テレスコピック) --- 対象が遠いほど伸ばして寄る
    if (this.crane) {
      const c = this.crane;
      const horiz = Math.hypot(ax - c.base.x, az - c.base.z);
      const extTarget = Math.max(0.1, Math.min(1, (horiz - 5) / 7));
      this.craneExt += (extTarget - this.craneExt) * Math.min(1, dt * 1.2);
      const headZ = 5.4 + this.craneExt * 2.0;
      c.head.position.z = headZ;
      const arm1End = 4.4; // arm1 の先端 z
      c.arm2.position.z = (arm1End + headZ) / 2;
      c.arm2.scale.z = Math.max(0.3, (headZ - arm1End) / 3.6);
    }

    // --- 肩担ぎカメラマン ---
    for (const cm of this.cameramen) {
      // 最寄りのロボットを狙う
      const rb = Math.hypot(cm.pos.x - m.blue.x, cm.pos.z - m.blue.z);
      const rr = Math.hypot(cm.pos.x - m.red.x, cm.pos.z - m.red.z);
      const tgt = rb < rr ? m.blue : m.red;
      // たまに立ち位置を動かす (ホーム周辺をゆっくり移動)
      const hx = cm.home.x + Math.sin(this.clockT * 0.09 + cm.phase) * 1.1;
      const hz = cm.home.z + Math.cos(this.clockT * 0.07 + cm.phase) * 0.6;
      cm.pos.x += (hx - cm.pos.x) * Math.min(1, dt * 0.6);
      cm.pos.z += (hz - cm.pos.z) * Math.min(1, dt * 0.6);
      cm.root.position.set(cm.pos.x, 0, cm.pos.z);
      // 体を対象へ向ける
      const bodyYaw = Math.atan2(tgt.x - cm.pos.x, tgt.z - cm.pos.z);
      let by = bodyYaw - cm.root.rotation.y;
      while (by > Math.PI) by -= Math.PI * 2;
      while (by < -Math.PI) by += Math.PI * 2;
      cm.root.rotation.y += by * Math.min(1, dt * 3);
      // 肩カメラのピッチ: 対象(低い)を見下ろす
      const d = Math.hypot(tgt.x - cm.pos.x, tgt.z - cm.pos.z);
      cm.aim.rotation.x = Math.atan2(0.6, Math.max(0.5, d)) * 0.7;
    }
  }

  private makeFieldPlayer(team: TeamId, slot: number): FieldPlayerVisual {
    const col = team === 'blue' ? 0x1f5fb0 : 0xb52731;
    const shirtCol = team === 'blue' ? 0xf2f5ff : 0x171b22;
    const accentCol = team === 'blue' ? 0x0f3f78 : 0xf0efe8;
    const root = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.48, 0.16), new THREE.MeshLambertMaterial({ color: shirtCol }));
    body.position.y = 0.74;
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.105, 10, 8), new THREE.MeshLambertMaterial({ color: 0xe8c39e }));
    head.position.y = 1.06;
    const helmet = new THREE.Mesh(
      new THREE.SphereGeometry(0.112, 10, 8, 0, Math.PI * 2, 0, Math.PI / 2),
      new THREE.MeshLambertMaterial({ color: 0xf5d327 }),
    );
    helmet.position.y = 1.09;
    const armMat = new THREE.MeshLambertMaterial({ color: 0xd8b08a });
    const legMat = new THREE.MeshLambertMaterial({ color: 0x252a34 });
    const armL = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.34, 0.055), armMat);
    const armR = armL.clone();
    armL.position.set(-0.17, 0.74, 0);
    armR.position.set(0.17, 0.74, 0);
    const legL = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.42, 0.07), legMat);
    const legR = legL.clone();
    legL.position.set(-0.07, 0.28, 0);
    legR.position.set(0.07, 0.28, 0);
    const rag = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.08, 0.18), new THREE.MeshLambertMaterial({ color: 0xefe9dc }));
    rag.position.set(0, 0.68, -0.16);
    rag.visible = false;
    const bibFront = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.31, 0.012), new THREE.MeshLambertMaterial({ color: col }));
    bibFront.position.set(0, 0.76, 0.087);
    const bibBack = bibFront.clone();
    bibBack.position.z = -0.087;
    const sash = new THREE.Mesh(new THREE.BoxGeometry(0.27, 0.045, 0.18), new THREE.MeshLambertMaterial({ color: accentCol }));
    sash.position.y = 0.53;
    root.add(body, bibFront, bibBack, sash, head, helmet, armL, armR, legL, legR, rag);
    // 6角レンチ (L字ヘックスキー) を右腰にぶら下げる。全選手が携行 (即修理用)
    const wrench = new THREE.Group();
    const wMat = new THREE.MeshPhongMaterial({ color: 0x53595f, shininess: 90 });
    const wLong = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.17, 6), wMat);
    wLong.position.y = -0.085;
    const wShort = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.055, 6), wMat);
    wShort.rotation.z = Math.PI / 2;
    wShort.position.x = 0.027;
    wrench.add(wLong, wShort);
    wrench.position.set(0.135, 0.5, 0.055);
    wrench.rotation.x = 0.15;
    root.add(wrench);
    // Q8: 操縦者 (slot1) はスマホを両手で構える
    let phone: THREE.Mesh | null = null;
    if (slot === 1) {
      phone = new THREE.Mesh(
        new THREE.BoxGeometry(0.075, 0.012, 0.15),
        new THREE.MeshLambertMaterial({ color: 0x11141a }),
      );
      phone.rotation.x = 0.5;
      phone.position.set(0, 0.82, 0.22);
      root.add(phone);
    }
    root.traverse((o) => {
      if (o instanceof THREE.Mesh) o.castShadow = true;
    });
    const pos = this.fieldPlayerTarget({ team, slot } as FieldPlayerVisual, 0);
    root.position.copy(pos);
    return { team, slot, root, body, head, armL, armR, legL, legR, rag, phone, pos: pos.clone() };
  }

  private fieldPlayerTarget(p: FieldPlayerVisual, t: number): THREE.Vector3 {
    const sign = p.team === 'blue' ? 1 : -1;
    const robot = p.team === 'blue' ? this.match.blue : this.match.red;
    const resup = homePos(p.team, 'resup');
    const control = homePos(p.team, 'control');
    const wp = waypoints(p.team);
    if (robot.retry) {
      if (p.slot < 2) {
        const side = p.slot === 0 ? -1 : 1;
        if (robot.retry.phase === 'repair' || robot.retry.phase === 'homing') {
          return new THREE.Vector3(wp.start.x + side * 0.34, 0, wp.start.z - sign * 0.42);
        }
        const spread = robot.retry.phase === 'declared' || robot.retry.phase === 'approach' ? 0.42 : 0.31;
        return new THREE.Vector3(robot.x + side * spread, 0, robot.z + sign * 0.16);
      }
      return new THREE.Vector3(control.x - 0.32, 0, sign * (FIELD.l / 2 + 0.45));
    }
    // リトライ搬送以外では選手はフィールド外 (フェンスの外側) にのみ居る。
    const OX = FIELD.w / 2 + 0.45; // 外周の左右 (フェンス外)
    const OZ = FIELD.l / 2 + 0.45; // 外周の遠端 (フェンス外)
    const zNear = 0.9; // 中央(教壇)寄りの端。これより中央には行かない
    const clampHalf = (z: number): number =>
      sign > 0 ? Math.max(zNear, Math.min(OZ, z)) : Math.min(-zNear, Math.max(-OZ, z));
    const refilling =
      robot.cur?.t === 'pickup' ||
      robot.status.includes('補充') ||
      robot.status.includes('ピックアップ') ||
      robot.anim.grab >= 0;

    if (p.slot === 0) {
      // 補充係: 常にスタートゾーン/補充スポットの前 (+xフェンス外) に固定。補充時だけ身を乗り出す。
      const lean = refilling ? (robot.anim.grab >= 0 ? robot.anim.grab : (Math.sin(t * 4) + 1) / 2) : 0;
      return new THREE.Vector3(FIELD.w / 2 + 0.45 - lean * 0.32, 0, resup.z + Math.sin(t * 0.6) * 0.05);
    }
    if (p.slot === 1) {
      // 操縦者: CS裏のサイドフェンス外でスマホ操縦 (据え置き)
      return new THREE.Vector3(control.x - 0.85 + Math.sin(t * 0.5) * 0.08, 0, sign * (FIELD.l / 2 + 0.42));
    }
    // slot 2: ロボットに一番近い外周(フェンス外)位置に追従するスポッター
    const rx = robot.x;
    const rz = robot.z;
    const dLeft = Math.abs(rx + OX);
    const dRight = Math.abs(rx - OX);
    const dFar = Math.abs(rz - sign * OZ);
    const mn = Math.min(dLeft, dRight, dFar);
    if (mn === dFar) return new THREE.Vector3(Math.max(-OX, Math.min(OX, rx)), 0, sign * OZ);
    if (mn === dLeft) return new THREE.Vector3(-OX, 0, clampHalf(rz));
    return new THREE.Vector3(OX, 0, clampHalf(rz));
  }

  /** 非リトライ時、選手は絶対にフィールド内に入らない (フェンス外へ押し出す安全弁) */
  private keepOutsideFence(pos: THREE.Vector3): void {
    const fx = FIELD.w / 2 + 0.32;
    const fz = FIELD.l / 2 + 0.32;
    if (Math.abs(pos.x) < fx && Math.abs(pos.z) < fz) {
      // フェンス内なら最寄りの外へ
      if (fx - Math.abs(pos.x) < fz - Math.abs(pos.z)) pos.x = Math.sign(pos.x || 1) * fx;
      else pos.z = Math.sign(pos.z || 1) * fz;
    }
  }

  private updateFieldPlayers(dt: number): void {
    const m = this.match;
    const fin = m.over || m.timeExpired;
    // 勝利ジャンプは紙吹雪の継続時間 (約12秒) だけ
    const celebrating = fin && this.confettiT >= 0 && this.confettiT < 12;
    const winner = celebrating ? (m.score.blue > m.score.red ? 'blue' : m.score.red > m.score.blue ? 'red' : null) : null;
    const ctrl = this.playerWalkActive ? this.controlledPlayer : undefined;
    for (const p of this.fieldPlayers) {
      const robotForP = p.team === 'blue' ? this.match.blue : this.match.red;
      // 選手操作モード: この選手は物理付き(移動/ジャンプ/当たり判定)で自由に動ける
      if (p === ctrl) {
        this.updateControlledPlayer(p, dt);
        continue;
      }
      // 持ち上げ〜搬送中の運搬者(slot 0/1)はロボットに追従しきれないと「自走」に見えるため、
      // 目標(=ロボット脇)へスナップして常に運んでいる見た目にする。
      const holding =
        p.slot < 2 &&
        !!robotForP.retry &&
        (robotForP.retry.phase === 'lift' || robotForP.retry.phase === 'carry');
      // リトライで持ち上げる前(宣言/接近)は、確実にロボットへ到達できるよう走って向かう
      const rp = robotForP.retry;
      const rushing = p.slot < 2 && !!rp && (rp.phase === 'declared' || rp.phase === 'approach');
      let target = this.fieldPlayerTarget(p, this.match.t);
      // 接近も競技物品を避ける A* 経路で歩く。最終点付近はロボット横のオフセット目標へ切替。
      if (rushing) {
        if (!p.walkPath) {
          p.walkPath = planPath(robotForP.team, { x: p.pos.x, z: p.pos.z }, { x: robotForP.x, z: robotForP.z });
          p.walkIdx = 0;
        }
        const wpArr = p.walkPath;
        if (wpArr && wpArr.length > 0) {
          let idx = p.walkIdx ?? 0;
          while (idx < wpArr.length - 1 && Math.hypot(wpArr[idx]!.x - p.pos.x, wpArr[idx]!.z - p.pos.z) < 0.35) idx++;
          p.walkIdx = idx;
          if (idx < wpArr.length - 1) target = new THREE.Vector3(wpArr[idx]!.x, 0, wpArr[idx]!.z);
        }
      } else {
        p.walkPath = undefined;
      }
      const dx = target.x - p.pos.x;
      const dz = target.z - p.pos.z;
      const d = Math.hypot(dx, dz);
      const walkSpeed = rushing ? 3.4 : p.slot === 1 ? 1.7 : 1.35;
      const step = holding ? d : Math.min(d, walkSpeed * dt);
      if (d > 1e-4) {
        p.pos.x += (dx / d) * step;
        p.pos.z += (dz / d) * step;
        if (!holding) p.root.rotation.y = Math.atan2(dx, dz);
        else p.root.rotation.y = Math.atan2(robotForP.x - p.pos.x, robotForP.z - p.pos.z);
      }
      // リトライ搬送中以外は、絶対にフィールド内へ入らない
      if (!robotForP.retry) this.keepOutsideFence(p.pos);
      p.root.position.copy(p.pos);
      const speed = step / Math.max(dt, 1e-4);
      const gait = Math.sin(this.clockT * (speed > 0.2 ? 12 : 2) + p.slot);
      p.body.position.y = 0.74 + Math.abs(gait) * 0.018;
      p.head.position.y = 1.06 + Math.abs(gait) * 0.012;
      p.legL.rotation.x = gait * 0.35;
      p.legR.rotation.x = -gait * 0.35;
      p.armL.rotation.x = -gait * 0.28;
      p.armR.rotation.x = gait * 0.28;
      const robot = p.team === 'blue' ? this.match.blue : this.match.red;
      const carrying = p.slot < 2 && !!robot.retry && (robot.retry.phase === 'lift' || robot.retry.phase === 'carry');
      if (carrying) {
        p.armL.rotation.z = p.slot === 0 ? -0.65 : 0.2;
        p.armR.rotation.z = p.slot === 0 ? -0.2 : 0.65;
      } else {
        p.armL.rotation.z = 0;
        p.armR.rotation.z = 0;
      }
      // Q8: 操縦者はスマホ両手持ちの固定ポーズ (リトライ搬送中は解除)
      if (p.slot === 1 && !robot.retry) {
        p.armL.rotation.x = -1.05;
        p.armR.rotation.x = -1.05;
        p.armL.rotation.z = 0.28;
        p.armR.rotation.z = -0.28;
        if (p.phone) p.phone.visible = true;
      } else if (p.phone) {
        p.phone.visible = false;
      }
      p.rag.visible = p.slot === 0 && !robot.retry && (robot.anim.grab >= 0 || robot.cur?.t === 'pickup');
      // Q21: 勝利チームの選手は飛び跳ねて喜ぶ
      if (winner === p.team) {
        p.root.position.y = Math.abs(Math.sin(this.clockT * 6.4 + p.slot * 1.3)) * 0.3;
        p.armL.rotation.z = -2.6 + Math.sin(this.clockT * 7 + p.slot) * 0.3;
        p.armR.rotation.z = 2.6 - Math.sin(this.clockT * 7 + p.slot) * 0.3;
      } else if (fin) {
        p.root.position.y = 0;
      }
    }
    this.updateCelebration();
  }

  /** Q21: 紙吹雪の生成と更新 */
  private updateCelebration(): void {
    const m = this.match;
    // 自動リピート: 競技終了(紙吹雪)から5秒喜んだらリセットして再開する
    if (this.celebrated && this.repeatT >= 0 && this.playing) {
      this.repeatT += 1 / 60;
      if (this.repeatT >= 5) {
        this.repeatT = -1;
        this.onAutoRepeat?.();
      }
    }
    if ((m.over || m.timeExpired) && !this.celebrated) {
      this.celebrated = true;
      this.repeatT = 0;
      const winner = m.score.blue > m.score.red ? 'blue' : m.score.red > m.score.blue ? 'red' : null;
      const n = this.confetti.count;
      this.confettiVel = new Float32Array(n * 3);
      this.confettiRot = new Float32Array(n * 4); // 角度3 + 角速度1(まとめ)
      const m4 = new THREE.Matrix4();
      const col = new THREE.Color();
      const base = winner === 'blue' ? [0x0068b7, 0xffd23e, 0xffffff, 0x7ab8ff]
        : winner === 'red' ? [0xd7000f, 0xffd23e, 0xffffff, 0xff8a8a]
        : [0xffd23e, 0xffffff, 0xc0c0c0, 0x7ab8ff];
      for (let i = 0; i < n; i++) {
        const x = (Math.random() - 0.5) * 10;
        const y = 6.5 + Math.random() * 3;
        const z = (Math.random() - 0.5) * 10;
        m4.makeTranslation(x, y, z);
        this.confetti.setMatrixAt(i, m4);
        col.set(base[i % base.length]!).offsetHSL(0, 0, (Math.random() - 0.5) * 0.1);
        this.confetti.setColorAt(i, col);
        this.confettiVel[i * 3] = (Math.random() - 0.5) * 0.8;
        this.confettiVel[i * 3 + 1] = -(0.5 + Math.random() * 0.7);
        this.confettiVel[i * 3 + 2] = (Math.random() - 0.5) * 0.8;
        this.confettiRot[i * 4] = Math.random() * Math.PI * 2;
        this.confettiRot[i * 4 + 1] = Math.random() * Math.PI * 2;
        this.confettiRot[i * 4 + 2] = Math.random() * Math.PI * 2;
        this.confettiRot[i * 4 + 3] = 2 + Math.random() * 5;
      }
      this.confetti.instanceMatrix.needsUpdate = true;
      if (this.confetti.instanceColor) this.confetti.instanceColor.needsUpdate = true;
      this.confetti.visible = true;
      this.confettiT = 0;
    }
    if (this.confettiT >= 0 && this.confettiVel && this.confettiRot) {
      const dt = 1 / 60;
      this.confettiT += dt;
      const m4 = new THREE.Matrix4();
      const q = new THREE.Quaternion();
      const e = new THREE.Euler();
      const v = new THREE.Vector3();
      const sc = new THREE.Vector3(1, 1, 1);
      for (let i = 0; i < this.confetti.count; i++) {
        this.confetti.getMatrixAt(i, m4);
        v.setFromMatrixPosition(m4);
        if (v.y > 0.03) {
          v.x += (this.confettiVel[i * 3]! + Math.sin(this.confettiT * 2 + i) * 0.25) * dt;
          v.y += this.confettiVel[i * 3 + 1]! * dt;
          v.z += this.confettiVel[i * 3 + 2]! * dt;
        }
        const w = this.confettiRot[i * 4 + 3]!;
        this.confettiRot[i * 4] += w * dt;
        this.confettiRot[i * 4 + 1] += w * 0.7 * dt;
        e.set(this.confettiRot[i * 4]!, this.confettiRot[i * 4 + 1]!, this.confettiRot[i * 4 + 2]!);
        q.setFromEuler(e);
        m4.compose(v, q, sc);
        this.confetti.setMatrixAt(i, m4);
      }
      this.confetti.instanceMatrix.needsUpdate = true;
      if (this.confettiT > 14) {
        this.confetti.visible = false;
        this.confettiT = -1;
      }
    }
  }

  private makeRagMesh(pr: Projectile): THREE.Mesh {
    const w = pr.superRag ? 0.6 : 0.3;
    const h = pr.superRag ? 0.4 : 0.2;
    const geo = new THREE.PlaneGeometry(w, h, 6, 4);
    const mesh = new THREE.Mesh(
      geo,
      new THREE.MeshLambertMaterial({
        map: pr.superRag ? this.ragTexS : this.ragTexN,
        side: THREE.DoubleSide,
      }),
    );
    mesh.userData.base = new Float32Array(
      (geo.getAttribute('position') as THREE.BufferAttribute).array,
    );
    mesh.userData.phase = Math.random() * 10;
    mesh.castShadow = true;
    return mesh;
  }

  private makeHungMesh(hr: HungRag): THREE.Object3D {
    const tex = hr.superRag ? this.ragTexS : this.ragTexN;
    const w = hr.superRag ? 0.5 : 0.3;
    const g = new THREE.Group();
    if (hr.kind === 'bar') {
      // 横棒に掛かった状態: テント状に2面
      const m1 = new THREE.Mesh(
        new THREE.PlaneGeometry(w, 0.16),
        new THREE.MeshLambertMaterial({ map: tex, side: THREE.DoubleSide }),
      );
      m1.rotation.x = -1.35;
      m1.position.z = 0.055;
      const m2 = m1.clone();
      m2.rotation.x = 1.35;
      m2.position.z = -0.055;
      g.add(m1, m2);
      g.position.set(hr.x, hr.y + 0.03, hr.z);
      g.rotation.y = hr.yaw;
    } else {
      // 床/棚/バケツ: くしゃっと着地した形 (頂点ノイズ)
      const geo = new THREE.PlaneGeometry(w, hr.superRag ? 0.35 : 0.2, 5, 4);
      const pos = geo.getAttribute('position') as THREE.BufferAttribute;
      for (let i = 0; i < pos.count; i++) {
        pos.setZ(i, Math.random() * 0.035);
        pos.setX(i, pos.getX(i) * (0.82 + Math.random() * 0.2));
        pos.setY(i, pos.getY(i) * (0.82 + Math.random() * 0.2));
      }
      geo.computeVertexNormals();
      const flat = new THREE.Mesh(
        geo,
        new THREE.MeshLambertMaterial({ map: tex, side: THREE.DoubleSide }),
      );
      flat.rotation.x = -Math.PI / 2;
      flat.rotation.z = hr.attachedTo ? 0 : hr.yaw;
      flat.castShadow = true;
      g.add(flat);
      if (!hr.attachedTo) g.position.set(hr.x, Math.max(hr.y, 0.015), hr.z);
    }
    return g;
  }

  // ---------------- 照準カメラ PiP + 照準オーバーレイ ----------------

  private renderPip(): void {
    const cam = this.pipCamera;
    const box = this.elems.pipBox;
    if (!this.showCam || !box.offsetParent) return;
    const cw = this.elems.container.clientWidth;
    const ch = this.elems.container.clientHeight;
    const r = box.getBoundingClientRect();
    const x = r.left;
    const y = ch - r.bottom;
    cam.aspect = r.width / r.height;
    cam.updateProjectionMatrix();
    this.updatePipCamera(cam);
    // 照準カメラ(没入ビュー)ではセンサーデバッグ表示を隠す: LiDAR点群(上下段)・ICP推定矢印/
    // 不確かさリング・相手推定ゴースト。いずれも毎フレーム更新ループで再設定されるため、ここで
    // PiP用に false にしても本画面(先に描画済み)には影響しない。
    this.lidarPts.visible = false;
    this.upperPts.visible = false;
    this.estArrow.visible = false;
    this.estRing.visible = false;
    this.ghost.visible = false;
    this.trueGhost.visible = false;
    this.renderer.setScissorTest(true);
    this.renderer.setScissor(x, y, r.width, r.height);
    this.renderer.setViewport(x, y, r.width, r.height);
    this.renderer.render(this.scene, cam);
    this.renderer.setScissorTest(false);
    this.renderer.setViewport(0, 0, cw, ch);
    this.drawAimOverlay(r.width, r.height);
  }

  private movingBucketWorld(r: { x: number; z: number; theta: number; bucketTopY: number }): THREE.Vector3 {
    const bx = 0.24;
    const bz = -0.24;
    const c = Math.cos(r.theta);
    const s = Math.sin(r.theta);
    return new THREE.Vector3(
      r.x + bx * c + bz * s,
      r.bucketTopY + 0.03,
      r.z - bx * s + bz * c,
    );
  }

  private updatePipCamera(cam: THREE.PerspectiveCamera): void {
    const player = this.match.player;
    const opponent = this.match.opponent;
    const yaw = player.aim
      ? Math.atan2(player.aim.target.x - player.x, player.aim.target.z - player.z)
      : player.theta + player.turretYaw;
    cam.position.set(
      player.x + Math.sin(yaw) * 0.43,
      1.24,
      player.z + Math.cos(yaw) * 0.43,
    );
    const focus = player.aim
      ? new THREE.Vector3(player.aim.target.x, player.aim.target.y, player.aim.target.z)
      : this.movingBucketWorld(opponent);
    cam.lookAt(focus);
  }

  private drawAimOverlay(w: number, h: number): void {
    const cv = this.elems.pipOverlay;
    if (cv.width !== w || cv.height !== h) {
      cv.width = w;
      cv.height = h;
    }
    const g = cv.getContext('2d');
    if (!g) return;
    g.clearRect(0, 0, w, h);
    // クロスヘア
    g.strokeStyle = 'rgba(255,255,255,.8)';
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(w / 2 - 14, h / 2);
    g.lineTo(w / 2 + 14, h / 2);
    g.moveTo(w / 2, h / 2 - 14);
    g.lineTo(w / 2, h / 2 + 14);
    g.stroke();

  }

}
