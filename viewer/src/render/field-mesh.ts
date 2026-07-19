import * as THREE from 'three';
import { FIELD, RED_SIDE, DIMS, mirror, REFLECTORS_RED, FLAG_CLOTH_DIR, type Vec2 } from '../config/field';
import { bannerTexture, noboriTexture, vinylTexture, woodTexture } from './textures';

export interface FieldRefs {
  group: THREE.Group;
  flagCloths: THREE.Mesh[];
  ragPiles: Record<'blue' | 'red', THREE.Mesh>;
  /** Q22: 大看板上のライブTV (残り時間・得点・チーム名を毎試合描き換える) */
  tvCtx: CanvasRenderingContext2D;
  tvTex: THREE.CanvasTexture;
}

function lambert(opts: THREE.MeshLambertMaterialParameters): THREE.MeshLambertMaterial {
  return new THREE.MeshLambertMaterial(opts);
}

/** 国技館風アリーナ: 四方の低い1階マス席 + 通路を挟んだ高い2階イス席 */
function buildGrandstands(g: THREE.Group): void {
  const masuTierMat = lambert({ color: 0x4b3f34 });
  const tamariMat = lambert({ color: 0x594437 });
  const tatamiMat = lambert({ color: 0xb7a46e });
  const cushionMat = lambert({ color: 0x7c2740 });
  const chairTierMat = lambert({ color: 0x303746 });
  const chairMat = lambert({ color: 0x6b1f34 });
  const railMat = lambert({ color: 0xb8bec8 });
  const aisleMat = lambert({ color: 0x242832 });
  interface Stand {
    axis: 'x' | 'z';
    sign: 1 | -1;
    length: number;
  }
  const stands: Stand[] = [
    { axis: 'z', sign: 1, length: 54 },
    { axis: 'z', sign: -1, length: 54 },
    { axis: 'x', sign: 1, length: 48 },
    { axis: 'x', sign: -1, length: 48 },
  ];

  const placeBox = (
    s: Stand,
    off: number,
    y: number,
    depth: number,
    height: number,
    length: number,
    mat: THREE.Material,
  ): THREE.Mesh => {
    const mesh = new THREE.Mesh(
      s.axis === 'x'
        ? new THREE.BoxGeometry(depth, height, length)
        : new THREE.BoxGeometry(length, height, depth),
      mat,
    );
    mesh.position.y = y;
    if (s.axis === 'x') mesh.position.x = s.sign * off;
    else mesh.position.z = s.sign * off;
    mesh.receiveShadow = true;
    g.add(mesh);
    return mesh;
  };

  const people: Array<[number, number, number]> = [];
  const cushions: Array<[number, number, number]> = [];
  const chairs: Array<[number, number, number]> = [];
  const chairBacks: Array<[number, number, number]> = [];

  for (const s of stands) {
    const edge = s.axis === 'x' ? FIELD.w / 2 : FIELD.l / 2;

    // 土俵周りの余白を意識し、フェンスから約6m離して最前列を置く。
    for (let i = 0; i < 6; i++) {
      const off = edge + 6.0 + i * 0.58;
      const y = 0.035;
      placeBox(s, off, y - 0.012, 0.46, 0.024, s.length * 0.72, tamariMat);
      const n = Math.floor((s.length * 0.68) / 0.74);
      for (let k = 0; k < n; k++) {
        if (Math.random() < 0.16) continue;
        const along = -s.length * 0.34 + (k + 0.5 + (Math.random() - 0.5) * 0.18) * 0.74;
        const x = s.axis === 'x' ? s.sign * off : along;
        const z = s.axis === 'x' ? along : s.sign * off;
        cushions.push([x, y + 0.006, z]);
        people.push([x, y + 0.18, z]);
      }
    }

    // 1階マス席: 15列。低くゆるい段差の箱席に座布団を並べる。
    for (let i = 0; i < 15; i++) {
      const off = edge + 10.2 + i * 0.62;
      const y = 0.09 + i * 0.075;
      const length = s.length * 0.86;
      placeBox(s, off, y - 0.04, 0.56, 0.08, length, masuTierMat);
      const n = Math.floor(length / 0.66);
      for (let k = 0; k < n; k++) {
        if (Math.random() < 0.18) continue;
        const along = -length / 2 + (k + 0.5 + (Math.random() - 0.5) * 0.16) * 0.66;
        const x = s.axis === 'x' ? s.sign * off : along;
        const z = s.axis === 'x' ? along : s.sign * off;
        cushions.push([x, y + 0.015, z]);
        people.push([x, y + 0.25, z]);
      }
    }

    // 1階後方通路と、2階席の手前の腰壁。
    placeBox(s, edge + 20.5, 0.012, 1.7, 0.024, s.length * 0.9, aisleMat);
    placeBox(s, edge + 22.6, 1.15, 0.18, 2.3, s.length * 0.92, railMat);

    // 2階イス席: 14列。国技館の2階席らしく高く、急勾配でフィールドを見下ろす。
    for (let i = 0; i < 14; i++) {
      const off = edge + 25.0 + i * 0.88;
      const y = 2.72 + i * 0.34;
      const length = s.length * 0.96;
      placeBox(s, off, y - 0.16, 0.78, 0.32, length, chairTierMat);
      const n = Math.floor(length / 0.54);
      for (let k = 0; k < n; k++) {
        if (Math.random() < 0.24) continue;
        const along = -length / 2 + (k + 0.5 + (Math.random() - 0.5) * 0.12) * 0.54;
        const x = s.axis === 'x' ? s.sign * off : along;
        const z = s.axis === 'x' ? along : s.sign * off;
        chairs.push([x, y + 0.02, z]);
        chairBacks.push([x, y + 0.24, z]);
        people.push([x, y + 0.43, z]);
      }
    }
  }

  const cushionIm = new THREE.InstancedMesh(new THREE.BoxGeometry(0.42, 0.035, 0.42), cushionMat, cushions.length);
  const chairIm = new THREE.InstancedMesh(new THREE.BoxGeometry(0.34, 0.06, 0.34), chairMat, chairs.length);
  const chairBackIm = new THREE.InstancedMesh(new THREE.BoxGeometry(0.34, 0.32, 0.045), chairMat, chairBacks.length);
  const bodyGeo = new THREE.BoxGeometry(0.24, 0.38, 0.2);
  const headGeo = new THREE.SphereGeometry(0.11, 8, 6);
  const bodyIm = new THREE.InstancedMesh(bodyGeo, new THREE.MeshLambertMaterial({ emissive: 0x1a1c22 }), people.length);
  const headIm = new THREE.InstancedMesh(
    headGeo,
    new THREE.MeshLambertMaterial({ color: 0xe8c39e, emissive: 0x2a2018 }),
    people.length,
  );
  const m4 = new THREE.Matrix4();
  const col = new THREE.Color();
  const palette = [0xd75a4a, 0x4a7dd7, 0x4ad78a, 0xd7c94a, 0xd77ab8, 0x8a6ad7, 0xcccccc, 0x556270];
  cushions.forEach(([x, y, z], i) => {
    m4.makeTranslation(x, y, z);
    cushionIm.setMatrixAt(i, m4);
  });
  chairs.forEach(([x, y, z], i) => {
    m4.makeTranslation(x, y, z);
    chairIm.setMatrixAt(i, m4);
  });
  chairBacks.forEach(([x, y, z], i) => {
    m4.makeTranslation(x, y, z);
    chairBackIm.setMatrixAt(i, m4);
  });
  people.forEach(([x, y, z], i) => {
    m4.makeTranslation(x, y, z);
    bodyIm.setMatrixAt(i, m4);
    col.set(palette[i % palette.length]!).offsetHSL(0, 0, (Math.random() - 0.5) * 0.15);
    bodyIm.setColorAt(i, col);
    m4.makeTranslation(x, y + 0.33, z);
    headIm.setMatrixAt(i, m4);
  });
  cushionIm.instanceMatrix.needsUpdate = true;
  chairIm.instanceMatrix.needsUpdate = true;
  chairBackIm.instanceMatrix.needsUpdate = true;
  bodyIm.instanceMatrix.needsUpdate = true;
  headIm.instanceMatrix.needsUpdate = true;
  g.add(cushionIm, chairIm, chairBackIm, bodyIm, headIm);
}

/** 大看板 (+x側) の裏のピットエリア: 壁なし、2m四方の木板だけを床に敷く */
function buildPits(gRoot: THREE.Group, wood?: THREE.Texture): void {
  // 内部座標は「+z 奥」で組み、全体を +90° 回して +x 側 (看板の裏) に配置する
  const g = new THREE.Group();
  g.rotation.y = Math.PI / 2;
  gRoot.add(g);
  const teamCols = [
    0x1f5fb0, 0xb52731, 0x2e8b57, 0xb8860b, 0x6a5acd, 0xcd5c5c, 0x20b2aa, 0xd2691e,
    0x4682b4, 0x9932cc, 0x8fbc8f, 0xbc8f8f, 0x5f9ea0, 0xdaa520, 0x708090, 0xc71585,
  ];
  const deskMat = lambert({ color: 0xcdb98e });
  const boardMat = wood ? lambert({ map: wood }) : lambert({ color: 0xb99462 });
  const tapeMat = lambert({ color: 0xffffff });
  const partMat = lambert({ color: 0x3a414d });
  const toolMat = lambert({ color: 0xc0392b });
  for (let i = 0; i < 16; i++) {
    const rowIdx = Math.floor(i / 8); // 0=手前列, 1=奥列
    const cx = -10.85 + (i % 8) * 3.1;
    const cz = FIELD.w / 2 + 16.3 + rowIdx * 3.25;
    const colTeam = teamCols[i]!;
    const board = new THREE.Mesh(new THREE.BoxGeometry(2, 0.035, 2), boardMat);
    board.position.set(cx, 0.018, cz);
    board.receiveShadow = true;
    g.add(board);
    const label = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.006, 0.035), tapeMat);
    label.position.set(cx, 0.04, cz - 0.88);
    g.add(label);
    // 作業机
    const desk = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.06, 0.7), deskMat);
    desk.position.set(cx, 0.74, cz + 0.38);
    desk.castShadow = true;
    g.add(desk);
    for (const [lx, lz] of [
      [-0.6, 0.25],
      [0.6, 0.25],
      [-0.6, 0.75],
      [0.6, 0.75],
    ] as const) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.72, 0.05), partMat);
      leg.position.set(cx + lx, 0.36, cz + lz);
      g.add(leg);
    }
    // 整備中のロボット (机上・チームカラー)
    const bot = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 0.5), lambert({ color: colTeam }));
    bot.position.set(cx - 0.15, 0.92, cz + 0.5);
    bot.castShadow = true;
    g.add(bot);
    const mast = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.5, 0.04), lambert({ color: 0x9aa0a8 }));
    mast.position.set(cx + 0.02, 1.3, cz + 0.62);
    g.add(mast);
    const bucket = new THREE.Mesh(
      new THREE.CylinderGeometry(0.1, 0.08, 0.18, 12, 1, true),
      new THREE.MeshPhongMaterial({ color: 0xdff1ff, transparent: true, opacity: 0.4, side: THREE.DoubleSide }),
    );
    bucket.position.set(cx + 0.02, 1.6, cz + 0.62);
    g.add(bucket);
    // 工具箱 + 予備タイヤ
    const tool = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.2, 0.2), toolMat);
    tool.position.set(cx + 0.45, 0.87, cz + 0.42);
    g.add(tool);
    const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.05, 12), lambert({ color: 0x2b2b31 }));
    tire.rotation.x = Math.PI / 2;
    tire.position.set(cx - 0.8, 0.1, cz - 0.3);
    g.add(tire);
    // 整備メンバー (2人)
    for (const off of [-0.35, 0.3]) {
      const person = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.5, 0.2), lambert({ color: colTeam }));
      person.position.set(cx + off, 0.95, cz - 0.15);
      person.castShadow = true;
      g.add(person);
      const head = new THREE.Mesh(new THREE.SphereGeometry(0.1, 8, 6), lambert({ color: 0xe8c39e }));
      head.position.set(cx + off, 1.32, cz - 0.15);
      g.add(head);
      // ヘルメット (安全対策 7.4.1)
      const helmet = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 8, 6, 0, Math.PI * 2, 0, Math.PI / 2),
        lambert({ color: 0xf5d327 }),
      );
      helmet.position.set(cx + off, 1.34, cz - 0.15);
      g.add(helmet);
    }
  }
}

export function buildField(): FieldRefs {
  const g = new THREE.Group();
  const flagCloths: THREE.Mesh[] = [];
  const wood = woodTexture();
  const woodMat = lambert({ map: wood });

  // 会場床 (フィールド外)。Q29: 「明るい黒」= チャコールグレー
  const hall = new THREE.Mesh(
    new THREE.PlaneGeometry(96, 90),
    lambert({ color: 0x3a3e48 }),
  );
  hall.rotation.x = -Math.PI / 2;
  hall.position.y = -0.02;
  hall.receiveShadow = true;
  g.add(hall);

  // フィールド床 (各陣で色味を変えたロンリウム)
  const mkFloor = (z0: number, z1: number, base: string): void => {
    const m = new THREE.Mesh(
      new THREE.PlaneGeometry(FIELD.w, Math.abs(z1 - z0)),
      lambert({ map: vinylTexture(base, '#9a9aa0') }),
    );
    m.rotation.x = -Math.PI / 2;
    m.position.set(0, 0, (z0 + z1) / 2);
    m.receiveShadow = true;
    g.add(m);
  };
  mkFloor(FIELD.podium.d / 2, FIELD.l / 2, '#edf8ff'); // 青陣 (ロンリウム3059CT)
  mkFloor(-FIELD.l / 2, -FIELD.podium.d / 2, '#ffebeb'); // 赤陣 (ロンリウム3022CT)

  // 教壇
  const podium = new THREE.Mesh(
    new THREE.BoxGeometry(FIELD.podium.w, FIELD.podium.h, FIELD.podium.d),
    woodMat,
  );
  podium.position.y = FIELD.podium.h / 2;
  podium.castShadow = podium.receiveShadow = true;
  g.add(podium);

  // フェンス (L字断面の木工フェンス塗装。公式平面図は赤陣=赤/青陣=青のチームカラー塗り分け)
  const fenceRedMat = lambert({ color: 0xff2600 });
  const fenceBlueMat = lambert({ color: 0x005bff });
  const mkFence = (w: number, d: number, x: number, z: number, mat: THREE.Material): void => {
    const f = new THREE.Mesh(new THREE.BoxGeometry(w, FIELD.fenceH, d), mat);
    f.position.set(x, FIELD.fenceH / 2, z);
    f.castShadow = true;
    g.add(f);
  };
  // 遠端 (赤陣 z<0 = 赤 / 青陣 z>0 = 青)
  mkFence(FIELD.w + 0.1, 0.05, 0, -FIELD.l / 2 - 0.03, fenceRedMat);
  mkFence(FIELD.w + 0.1, 0.05, 0, FIELD.l / 2 + 0.03, fenceBlueMat);
  // 側面は中央 (教壇) で赤/青に分割
  const sideLen = FIELD.l + 0.1;
  for (const sx of [-1, 1] as const) {
    const fx = sx * (FIELD.w / 2 + 0.03);
    mkFence(0.05, sideLen / 2, fx, -sideLen / 4, fenceRedMat); // 赤半分
    mkFence(0.05, sideLen / 2, fx, sideLen / 4, fenceBlueMat); // 青半分
  }

  // ===== 会場: スタートゾーン側 (+x の縁) に大看板、その裏にピット、他3方はアリーナ観客席 =====
  const banner = new THREE.Mesh(new THREE.PlaneGeometry(19, 4.6), lambert({ map: bannerTexture() }));
  banner.rotation.y = -Math.PI / 2; // フィールド (-x) 向き
  banner.position.set(FIELD.w / 2 + 2.0, 2.5, 0);
  g.add(banner);
  const bannerBack = new THREE.Mesh(new THREE.BoxGeometry(0.15, 4.6, 19), lambert({ color: 0x232833 }));
  bannerBack.position.set(FIELD.w / 2 + 2.08, 2.5, 0);
  g.add(bannerBack);

  // Q22: 大看板の上のライブTVスクリーン (会場ビジョン)。内容は three-scene が毎秒描き換える
  const tvCanvas = document.createElement('canvas');
  tvCanvas.width = 1024;
  tvCanvas.height = 256;
  const tvCtx = tvCanvas.getContext('2d');
  if (!tvCtx) throw new Error('2d context unavailable');
  tvCtx.fillStyle = '#05070c';
  tvCtx.fillRect(0, 0, 1024, 256);
  const tvTex = new THREE.CanvasTexture(tvCanvas);
  tvTex.colorSpace = THREE.SRGBColorSpace;
  const tvFrame = new THREE.Mesh(new THREE.BoxGeometry(0.22, 2.15, 8.3), lambert({ color: 0x14161c }));
  tvFrame.position.set(FIELD.w / 2 + 2.1, 5.85, 0);
  g.add(tvFrame);
  const tvScreen = new THREE.Mesh(
    new THREE.PlaneGeometry(8.0, 1.95),
    new THREE.MeshBasicMaterial({ map: tvTex, toneMapped: false }),
  );
  tvScreen.rotation.y = -Math.PI / 2;
  tvScreen.position.set(FIELD.w / 2 + 1.98, 5.85, 0);
  g.add(tvScreen);
  for (const sz of [-3.4, 3.4] as const) {
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.1, 0.12), lambert({ color: 0x2a2e38 }));
    post.position.set(FIELD.w / 2 + 2.1, 4.35, sz);
    g.add(post);
  }

  buildGrandstands(g);
  buildPits(g, wood);

  // カッティングシート (白)
  const sheetMat = lambert({ color: 0xffffff });
  const mkSheet = (shape: 'circle' | 'rect', p: Vec2, a: number, b: number): void => {
    const geo =
      shape === 'circle' ? new THREE.CircleGeometry(a, 32) : new THREE.PlaneGeometry(a, b);
    const m = new THREE.Mesh(geo, sheetMat);
    m.rotation.x = -Math.PI / 2;
    m.position.set(p.x, 0.004, p.z);
    g.add(m);
  };

  const ragPiles = {} as FieldRefs['ragPiles'];

  const buildSide = (sign: 1 | -1): void => {
    const team = sign === 1 ? 'blue' : 'red';
    const S = (p: Vec2): Vec2 => (sign === -1 ? p : mirror(p));
    const P = RED_SIDE;

    // 旗
    const f = S(P.flag);
    mkSheet('rect', f, 0.45, 0.45);
    const poleMat = lambert({ color: 0xf5f5f5 });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.025, 3.0, 12), poleMat);
    pole.position.set(f.x, 1.5, f.z);
    pole.castShadow = true;
    g.add(pole);
    // 横棒と布はポールから片側へ突き出す。図面精査: 旗だけ鏡映でなく 赤=+x / 青=-x 向き
    const cDir = FLAG_CLOTH_DIR[team];
    const bar = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.62, 10), poleMat);
    bar.rotation.z = Math.PI / 2;
    bar.position.set(f.x + cDir * 0.31, DIMS.flagBarY - 0.01, f.z);
    g.add(bar);
    // のぼり旗クロス (揺らぎアニメーション対象)
    const cloth = new THREE.Mesh(
      new THREE.PlaneGeometry(0.58, 1.78, 8, 12),
      new THREE.MeshLambertMaterial({
        map: noboriTexture(team === 'blue' ? 'blue' : 'red'),
        side: THREE.DoubleSide,
      }),
    );
    cloth.position.set(f.x + cDir * 0.31, DIMS.flagBarY - 0.92, f.z);
    cloth.castShadow = true;
    flagCloths.push(cloth);
    g.add(cloth);
    // 土台 390 角 (図面: 土台390角 > シート450角)。LiDAR下段(0.12m)で四角として観測されるよう
    // ペデスタル状の高さ(0.22m)を持たせる。上段(0.5m)では細いポールだけが見える。
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.39, 0.22, 0.39), lambert({ color: 0x4b4b52 }));
    base.position.set(f.x, 0.11, f.z);
    base.castShadow = true;
    g.add(base);

    // 固定バケツ + 台
    // 実物はエンテック PO-24A 透明ポリカバケツ (無色透明)
    const bucketMat = new THREE.MeshPhongMaterial({
      color: 0xf6fbff,
      transparent: true,
      opacity: 0.2,
      shininess: 110,
      specular: 0x99aabb,
      side: THREE.DoubleSide,
    });
    for (const key of ['b1', 'b2', 'b3'] as const) {
      const p = S(P[key]);
      const ped = DIMS.pedestal[key];
      mkSheet(key === 'b1' ? 'circle' : 'rect', p, key === 'b1' ? 0.17 : 0.36, 0.36);
      if (ped > 0) {
        const box = new THREE.Mesh(new THREE.BoxGeometry(0.3, ped, 0.3), woodMat);
        box.position.set(p.x, ped / 2, p.z);
        box.castShadow = true;
        g.add(box);
      }
      const bucket = new THREE.Mesh(
        new THREE.CylinderGeometry(DIMS.bucketRimR, 0.108, DIMS.bucketH, 24, 1, true),
        bucketMat,
      );
      bucket.position.set(p.x, ped + DIMS.bucketH / 2, p.z);
      g.add(bucket);
      const rim = new THREE.Mesh(
        new THREE.TorusGeometry(DIMS.bucketRimR, 0.008, 8, 24),
        lambert({ color: 0xffffff }),
      );
      rim.rotation.x = Math.PI / 2;
      rim.position.set(p.x, ped + DIMS.bucketH, p.z);
      g.add(rim);
      // 錘
      const weight = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.05, 12), lambert({ color: 0x666670 }));
      weight.position.set(p.x, ped + 0.03, p.z);
      g.add(weight);
    }

    // 机 (ゴール2台 + 補充スポット + コントロールステーション)。x=450/z=650 (公式図)
    const mkDesk = (p: Vec2, kind: 'goal' | 'resup' | 'control'): void => {
      if (kind === 'goal') mkSheet('rect', p, 0.71, 0.51); // 机下シート W710(x)×D510(z)
      const top = new THREE.Mesh(
        new THREE.BoxGeometry(DIMS.desk.wx, 0.03, DIMS.desk.wz),
        woodMat,
      );
      top.position.set(p.x, DIMS.desk.h, p.z);
      top.castShadow = true;
      g.add(top);
      const legMat = lambert({ color: 0x5d6068 });
      for (const [lx, lz] of [
        [-0.29, -0.19],
        [0.29, -0.19],
        [-0.29, 0.19],
        [0.29, 0.19],
      ] as const) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.03, DIMS.desk.h, 0.03), legMat);
        leg.position.set(p.x + lx, DIMS.desk.h / 2, p.z + lz);
        g.add(leg);
      }
      if (kind === 'goal') {
        const shelf = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.02, 0.38), legMat);
        shelf.position.set(p.x, DIMS.desk.shelfY - 0.12, p.z);
        g.add(shelf);
      }
      if (kind === 'control') {
        // Q27: ノートPC + WiFiルーター + Starlink mini (会場回線バックアップ)
        const pc = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.02, 0.22), lambert({ color: 0x2a2f3a }));
        pc.position.set(p.x, DIMS.desk.h + 0.03, p.z - 0.1);
        g.add(pc);
        const router = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.05, 0.1), lambert({ color: 0x3d4457 }));
        router.position.set(p.x - 0.11, DIMS.desk.h + 0.04, p.z + 0.18);
        g.add(router);
        for (let ai = 0; ai < 2; ai++) {
          const ant = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.09, 6), lambert({ color: 0x20242e }));
          ant.position.set(p.x - 0.15 + ai * 0.08, DIMS.desk.h + 0.11, p.z + 0.14);
          g.add(ant);
        }
        // Starlink mini: 白い平板アンテナを斜め上向きに立てる
        const dishG = new THREE.Group();
        const dish = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.016, 0.25), lambert({ color: 0xf2f4f6 }));
        dish.rotation.x = -0.55;
        dish.position.y = 0.13;
        const stand = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.02, 0.11, 8), lambert({ color: 0xd8dce2 }));
        stand.position.y = 0.055;
        dishG.add(dish, stand);
        dishG.position.set(p.x + 0.13, DIMS.desk.h + 0.015, p.z + 0.2);
        g.add(dishG);
      }
    };
    mkDesk(S(P.desk1), 'goal');
    mkDesk(S(P.desk2), 'goal');
    mkDesk(S(P.resup), 'resup');
    mkDesk(S(P.control), 'control');

    // Q2: コントロールステーション脇のフェンスに反射材マーカー柱 (LiDAR位置合わせ用)
    for (const rm of REFLECTORS_RED) {
      const q = S(rm);
      const pole = new THREE.Mesh(
        new THREE.CylinderGeometry(0.03, 0.03, 0.78, 10),
        lambert({ color: 0x30343c }),
      );
      pole.position.set(q.x, 0.39, q.z);
      pole.castShadow = true;
      g.add(pole);
      // 反射テープ帯 (下段0.12m / 上段0.50mのスキャン面に合わせて2本)
      for (const ty of [0.12, 0.5] as const) {
        const tape = new THREE.Mesh(
          new THREE.CylinderGeometry(0.033, 0.033, 0.09, 10),
          new THREE.MeshBasicMaterial({ color: 0xf8fbff }),
        );
        tape.position.set(q.x, ty, q.z);
        g.add(tape);
      }
    }

    // 補充スポットの雑巾の山
    const pile = new THREE.Mesh(
      new THREE.BoxGeometry(0.3, 0.05, 0.2),
      lambert({ color: 0xefe9dc }),
    );
    const rp = S(P.resup);
    pile.position.set(rp.x, DIMS.desk.h + 0.045, rp.z);
    pile.castShadow = true;
    g.add(pile);
    ragPiles[team] = pile;

    // 椅子 (障害物)
    const c = S(P.chair);
    mkSheet('rect', c, 0.42, 0.46);
    const seat = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.03, 0.4), woodMat);
    seat.position.set(c.x, DIMS.chair.seatY, c.z);
    seat.castShadow = true;
    g.add(seat);
    const back = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.32, 0.03), woodMat);
    back.position.set(c.x, DIMS.chair.seatY + 0.2, c.z + 0.185 * sign);
    g.add(back);
    const legMat2 = lambert({ color: 0x777c84 });
    for (const [lx, lz] of [
      [-0.15, -0.17],
      [0.15, -0.17],
      [-0.15, 0.17],
      [0.15, 0.17],
    ] as const) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, DIMS.chair.seatY, 8), legMat2);
      leg.position.set(c.x + lx, DIMS.chair.seatY / 2, c.z + lz);
      g.add(leg);
    }

    // スタートゾーン: メインフロアに濃色ロンリウムを埋め込み貼り (チームカラーのべた塗り)
    const st = S(P.start);
    const zone = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({ color: sign === 1 ? 0x0050e4 : 0xe82100 }),
    );
    zone.rotation.x = -Math.PI / 2;
    zone.position.set(st.x, 0.006, st.z);
    g.add(zone);
  };
  buildSide(1);
  buildSide(-1);

  return { group: g, flagCloths, ragPiles, tvCtx, tvTex };
}
