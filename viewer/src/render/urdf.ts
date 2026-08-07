import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

/**
 * URDF (Unified Robot Description Format) を読んで Three.js の階層に組む。
 *
 * ⚠ **URDF は Z 上・X 前の右手系**、このビューアは **Y 上・Z 前**。
 *   変換は木の根に 1 枚だけ挟む（`AXIS_FIX`）。関節の軸も原点も URDF の
 *   数字をそのまま使えるようにするためで、リンクごとに座標を直すと
 *   軸の符号をどこかで必ず間違える。
 *
 * ⚠ メッシュは **mm**（`scale="0.001 …"` が URDF に書いてある）。
 *   スケールは visual ごとに読む。決め打ちにすると単位を変えた瞬間に壊れる。
 */

export type JointType = 'revolute' | 'continuous' | 'prismatic' | 'fixed' | 'floating' | 'planar';

export interface UrdfJoint {
  name: string;
  type: JointType;
  axis: THREE.Vector3;
  lower: number;
  upper: number;
  /** 関節原点（親リンク座標）。ここは動かない */
  frame: THREE.Group;
  /** 関節値で動く枠。子リンクはこの下 */
  moving: THREE.Group;
  value: number;
}

export interface UrdfModel {
  /** シーンに add するのはこれ。中は URDF 座標（Z 上）で、この根が Y 上へ直す */
  root: THREE.Group;
  links: Map<string, THREE.Group>;
  joints: Map<string, UrdfJoint>;
  rootLink: string;
  /** 三角形数（表示負荷の目安） */
  triangles: number;
  setJoint(name: string, value: number): void;
  /** 全関節を既定値（0 か下限）に戻す */
  reset(): void;
  dispose(): void;
}

/** URDF(Z 上・X 前) → ビューア(Y 上・Z 前)。(x,y,z)_urdf → (y,z,x)_viewer */
const AXIS_FIX = new THREE.Matrix4().set(
  0, 1, 0, 0,
  0, 0, 1, 0,
  1, 0, 0, 0,
  0, 0, 0, 1,
);

function rpyToQuat(r: number, p: number, y: number): THREE.Quaternion {
  // URDF の rpy は固定軸 XYZ = R = Rz(y)·Ry(p)·Rx(r)
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), r);
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), p);
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), y);
  return qz.multiply(qy).multiply(qx);
}

/**
 * URDF の色から既定のマテリアルを作る。
 *
 * ⚠ **明るさで金属かどうかを決めない。** 色だけ見て metalness を振ると、
 *   白い POM ブッシュが鏡になり、黒いモーターがゴムになる。呼ぶ側が
 *   `makeMaterial` で材質名を見て決めるのが本筋で、ここは無難な既定値。
 */
function defaultMaterial(rgba: [number, number, number, number]): THREE.Material {
  const [r, g, b, a] = rgba;
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(r, g, b),
    metalness: 0.45,
    roughness: 0.5,
    transparent: a < 1,
    opacity: a,
  });
}

/**
 * メッシュを取ってくる。**STL であることを確かめてから返す。**
 *
 * ⚠ 開発サーバ（Vite）も Cloudflare Pages も、**知らないパスには
 *   404 ではなく index.html を 200 で返す**（SPA フォールバック）。
 *   STLLoader はそれをバイナリ STL と信じて先頭 80 バイトの次を
 *   三角形数として読むので、`Invalid typed array length: 15312670587`
 *   （= 'c','o','l','e' の 4 バイト）という、原因の見当も付かない
 *   例外になる。実際これで 1 度潰した。
 *   ここで弾いて「どの URL が何を返したか」を言う。
 */
async function fetchMesh(url: string): Promise<ArrayBuffer> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`メッシュが読めない: ${url} (HTTP ${res.status})`);
  const buf = await res.arrayBuffer();
  const head = new Uint8Array(buf, 0, Math.min(64, buf.byteLength));
  const text = String.fromCharCode(...head).trimStart().toLowerCase();
  if (text.startsWith('<!doctype') || text.startsWith('<html') || text.startsWith('<?xml')) {
    const ct = res.headers.get('content-type') ?? '?';
    throw new Error(
      `メッシュのはずが HTML/XML が返ってきた: ${url} (${ct}, ${buf.byteLength}B)。` +
        `パスが違うと開発サーバは 404 ではなく index.html を 200 で返す。` +
        `URDF の filename は **URDF から見た相対パス**なので meshBase は URDF の置き場を渡すこと`,
    );
  }
  // バイナリ STL は 80B ヘッダ + 4B 三角形数 + 50B×N。長さが合わなければ ASCII STL
  if (buf.byteLength >= 84) {
    const n = new DataView(buf).getUint32(80, true);
    const expect = 84 + n * 50;
    const ascii = text.startsWith('solid');
    if (expect !== buf.byteLength && !ascii) {
      throw new Error(
        `STL として辻褄が合わない: ${url} (${buf.byteLength}B だが ` +
          `三角形数 ${n} なら ${expect}B のはず)`,
      );
    }
  }
  return buf;
}

function nums(s: string | null, fallback: number[]): number[] {
  if (!s) return fallback;
  const v = s.trim().split(/\s+/).map(Number);
  return v.length && v.every((n) => Number.isFinite(n)) ? v : fallback;
}

function applyOrigin(obj: THREE.Object3D, el: Element | null): void {
  const o = el?.querySelector(':scope > origin') ?? null;
  const [x, y, z] = nums(o?.getAttribute('xyz') ?? null, [0, 0, 0]);
  const [r, p, yw] = nums(o?.getAttribute('rpy') ?? null, [0, 0, 0]);
  obj.position.set(x, y, z);
  obj.quaternion.copy(rpyToQuat(r, p, yw));
}

export interface UrdfLoadOptions {
  /**
   * mesh の `filename` を解決する基準 URL（末尾 / つき）。
   * ⚠ URDF の `filename` は **URDF ファイルから見た相対パス**なので、ここは
   *   URDF が置いてあるディレクトリを渡すこと。メッシュの置き場を渡すと
   *   パスが二重になり、開発サーバは 404 の代わりに index.html を 200 で
   *   返すので、STL パーサが HTML を読んで意味不明な例外で落ちる。
   */
  meshBase: string;
  /** 既定マテリアル。渡すと URDF の色を**全部無視**して 1 色になる */
  material?: THREE.Material;
  /**
   * URDF の色からマテリアルを作る。材質ごとに質感を変えたいときに使う。
   * 省略すると金属寄りの標準マテリアルになる。
   */
  makeMaterial?: (rgba: [number, number, number, number], name: string) => THREE.Material;
  /** 読み込み進捗（0..1）。メッシュ単位で呼ばれる */
  onProgress?: (done: number, total: number) => void;
}

/** `<material>` の色を読む。visual 直下 → robot 直下の同名 の順で探す */
function readColor(
  v: Element,
  robot: Element,
): { rgba: [number, number, number, number]; name: string } | null {
  const m = v.querySelector(':scope > material');
  if (!m) return null;
  const name = m.getAttribute('name') ?? '';
  let c = m.querySelector(':scope > color');
  if (!c && name) {
    // 名前だけの参照。robot 直下の定義を引く
    for (const g of Array.from(robot.querySelectorAll(':scope > material'))) {
      if (g.getAttribute('name') === name) {
        c = g.querySelector(':scope > color');
        break;
      }
    }
  }
  const v4 = nums(c?.getAttribute('rgba') ?? null, [0.7, 0.7, 0.7, 1]);
  return { rgba: [v4[0]!, v4[1]!, v4[2]!, v4[3] ?? 1], name };
}

/**
 * URDF を読んで組み立てる。メッシュの読み込みが終わるまで待つので、
 * 返ってきた時点で見た目は完成している（**歯抜けの機体を一瞬出さない**）。
 */
export async function loadUrdf(url: string, opts: UrdfLoadOptions): Promise<UrdfModel> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`URDF が読めない: ${url} (${res.status})`);
  const doc = new DOMParser().parseFromString(await res.text(), 'application/xml');
  const err = doc.querySelector('parsererror');
  if (err) throw new Error(`URDF の XML が壊れている: ${err.textContent?.slice(0, 120)}`);
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('URDF に <robot> が無い');

  const loader = new STLLoader();
  const geoCache = new Map<string, Promise<THREE.BufferGeometry>>();
  const loadGeo = (file: string): Promise<THREE.BufferGeometry> => {
    let g = geoCache.get(file);
    if (!g) {
      g = fetchMesh(opts.meshBase + file.replace(/^\.?\//, '')).then((buf) => loader.parse(buf));
      geoCache.set(file, g);
    }
    return g;
  };

  const links = new Map<string, THREE.Group>();
  const joints = new Map<string, UrdfJoint>();
  const owned: (THREE.BufferGeometry | THREE.Material)[] = [];
  let triangles = 0;

  // --- リンク（visual だけ組む。collision は当たり判定側の仕事）-----------
  const linkEls = Array.from(robot.querySelectorAll(':scope > link'));
  const visualJobs: (() => Promise<void>)[] = [];
  for (const el of linkEls) {
    const name = el.getAttribute('name');
    if (!name) continue;
    const g = new THREE.Group();
    g.name = `link:${name}`;
    links.set(name, g);
    for (const v of Array.from(el.querySelectorAll(':scope > visual'))) {
      const mesh = v.querySelector(':scope > geometry > mesh');
      if (!mesh) continue; // 箱・円柱の visual はこの機体では使っていない
      const file = mesh.getAttribute('filename');
      if (!file) continue;
      const [sx, sy, sz] = nums(mesh.getAttribute('scale'), [1, 1, 1]);
      const col = readColor(v, robot);
      visualJobs.push(async () => {
        const geo = await loadGeo(file);
        const mtl =
          opts.material ??
          (col
            ? (opts.makeMaterial?.(col.rgba, col.name) ?? defaultMaterial(col.rgba))
            : new THREE.MeshStandardMaterial());
        if (!opts.material) owned.push(mtl);
        const m = new THREE.Mesh(geo, mtl);
        m.name = `visual:${name}:${col?.name ?? ''}`;
        m.castShadow = true;
        m.receiveShadow = true;
        applyOrigin(m, v);
        m.scale.set(sx, sy, sz);
        g.add(m);
        const pos = geo.getAttribute('position');
        if (pos) triangles += (geo.index ? geo.index.count : pos.count) / 3;
      });
    }
  }
  let done = 0;
  const total = visualJobs.length;
  opts.onProgress?.(0, total);
  await Promise.all(
    visualJobs.map(async (job) => {
      await job();
      opts.onProgress?.(++done, total);
    }),
  );
  for (const p of geoCache.values()) owned.push(await p);

  // --- 関節 ---------------------------------------------------------------
  const children = new Set<string>();
  for (const el of Array.from(robot.querySelectorAll(':scope > joint'))) {
    const name = el.getAttribute('name');
    const type = (el.getAttribute('type') ?? 'fixed') as JointType;
    const parent = el.querySelector(':scope > parent')?.getAttribute('link');
    const child = el.querySelector(':scope > child')?.getAttribute('link');
    if (!name || !parent || !child) continue;
    const pg = links.get(parent);
    const cg = links.get(child);
    if (!pg || !cg) throw new Error(`関節 ${name} の相手リンクが無い (${parent} → ${child})`);

    const frame = new THREE.Group();
    frame.name = `joint:${name}`;
    applyOrigin(frame, el);
    const moving = new THREE.Group();
    moving.name = `moving:${name}`;
    frame.add(moving);
    moving.add(cg);
    pg.add(frame);
    children.add(child);

    const [ax, ay, az] = nums(el.querySelector(':scope > axis')?.getAttribute('xyz') ?? null, [1, 0, 0]);
    const lim = el.querySelector(':scope > limit');
    joints.set(name, {
      name,
      type,
      axis: new THREE.Vector3(ax, ay, az).normalize(),
      lower: Number(lim?.getAttribute('lower') ?? (type === 'continuous' ? -Infinity : 0)),
      upper: Number(lim?.getAttribute('upper') ?? (type === 'continuous' ? Infinity : 0)),
      frame,
      moving,
      value: 0,
    });
  }

  const rootLink = Array.from(links.keys()).find((n) => !children.has(n));
  if (!rootLink) throw new Error('URDF の根リンクが見つからない（閉ループ？）');

  const root = new THREE.Group();
  root.name = 'urdf';
  root.matrixAutoUpdate = false;
  root.matrix.copy(AXIS_FIX);
  root.add(links.get(rootLink)!);

  const setJoint = (name: string, value: number): void => {
    const j = joints.get(name);
    if (!j) return;
    let v = value;
    if (j.type === 'revolute' || j.type === 'prismatic') {
      v = Math.min(j.upper, Math.max(j.lower, v));
    }
    j.value = v;
    if (j.type === 'prismatic') {
      j.moving.position.copy(j.axis).multiplyScalar(v);
    } else if (j.type === 'revolute' || j.type === 'continuous') {
      j.moving.quaternion.setFromAxisAngle(j.axis, v);
    }
  };

  const model: UrdfModel = {
    root,
    links,
    joints,
    rootLink,
    triangles: Math.round(triangles),
    setJoint,
    reset() {
      for (const j of joints.values()) {
        setJoint(j.name, j.type === 'revolute' && j.lower > 0 ? j.lower : 0);
      }
    },
    dispose() {
      for (const o of owned) o.dispose();
      root.traverse((n) => {
        const m = n as THREE.Mesh;
        if (m.isMesh && m.material && !Array.isArray(m.material)) m.material.dispose();
      });
    },
  };
  model.reset();
  return model;
}
