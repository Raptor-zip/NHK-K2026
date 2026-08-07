/**
 * URDF の `<collision>` から「ロボットの実体」を取り出す。
 *
 * LiDAR シムが相手機を**正方形で近似**していたのを、CAD の当たり判定そのものに
 * 置き換えるために使う。生成した表は `src/config/robot-shape.ts` に焼き込み、
 * `urdf:check` が毎回ここで作り直して一致を確かめる（URDF は生成物なので
 * git に入らない。表だけが版に残る）。
 *
 * ⚠ **`<visual>` ではなく `<collision>` を読む。** 見た目のメッシュは 5 万三角形
 *   あって、レイと総当たりできない。当たり判定の箱は CAD の `link_*()` の
 *   bbox から作ってあるので、外形として正しく、数も 40 個ほどで済む。
 */
import { readFileSync } from 'node:fs';

/** 直方体（ビューア座標・ロボット局所）。y は床から。 */
export interface ShapeBox {
  /** 左が + */
  x: number;
  /** 上が + = 床からの高さ */
  y: number;
  /** 前が + */
  z: number;
  sx: number;
  sy: number;
  sz: number;
  link: string;
}

interface Xyz {
  x: number;
  y: number;
  z: number;
}

const ZERO: Xyz = { x: 0, y: 0, z: 0 };

function triple(s: string | undefined, d = ZERO): Xyz {
  if (!s) return d;
  const v = s.trim().split(/\s+/).map(Number);
  return { x: v[0] ?? 0, y: v[1] ?? 0, z: v[2] ?? 0 };
}

function attr(tag: string, key: string): string | undefined {
  return new RegExp(`${key}="([^"]*)"`).exec(tag)?.[1];
}

/**
 * URDF の当たり判定を、**全関節 0** のときの base_link 座標での AABB として返す。
 *
 * ⚠ 関節を 0 に置いた姿勢で切っている。走行中に動くのは砲塔・グラバー・
 *   フォークだけで、どれもスキャン面（0.12 / 0.50 m）より上にあるので
 *   断面は変わらない。0.5m より下に動く部品を足したら、ここも作り直すこと。
 */
export function readRobotBoxes(urdfPath: string): ShapeBox[] {
  const xml = readFileSync(urdfPath, 'utf8');

  // 関節: 親→子の平行移動。可動関節の取付に回転があると AABB を平行移動で
  // 積めないので、そこで落とす（センサーフレームの yaw だけは持ち歩く）。
  const joints = new Map<string, { parent: string; xyz: Xyz; yaw: number; fixed: boolean }>();
  const jre = /<joint\s+name="([^"]+)"\s+type="([^"]+)"([\s\S]*?)<\/joint>/g;
  for (let m = jre.exec(xml); m; m = jre.exec(xml)) {
    const body = m[3] ?? '';
    const fixed = m[2] === 'fixed';
    const child = attr(/<child[^>]*\/>/.exec(body)?.[0] ?? '', 'link');
    const parent = attr(/<parent[^>]*\/>/.exec(body)?.[0] ?? '', 'link');
    const org = /<origin[^>]*\/>/.exec(body)?.[0] ?? '';
    const rpy = triple(attr(org, 'rpy'));
    if (Math.abs(rpy.x) + Math.abs(rpy.y) > 1e-9 || (!fixed && Math.abs(rpy.z) > 1e-9)) {
      throw new Error(`関節 ${m[1]} の取付に回転がある。readRobotBoxes は平行移動しか見ていない`);
    }
    if (child && parent) {
      joints.set(child, { parent, xyz: triple(attr(org, 'xyz')), yaw: rpy.z, fixed });
    }
  }

  const frameOf = (link: string): { p: Xyz; yaw: number } => {
    let p = { ...ZERO };
    let yaw = 0;
    for (let cur = link, guard = 0; guard < 32; guard++) {
      const j = joints.get(cur);
      if (!j) break;
      p = { x: p.x + j.xyz.x, y: p.y + j.xyz.y, z: p.z + j.xyz.z };
      yaw += j.yaw;
      cur = j.parent;
    }
    return { p, yaw };
  };

  const out: ShapeBox[] = [];
  const lre = /<link\s+name="([^"]+)">([\s\S]*?)<\/link>/g;
  for (let m = lre.exec(xml); m; m = lre.exec(xml)) {
    const link = m[1]!;
    const { p: base, yaw } = frameOf(link);
    if (Math.abs(yaw) > 1e-9 && /<collision>/.test(m[2] ?? '')) {
      throw new Error(`${link}: 回った枠の下に当たり判定がある。AABB を平行移動で積めない`);
    }
    const cre = /<collision>([\s\S]*?)<\/collision>/g;
    for (let c = cre.exec(m[2] ?? ''); c; c = cre.exec(m[2] ?? '')) {
      const body = c[1] ?? '';
      const org = /<origin[^>]*\/>/.exec(body)?.[0] ?? '';
      const p = triple(attr(org, 'xyz'));
      const rpy = triple(attr(org, 'rpy'));
      const box = /<box[^>]*\/>/.exec(body)?.[0];
      const cyl = /<cylinder[^>]*\/>/.exec(body)?.[0];
      let size: Xyz;
      if (box) {
        if (Math.abs(rpy.x) + Math.abs(rpy.y) + Math.abs(rpy.z) > 1e-6) {
          throw new Error(`${link}: 傾いた箱は AABB にできない（rpy=${attr(org, 'rpy')}）`);
        }
        size = triple(attr(box, 'size'));
      } else if (cyl) {
        const r = Number(attr(cyl, 'radius') ?? 0);
        const len = Number(attr(cyl, 'length') ?? 0);
        // 車輪は rpy="1.5708 0 0" で軸が Y。それ以外の向きは想定していない
        const axisY = Math.abs(Math.abs(rpy.x) - Math.PI / 2) < 1e-3;
        size = axisY ? { x: 2 * r, y: len, z: 2 * r } : { x: 2 * r, y: 2 * r, z: len };
      } else {
        continue; // mesh の当たり判定は使っていない
      }
      // URDF (x前, y左, z上) → ビューア (X左, Y上, Z前)
      out.push({
        x: base.y + p.y,
        y: base.z + p.z,
        z: base.x + p.x,
        sx: size.y,
        sy: size.z,
        sz: size.x,
        link,
      });
    }
  }
  return out;
}

/** LiDAR の取付（ビューア局所・m / rad）。走査面の高さは y そのもの。 */
export interface LidarMount {
  name: string;
  x: number;
  y: number;
  z: number;
  /** 車体前方から測った据付方位（左回りが +） */
  yaw: number;
}

/**
 * URDF の fixed 関節からセンサーの取付を読む。
 *
 * ⚠ **走査原点は車体中心ではない。** 前後の下段は 908mm 離れていて、
 *   中心から飛ばすと片方からしか見えない物（机の脚・椅子の脚）が
 *   両方から見えることになり、ICP が実機より当たりやすくなる。
 */
export function readLidarMounts(urdfPath: string): LidarMount[] {
  const xml = readFileSync(urdfPath, 'utf8');
  const out: LidarMount[] = [];
  const jre = /<joint\s+name="([^"]+)"\s+type="fixed"([\s\S]*?)<\/joint>/g;
  for (let m = jre.exec(xml); m; m = jre.exec(xml)) {
    const body = m[2] ?? '';
    const child = attr(/<child[^>]*\/>/.exec(body)?.[0] ?? '', 'link') ?? '';
    if (!/lidar/.test(child)) continue;
    const org = /<origin[^>]*\/>/.exec(body)?.[0] ?? '';
    const p = triple(attr(org, 'xyz'));
    const rpy = triple(attr(org, 'rpy'));
    // URDF (x前, y左, z上) → ビューア (X左, Y上, Z前)
    out.push({ name: child, x: p.y, y: p.z, z: p.x, yaw: rpy.z });
  }
  return out;
}

/** 生成した表を `src/config/robot-shape.ts` の中身にする */
export function renderModule(boxes: ShapeBox[], mounts: LidarMount[]): string {
  const n = (v: number): string => (Math.round(v * 1e6) / 1e6).toString();
  const rows = boxes
    .map(
      (b) =>
        `  { x: ${n(b.x)}, y: ${n(b.y)}, z: ${n(b.z)}, ` +
        `sx: ${n(b.sx)}, sy: ${n(b.sy)}, sz: ${n(b.sz)}, link: '${b.link}' },`,
    )
    .join('\n');
  const mountRows = mounts
    .map((m) => `  { name: '${m.name}', x: ${n(m.x)}, y: ${n(m.y)}, z: ${n(m.z)}, yaw: ${n(m.yaw)} },`)
    .join('\n');
  return `/**
 * ロボットの実体（CAD の当たり判定そのもの）。**自動生成。手で書き換えない。**
 *
 *     npm run urdf:shape        # cad/urdf/tr.urdf から作り直す
 *
 * LiDAR シムはこの箱をスキャン面で切って相手機の断面にする。以前は 0.76m 角の
 * 正方形 1 つで近似していたので、実際には見えないはずの向きから見え、
 * 逆に机の脚のような細い物と区別が付かなかった。
 *
 * 座標はビューア局所（X 左 / Y 上＝床から / Z 前）で、単位は m。
 * 全関節 0 の姿勢で取ってある（動く部品はどれもスキャン面より上）。
 * ⚠ URDF は生成物で git に入らないので、**この表が版に残る唯一の実体**。
 *   \`npm run urdf:check\` が毎回 URDF から作り直して一致を確かめる。
 */
export interface ShapeBox {
  x: number;
  y: number;
  z: number;
  sx: number;
  sy: number;
  sz: number;
  link: string;
}

export const ROBOT_BOXES: readonly ShapeBox[] = [
${rows}
] as const;

/** LiDAR の取付（ビューア局所 m / rad）。走査面の高さは y そのもの。 */
export interface LidarMount {
  name: string;
  x: number;
  y: number;
  z: number;
  yaw: number;
}

/**
 * ⚠ **走査原点は車体中心ではない。** 前後の下段は 908mm 離れている。
 *   中心 1 点から飛ばすと、実機なら片方からしか見えない物が両方から
 *   見えることになり、自己位置推定が実機より当たってしまう。
 */
export const LIDAR_MOUNTS: readonly LidarMount[] = [
${mountRows}
] as const;

export function lidarMount(name: string): LidarMount {
  const m = LIDAR_MOUNTS.find((v) => v.name === name);
  if (!m) throw new Error(\`URDF に LiDAR フレーム \${name} が無い\`);
  return m;
}

/** 高さ h [m] でロボットを水平に切った断面（軸並行の長方形の集まり） */
export function sectionRects(h: number): { cx: number; cz: number; hw: number; hd: number }[] {
  const out: { cx: number; cz: number; hw: number; hd: number }[] = [];
  for (const b of ROBOT_BOXES) {
    if (h < b.y - b.sy / 2 || h > b.y + b.sy / 2) continue;
    out.push({ cx: b.x, cz: b.z, hw: b.sx / 2, hd: b.sz / 2 });
  }
  return out;
}
`;
}
