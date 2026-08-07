/**
 * `cad/urdf/tr.urdf` の当たり判定から `src/config/robot-shape.ts` を作り直す。
 *
 *     npm run urdf:shape
 *
 * CAD を直したら回す。ずれたままだと `npm run urdf:check` が落ちる。
 */
import { writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { readLidarMounts, readRobotBoxes, renderModule } from './urdf-shape';

const HERE = dirname(fileURLToPath(import.meta.url));
const URDF = resolve(HERE, '..', '..', 'cad', 'urdf', 'tr.urdf');
const OUT = resolve(HERE, '..', 'src', 'config', 'robot-shape.ts');

const boxes = readRobotBoxes(URDF);
const mounts = readLidarMounts(URDF);
writeFileSync(OUT, renderModule(boxes, mounts), 'utf8');

const byLink = new Map<string, number>();
for (const b of boxes) byLink.set(b.link, (byLink.get(b.link) ?? 0) + 1);
console.log(`${URDF}\n → ${OUT}`);
console.log(`当たり判定 ${boxes.length} 個 / ${byLink.size} リンク`);
for (const [k, v] of byLink) console.log(`  ${k.padEnd(14)} ${v}`);

console.log(`LiDAR 取付 ${mounts.length} 個`);
for (const m of mounts) {
  console.log(
    `  ${m.name.padEnd(16)} 局所 (X${m.x.toFixed(3)}, Y${m.y.toFixed(3)}, Z${m.z.toFixed(3)}) ` +
      `方位 ${((m.yaw * 180) / Math.PI).toFixed(0)}°`,
  );
}

for (const h of [...new Set(mounts.map((m) => m.y))].sort()) {
  const at = boxes.filter((b) => h >= b.y - b.sy / 2 && h <= b.y + b.sy / 2);
  const lo = { x: Infinity, z: Infinity };
  const hi = { x: -Infinity, z: -Infinity };
  for (const b of at) {
    lo.x = Math.min(lo.x, b.x - b.sx / 2);
    hi.x = Math.max(hi.x, b.x + b.sx / 2);
    lo.z = Math.min(lo.z, b.z - b.sz / 2);
    hi.z = Math.max(hi.z, b.z + b.sz / 2);
  }
  console.log(
    `  スキャン面 ${h.toFixed(2)}m: ${at.length} 個 ` +
      `外形 ${(hi.x - lo.x).toFixed(3)} x ${(hi.z - lo.z).toFixed(3)} m ` +
      `[${at.map((b) => b.link).join(',')}]`,
  );
}
