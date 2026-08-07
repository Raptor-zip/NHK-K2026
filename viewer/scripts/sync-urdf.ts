/**
 * CAD が出した URDF とリンクメッシュを、**本番ビルド用に** public へ置く。
 *
 *     npm run urdf:sync     # npm run build が自動で呼ぶ
 *
 * ⚠ **開発中は要らない。** `vite.config.ts` の `urdfDev` プラグインが
 *   `/urdf/**` を `cad/urdf/` から毎回ディスクを読んで返すので、
 *   CAD を出し直したらリロードだけで反映される。
 *   public に置くやり方だと、**サーバ起動より後に増えたファイルが
 *   index.html にフォールバックする**（Vite は起動時に public の一覧を
 *   持つ）。実際それで 58 枚中 56 枚が HTML になった。
 *   本番ビルドは dist に入れる必要があるので、ここで public へ複製する。
 *
 * ⚠ **`viewer/public/urdf/` は生成物**なので git では追跡しない。
 *   CAD を直したら `cad/scripts/export_meshes.py` → `cad/urdf/tr_urdf.py`
 *   の順で回す（配置はビルドが面倒を見る）。
 */
import { cpSync, existsSync, mkdirSync, readdirSync, rmSync, statSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const SRC = resolve(HERE, '..', '..', 'cad', 'urdf');
const DST = resolve(HERE, '..', 'public', 'urdf');

function kb(p: string): string {
  return `${(statSync(p).size / 1024).toFixed(0)} KB`;
}

function main(): number {
  if (!existsSync(join(SRC, 'tr.urdf'))) {
    console.error(`URDF が無い: ${join(SRC, 'tr.urdf')}`);
    console.error('先に cad/urdf/tr_urdf.py を実行すること');
    return 1;
  }
  const meshDir = join(SRC, 'meshes');
  if (!existsSync(meshDir)) {
    console.error(`メッシュが無い: ${meshDir}`);
    console.error('先に cad/scripts/export_meshes.py を実行すること');
    return 1;
  }

  rmSync(DST, { recursive: true, force: true });
  mkdirSync(join(DST, 'meshes'), { recursive: true });
  cpSync(join(SRC, 'tr.urdf'), join(DST, 'tr.urdf'));
  console.log(`tr.urdf          ${kb(join(DST, 'tr.urdf'))}`);

  let total = 0;
  for (const f of readdirSync(meshDir).filter((n) => n.toLowerCase().endsWith('.stl')).sort()) {
    cpSync(join(meshDir, f), join(DST, 'meshes', f));
    total += statSync(join(DST, 'meshes', f)).size;
    console.log(`meshes/${f.padEnd(24)} ${kb(join(DST, 'meshes', f))}`);
  }
  console.log(`\n合計 ${(total / 1024 / 1024).toFixed(1)} MB → ${DST}`);
  return 0;
}

process.exit(main());
