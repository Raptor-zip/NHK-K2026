import { createReadStream, existsSync, statSync } from 'node:fs';
import { dirname, join, normalize, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import type { Plugin } from 'vite';

/**
 * 開発中だけ `/urdf/**` を **`cad/urdf/` から毎回ディスクを読んで**返す。
 *
 * ⚠ **`public/` に置くと、サーバ起動より後に増えたファイルが 404 になる。**
 *   Vite は起動時に public のファイル一覧を作って持つので、`urdf:sync` で
 *   ファイルを増やしても、動いているサーバはそれを「public に無い」と見る。
 *   404 は SPA フォールバックで **index.html が 200 で返る**ため、
 *   STL パーサが HTML を読んで意味不明な例外で落ちる。
 *   実際これで 58 枚中 56 枚が HTML になった（サーバ起動 05:38 /
 *   メッシュ生成 05:49）。「同期したらサーバを再起動」で運用するのは
 *   忘れた方が負けるので、そもそも public を経由しない。
 *
 * ⚠ 本番ビルドは別。`npm run urdf:sync` で `public/urdf/` に置いてから
 *   `npm run build` する（dist に入れるにはビルド時に public にある必要がある）。
 */
export function urdfDev(): Plugin {
  const HERE = dirname(fileURLToPath(import.meta.url));
  const ROOT = resolve(HERE, '..', '..', 'cad', 'urdf');
  const TYPES: Record<string, string> = {
    '.urdf': 'application/xml',
    '.stl': 'model/stl',
    '.json': 'application/json',
  };
  return {
    name: 'k2026-urdf-dev',
    apply: 'serve',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = (req.url ?? '').split('?')[0]!;
        if (!url.startsWith('/urdf/')) return next();
        // パストラバーサル止め。`..` を潰してから root の下かを確かめる
        const rel = normalize(decodeURIComponent(url.slice('/urdf/'.length)));
        const file = join(ROOT, rel);
        if (!file.startsWith(ROOT + sep) || !existsSync(file) || !statSync(file).isFile()) {
          // ⚠ **index.html にフォールバックさせない。** 404 は 404 として返す。
          //   ここで next() すると SPA フォールバックが HTML を 200 で返し、
          //   「STL のはずが HTML」という分かりにくい失敗になる。
          res.statusCode = 404;
          res.setHeader('content-type', 'text/plain; charset=utf-8');
          res.end(`urdf: ${rel} が cad/urdf/ に無い`);
          return;
        }
        const ext = rel.slice(rel.lastIndexOf('.'));
        res.setHeader('content-type', TYPES[ext] ?? 'application/octet-stream');
        res.setHeader('content-length', String(statSync(file).size));
        // 開発中は毎回読み直す（CAD を出し直したら即反映される）
        res.setHeader('cache-control', 'no-cache');
        createReadStream(file).pipe(res);
      });
    },
  };
}
