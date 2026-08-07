import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import { urdfDev } from './scripts/vite-urdf';

export default defineConfig({
  // ⚠ `urdfDev` は開発時だけ `/urdf/**` を cad/urdf から直に配る。
  //   public/ 経由にすると、サーバ起動より後に増えたファイルが
  //   index.html にフォールバックする（scripts/vite-urdf.ts の注記）。
  plugins: [svelte(), urdfDev()],
  // スマホ等の他デバイスからLAN経由で見られるように 0.0.0.0 で待ち受け
  server: { host: true },
  preview: { host: true },
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 1200,
  },
});
