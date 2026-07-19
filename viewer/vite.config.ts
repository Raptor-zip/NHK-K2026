import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  // スマホ等の他デバイスからLAN経由で見られるように 0.0.0.0 で待ち受け
  server: { host: true },
  preview: { host: true },
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 1200,
  },
});
