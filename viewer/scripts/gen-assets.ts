/**
 * favicon / apple-touch-icon / OGP画像 を public/ に生成する。
 *   npx tsx scripts/gen-assets.ts
 */
import { createCanvas, GlobalFonts, type SKRSContext2D } from '@napi-rs/canvas';
import { writeFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

GlobalFonts.registerFromPath('/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc', 'NotoBlack');
const outDir = fileURLToPath(new URL('../public/', import.meta.url));
mkdirSync(outDir, { recursive: true });

/** アイコン: 移動バケツに雑巾が飛び込む図案 */
function drawIcon(size: number): Buffer {
  const c = createCanvas(size, size);
  const g = c.getContext('2d');
  const s = size / 100;
  // 背景 (角丸)
  const r = 18 * s;
  g.beginPath();
  g.moveTo(r, 0);
  g.arcTo(size, 0, size, size, r);
  g.arcTo(size, size, 0, size, r);
  g.arcTo(0, size, 0, 0, r);
  g.arcTo(0, 0, size, 0, r);
  g.closePath();
  const bg = g.createLinearGradient(0, 0, size, size);
  bg.addColorStop(0, '#182036');
  bg.addColorStop(1, '#0d1220');
  g.fillStyle = bg;
  g.fill();
  // 赤青の帯
  g.fillStyle = '#d7000f';
  g.fillRect(0, 84 * s, 50 * s, 16 * s);
  g.fillStyle = '#0068b7';
  g.fillRect(50 * s, 84 * s, 50 * s, 16 * s);
  // バケツ (透明ポリカ風)
  g.strokeStyle = '#dff1ff';
  g.lineWidth = 5 * s;
  g.lineJoin = 'round';
  g.beginPath();
  g.moveTo(28 * s, 46 * s);
  g.lineTo(35 * s, 82 * s);
  g.lineTo(65 * s, 82 * s);
  g.lineTo(72 * s, 46 * s);
  g.closePath();
  g.globalAlpha = 0.25;
  g.fillStyle = '#dff1ff';
  g.fill();
  g.globalAlpha = 1;
  g.stroke();
  g.beginPath();
  g.ellipse(50 * s, 46 * s, 22 * s, 7 * s, 0, 0, Math.PI * 2);
  g.stroke();
  // 雑巾 (オレンジのはためく布)
  g.save();
  g.translate(63 * s, 22 * s);
  g.rotate(-0.5);
  g.fillStyle = '#ff9a3d';
  g.beginPath();
  g.moveTo(-16 * s, -8 * s);
  g.quadraticCurveTo(0, -16 * s, 16 * s, -6 * s);
  g.quadraticCurveTo(6 * s, 0, 16 * s, 8 * s);
  g.quadraticCurveTo(0, 14 * s, -16 * s, 6 * s);
  g.quadraticCurveTo(-8 * s, 0, -16 * s, -8 * s);
  g.closePath();
  g.fill();
  g.strokeStyle = '#c05a00';
  g.lineWidth = 1.6 * s;
  g.setLineDash([3 * s, 2.4 * s]);
  g.stroke();
  g.restore();
  // 軌跡
  g.strokeStyle = 'rgba(255,255,255,.5)';
  g.lineWidth = 2 * s;
  g.setLineDash([4 * s, 4 * s]);
  g.beginPath();
  g.moveTo(16 * s, 44 * s);
  g.quadraticCurveTo(30 * s, 8 * s, 56 * s, 18 * s);
  g.stroke();
  return c.toBuffer('image/png');
}

/** OGP 1200×630 */
function drawOg(): Buffer {
  const W = 1200;
  const H = 630;
  const c = createCanvas(W, H);
  const g = c.getContext('2d') as SKRSContext2D;
  const bg = g.createLinearGradient(0, 0, 0, H);
  bg.addColorStop(0, '#141a2c');
  bg.addColorStop(1, '#0b0f1a');
  g.fillStyle = bg;
  g.fillRect(0, 0, W, H);

  // 簡易フィールド俯瞰
  const fx = 700;
  const fy = 140;
  const fw = 420;
  const fh = 360;
  g.fillStyle = '#e6e8ee';
  g.fillRect(fx, fy, fw, fh);
  g.fillStyle = '#c9a86a';
  g.fillRect(fx, fy + fh / 2 - 11, fw, 22); // 教壇
  g.fillStyle = 'rgba(215,0,15,.14)';
  g.fillRect(fx, fy, fw, fh / 2 - 11);
  g.fillStyle = 'rgba(0,104,183,.14)';
  g.fillRect(fx, fy + fh / 2 + 11, fw, fh / 2 - 11);
  // ゴール類
  const bucket = (x: number, y: number): void => {
    g.strokeStyle = '#5a7c9e';
    g.lineWidth = 3;
    g.beginPath();
    g.arc(x, y, 12, 0, Math.PI * 2);
    g.stroke();
  };
  bucket(fx + 250, fy + 60);
  bucket(fx + 90, fy + 130);
  bucket(fx + 170, fy + 100);
  bucket(fx + 170, fy + 260);
  bucket(fx + 250, fy + 230);
  bucket(fx + 90, fy + 290);
  g.fillStyle = '#d7000f';
  g.fillRect(fx + 330, fy + 52, 10, 44); // 旗
  g.fillStyle = '#0068b7';
  g.fillRect(fx + 80, fy + 264, 10, 44);
  // ロボット
  g.fillStyle = '#0068b7';
  g.fillRect(fx + 195, fy + 235, 30, 30);
  g.fillStyle = '#d7000f';
  g.fillRect(fx + 195, fy + 95, 30, 30);
  // 雑巾軌道
  g.strokeStyle = '#ff9a3d';
  g.lineWidth = 4;
  g.setLineDash([8, 7]);
  g.beginPath();
  g.moveTo(fx + 210, fy + 235);
  g.quadraticCurveTo(fx + 280, fy + 90, fx + 337, fy + 60);
  g.stroke();
  g.setLineDash([]);

  // テキスト
  g.fillStyle = '#ffd23e';
  g.font = '900 34px NotoBlack';
  g.fillText('高専ロボコン2026', 70, 150);
  g.fillStyle = '#ffffff';
  g.font = '900 78px NotoBlack';
  g.fillText('高専杯', 70, 250);
  g.fillText('雑巾投擲選手権', 70, 345);
  g.fillStyle = '#9fb4c8';
  g.font = '900 36px NotoBlack';
  g.fillText('試合シミュレーター', 70, 420);
  g.font = '700 24px NotoBlack';
  g.fillStyle = '#68788c';
  g.fillText('3分間の試合再生 / 弾道・LiDAR・経路のシミュレーション', 70, 470);
  // 帯
  g.fillStyle = '#d7000f';
  g.fillRect(0, H - 14, W / 2, 14);
  g.fillStyle = '#0068b7';
  g.fillRect(W / 2, H - 14, W / 2, 14);
  return c.toBuffer('image/png');
}

writeFileSync(outDir + 'favicon-32.png', drawIcon(32));
writeFileSync(outDir + 'icon-192.png', drawIcon(192));
writeFileSync(outDir + 'apple-touch-icon.png', drawIcon(180));
writeFileSync(outDir + 'og.png', drawOg());
console.log('generated: favicon-32.png, icon-192.png, apple-touch-icon.png, og.png ->', outDir);
