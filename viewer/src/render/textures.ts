import * as THREE from 'three';

/** 実行時 Canvas 生成テクスチャ (外部アセット不要で見た目のリアリティを上げる) */

function canvas(w: number, h: number): [HTMLCanvasElement, CanvasRenderingContext2D] {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  const ctx = c.getContext('2d');
  if (!ctx) throw new Error('2d context unavailable');
  return [c, ctx];
}

function toTexture(c: HTMLCanvasElement, repeat?: [number, number]): THREE.CanvasTexture {
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  if (repeat) {
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(repeat[0], repeat[1]);
  }
  return t;
}

/** のぼり旗 (W600×H1800)。チームカラーの縦書きデザイン */
export function noboriTexture(team: 'red' | 'blue'): THREE.CanvasTexture {
  const [c, g] = canvas(256, 768);
  const main = team === 'red' ? '#d7000f' : '#0068b7';
  const dark = team === 'red' ? '#8e0009' : '#004077';
  const grad = g.createLinearGradient(0, 0, 256, 0);
  grad.addColorStop(0, main);
  grad.addColorStop(1, dark);
  g.fillStyle = grad;
  g.fillRect(0, 0, 256, 768);
  // 白帯
  g.fillStyle = '#ffffff';
  g.fillRect(18, 14, 220, 96);
  g.fillRect(200, 130, 44, 560);
  // ヘッダー
  g.fillStyle = dark;
  g.font = '900 34px "Noto Sans JP", sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('高専杯', 128, 46);
  g.font = '900 26px "Noto Sans JP", sans-serif';
  g.fillText('ROBOCON', 128, 84);
  // 縦書きタイトル
  const title = '雑巾投擲選手権';
  g.fillStyle = '#ffffff';
  g.font = '900 58px "Noto Sans JP", sans-serif';
  [...title].forEach((ch, i) => {
    g.fillText(ch, 110, 190 + i * 78);
  });
  g.fillStyle = dark;
  g.font = '900 30px "Noto Sans JP", sans-serif';
  [...'2026'].forEach((ch, i) => {
    g.fillText(ch, 222, 210 + i * 120);
  });
  return toTexture(c);
}

/** ロンリウム床 (つや消しビニル + つなぎ目テープ) */
export function vinylTexture(base: string, seam: string): THREE.CanvasTexture {
  const [c, g] = canvas(512, 512);
  g.fillStyle = base;
  g.fillRect(0, 0, 512, 512);
  // ノイズ
  for (let i = 0; i < 2600; i++) {
    const a = Math.random() * 0.05;
    g.fillStyle = `rgba(${Math.random() < 0.5 ? '255,255,255' : '0,0,0'},${a})`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 2, 2);
  }
  // Q23: 実フィールドに格子模様は無いため、つなぎ目の線は描かない (seam引数は互換のため残置)
  void seam;
  return toTexture(c, [3, 3]);
}

/** 木目 (教壇・机・台) */
export function woodTexture(light = '#c9a86a', dark = '#a5854c'): THREE.CanvasTexture {
  const [c, g] = canvas(256, 256);
  g.fillStyle = light;
  g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 22; i++) {
    const y = Math.random() * 256;
    g.strokeStyle = dark;
    g.globalAlpha = 0.12 + Math.random() * 0.25;
    g.lineWidth = 1 + Math.random() * 3;
    g.beginPath();
    g.moveTo(0, y);
    for (let x = 0; x <= 256; x += 32) {
      g.lineTo(x, y + Math.sin(x * 0.05 + i) * 4);
    }
    g.stroke();
  }
  g.globalAlpha = 1;
  return toTexture(c, [2, 2]);
}

/** 会場バックドロップ (スポンサーバナー風) */
export function bannerTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(2048, 512);
  const grad = g.createLinearGradient(0, 0, 0, 512);
  grad.addColorStop(0, '#b3000c');
  grad.addColorStop(1, '#7c0008');
  g.fillStyle = grad;
  g.fillRect(0, 0, 2048, 512);
  // 段ボールブロック風の壁 (2025会場オマージュ)
  for (let i = 0; i < 40; i++) {
    g.fillStyle = `rgba(255,255,255,${0.04 + Math.random() * 0.05})`;
    const w = 120 + Math.random() * 160;
    g.fillRect(Math.random() * 2048, Math.random() * 512, w, 60);
  }
  g.fillStyle = '#ffffff';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  const fitText = (text: string, basePx: number, y: number, maxW: number): void => {
    let px = basePx;
    do {
      g.font = `900 ${px}px "Noto Sans JP", sans-serif`;
      px -= 4;
    } while (g.measureText(text).width > maxW && px > 20);
    g.fillText(text, 1024, y);
  };
  fitText('アイデア対決・全国高等専門学校ロボットコンテスト2026', 88, 150, 1920);
  fitText('高専杯 雑巾投擲選手権', 170, 350, 1920);
  return toTexture(c);
}

/** 雑巾 (パイル地 + ステッチ) */
export function ragTexture(superRag: boolean): THREE.CanvasTexture {
  const [c, g] = canvas(128, 128);
  g.fillStyle = superRag ? '#f5d327' : '#efe9dc';
  g.fillRect(0, 0, 128, 128);
  for (let i = 0; i < 900; i++) {
    g.fillStyle = `rgba(120,110,90,${Math.random() * 0.15})`;
    g.fillRect(Math.random() * 128, Math.random() * 128, 2, 1);
  }
  g.strokeStyle = superRag ? '#c7a800' : '#c04040';
  g.lineWidth = 2;
  g.setLineDash([5, 4]);
  g.strokeRect(8, 8, 112, 112);
  g.beginPath();
  g.moveTo(8, 64);
  g.lineTo(120, 64);
  if (superRag) {
    g.moveTo(64, 8);
    g.lineTo(64, 120);
  }
  g.stroke();
  return toTexture(c);
}

/** マスコットの顔 */
export function mascotTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(128, 128);
  g.fillStyle = '#ffd75e';
  g.fillRect(0, 0, 128, 128);
  g.fillStyle = '#222';
  g.beginPath();
  g.arc(42, 52, 7, 0, Math.PI * 2);
  g.arc(86, 52, 7, 0, Math.PI * 2);
  g.fill();
  g.strokeStyle = '#222';
  g.lineWidth = 5;
  g.beginPath();
  g.arc(64, 74, 20, 0.15 * Math.PI, 0.85 * Math.PI);
  g.stroke();
  // ほっぺ
  g.fillStyle = 'rgba(255,110,110,.7)';
  g.beginPath();
  g.arc(30, 74, 8, 0, Math.PI * 2);
  g.arc(98, 74, 8, 0, Math.PI * 2);
  g.fill();
  return toTexture(c);
}

/** スタートゾーンなどの白線枠 */
export function zoneLineTexture(color: string): THREE.CanvasTexture {
  const [c, g] = canvas(128, 128);
  g.clearRect(0, 0, 128, 128);
  g.strokeStyle = color;
  g.lineWidth = 10;
  g.strokeRect(5, 5, 118, 118);
  return toTexture(c);
}
