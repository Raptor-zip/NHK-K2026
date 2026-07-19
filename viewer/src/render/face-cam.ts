import * as THREE from 'three';
import type { FaceLandmarker as FaceLandmarkerType } from '@mediapipe/tasks-vision';

/**
 * インカメ(セルフィーカメラ)の顔を MediaPipe FaceLandmarker で切り出し、
 * CanvasTexture として供給する。重い依存は動的 import で遅延読込するため、
 * 顔モードを有効にしたときだけ MediaPipe 本体・WASM・モデルを取得する。
 *
 * プライバシー: 映像はブラウザ内でのみ処理し、どこにも送信しない。
 */

const WASM_CDN = 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.20/wasm';
const MODEL_URL =
  'https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task';

export class FaceCam {
  readonly texture: THREE.CanvasTexture;
  readonly canvas = document.createElement('canvas');
  private readonly ctx: CanvasRenderingContext2D;
  private readonly video = document.createElement('video');
  private landmarker: FaceLandmarkerType | null = null;
  private stream: MediaStream | null = null;
  private starting = false;
  ready = false;
  error: string | null = null;

  constructor() {
    this.canvas.width = 256;
    this.canvas.height = 256;
    this.ctx = this.canvas.getContext('2d')!;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.texture.premultiplyAlpha = true;
  }

  /** カメラ許可 → MediaPipe 初期化。失敗時は error にメッセージを残す。 */
  async start(): Promise<void> {
    if (this.ready || this.starting) return;
    this.starting = true;
    this.error = null;
    try {
      this.stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'user', width: 640, height: 480 },
        audio: false,
      });
      this.video.srcObject = this.stream;
      this.video.muted = true;
      this.video.playsInline = true;
      await this.video.play();
      const vision = await import('@mediapipe/tasks-vision');
      const fileset = await vision.FilesetResolver.forVisionTasks(WASM_CDN);
      // GPU デリゲートが使えない環境があるので、失敗したら CPU にフォールバック
      const make = (delegate: 'GPU' | 'CPU'): Promise<FaceLandmarkerType> =>
        vision.FaceLandmarker.createFromOptions(fileset, {
          baseOptions: { modelAssetPath: MODEL_URL, delegate },
          runningMode: 'VIDEO',
          numFaces: 1,
        });
      try {
        this.landmarker = await make('GPU');
      } catch (gpuErr) {
        console.warn('[faceCam] GPU デリゲート失敗、CPUで再試行', gpuErr);
        this.landmarker = await make('CPU');
      }
      this.ready = true;
      console.log('[faceCam] 準備完了');
    } catch (e) {
      this.error = e instanceof Error ? e.message : String(e);
      console.error('[faceCam] 初期化失敗:', this.error);
      this.stop();
    } finally {
      this.starting = false;
    }
  }

  /** 毎フレーム: 顔を検出し、鏡映した顔を楕円マスクで canvas へ描画する。 */
  update(nowMs: number): boolean {
    if (!this.ready || !this.landmarker || this.video.readyState < 2) return false;
    let res;
    try {
      res = this.landmarker.detectForVideo(this.video, nowMs);
    } catch {
      return false;
    }
    const lm = res?.faceLandmarks?.[0];
    const W = this.canvas.width;
    const H = this.canvas.height;
    const ctx = this.ctx;
    // 顔が取れないフレームは前回の描画を残す (点滅防止)
    if (!lm || lm.length === 0) return false;
    ctx.clearRect(0, 0, W, H);

    // 鏡映後の正規化空間 (x' = 1 - x) で顔の外接矩形を求める
    let minX = 1;
    let maxX = 0;
    let minY = 1;
    let maxY = 0;
    for (const p of lm) {
      const fx = 1 - p.x;
      if (fx < minX) minX = fx;
      if (fx > maxX) maxX = fx;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    }
    const padX = (maxX - minX) * 0.14;
    const padY = (maxY - minY) * 0.16;
    minX = Math.max(0, minX - padX);
    maxX = Math.min(1, maxX + padX);
    minY = Math.max(0, minY - padY * 1.6); // 額を広めに含める
    maxY = Math.min(1, maxY + padY * 0.6);

    const vW = this.video.videoWidth || 640;
    const vH = this.video.videoHeight || 480;
    const sx = (1 - maxX) * vW;
    const sw = (maxX - minX) * vW;
    const sy = minY * vH;
    const sh = (maxY - minY) * vH;
    if (sw < 4 || sh < 4) return false;

    // 顔クロップを水平反転して canvas 全面へ
    ctx.save();
    ctx.translate(W, 0);
    ctx.scale(-1, 1);
    ctx.drawImage(this.video, sx, sy, sw, sh, 0, 0, W, H);
    ctx.restore();

    // 楕円フェードで顔形に切り抜く (角を透明化)
    ctx.globalCompositeOperation = 'destination-in';
    ctx.save();
    ctx.translate(W / 2, H / 2);
    ctx.scale(0.9, 1.1);
    const g = ctx.createRadialGradient(0, 0, H * 0.18, 0, 0, H * 0.5);
    g.addColorStop(0, 'rgba(0,0,0,1)');
    g.addColorStop(0.72, 'rgba(0,0,0,1)');
    g.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = g;
    ctx.fillRect(-W, -H, W * 2, H * 2);
    ctx.restore();
    ctx.globalCompositeOperation = 'source-over';

    this.texture.needsUpdate = true;
    return true;
  }

  stop(): void {
    this.ready = false;
    try {
      this.landmarker?.close();
    } catch {
      /* noop */
    }
    this.landmarker = null;
    if (this.stream) {
      for (const t of this.stream.getTracks()) t.stop();
      this.stream = null;
    }
    this.video.srcObject = null;
  }
}
