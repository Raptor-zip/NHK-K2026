/**
 * CPU フォールバック描画 (WebGL が使えない環境用)
 *
 * ハードウェアアクセラレーションが無効・GPUプロセスが使えないブラウザでは three.js の
 * WebGLRenderer を生成できない。その場合でもシム自体は純粋な TypeScript (CPU) で動くので、
 * Canvas 2D による俯瞰 (トップダウン) 表示に切り替えて試合を見られるようにする。
 *
 * SimViewer と同じ公開APIを持つ (App.svelte から差し替え可能)。3D 専用の機能
 * (FPS/TPS/クレーン/顔カメラ等のカメラ操作) は no-op。手動操縦の走行・射出は
 * match.manual 経由なので 2D でもそのまま効く。
 */
import { MatchSim } from '../sim/match';
import { DIMS, FIELD, RED_SIDE, type TeamId } from '../config/field';
import type { AudioEngine } from './audio';
import type { HudSnapshot, PerfSnapshot, ViewerElems } from './three-scene';

const COLOR = {
  bg: '#14161d',
  floor: '#e9ecf2',
  podium: '#d8b380',
  blue: '#2f7fd8',
  red: '#d8452f',
  line: '#9aa4b4',
  prop: '#8d7350',
  bucket: '#c9cfda',
  rag: '#f2f4f8',
  text: '#e7ecf5',
} as const;

/** 赤陣の配置を z 反転すると青陣になる (フィールドは鏡映対称) */
function sideOf(team: TeamId): typeof RED_SIDE {
  if (team === 'red') return RED_SIDE;
  const m = {} as Record<string, { x: number; z: number }>;
  for (const [k, v] of Object.entries(RED_SIDE)) m[k] = { x: v.x, z: -v.z };
  return m as unknown as typeof RED_SIDE;
}

export class Canvas2DViewer {
  match: MatchSim;
  playing = false;
  speed = 1;
  // 3D 専用のトグル群 (2Dでは効かないが、App 側から同じように書き込まれる)
  showLidar = true;
  showCam = true;
  showPath = true;
  fpsActive = false;
  tpsActive = false;
  playerWalkActive = false;
  craneControlActive = false;
  multiCamActive = false;
  ragCamActive = false;
  faceCamActive = false;
  teamNames = { blue: '青チーム', red: '赤チーム' };
  audio: AudioEngine | null = null;
  onHud: ((s: HudSnapshot) => void) | null = null;
  onPerf: ((s: PerfSnapshot) => void) | null = null;
  onToast: ((msg: string, ok: boolean) => void) | null = null;
  onAutoRepeat: (() => void) | null = null;

  private elems: ViewerElems;
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private raf = 0;
  private lastNow = 0;
  /** 現在の ワールド[m] → 画面[px] スケール (線幅をワールド単位に直すのに使う) */
  private scale = 1;
  private hudTimer = 0;
  private perfWindow = 0;
  private perfFrames = 0;
  private perfFrameMs = 0;
  private perfSimMs = 0;
  private perfRenderMs = 0;
  private perfMaxFrameMs = 0;
  /** 試合終了→自動リセットまでの待ち (SimViewer と同じく5秒) */
  private celebrated = false;
  private repeatT = -1;

  constructor(elems: ViewerElems, match: MatchSim) {
    this.elems = elems;
    this.match = match;
    this.canvas = document.createElement('canvas');
    this.canvas.style.width = '100%';
    this.canvas.style.height = '100%';
    this.canvas.style.display = 'block';
    const ctx = this.canvas.getContext('2d');
    if (!ctx) throw new Error('Canvas 2D コンテキストを生成できませんでした');
    this.ctx = ctx;
    elems.container.appendChild(this.canvas);
    this.resize();
    addEventListener('resize', this.onResize);
    this.lastNow = performance.now();
    this.loop(this.lastNow);
  }

  // ---- SimViewer 互換 API (3D 専用機能は no-op) ----
  cameraPreset(_which: 'broadcast' | 'onboard' | 'top'): void {}
  setPlayerWalk(_f: number, _s: number): void {}
  triggerPlayerJump(): void {}
  lookPlayer(_mx: number, _my: number): void {}
  setCraneInput(_pan: number, _lift: number): void {}
  lookCrane(_mx: number, _my: number): void {}
  craneZoom(_deltaY: number): void {}
  followPointerDown(): void {}
  followPointerUp(): void {}
  followPointerMove(_mx: number, _my: number): void {}
  followWheel(_deltaY: number): void {}
  setTpsCursor(_clientX: number, _clientY: number): void {}

  /** 手動照準 (砲塔ヨー/ピッチ) は 2D でもそのまま効く */
  addAim(dyaw: number, dpitch: number): void {
    const m = this.match.manual;
    m.aimYaw += dyaw;
    while (m.aimYaw > Math.PI) m.aimYaw -= Math.PI * 2;
    while (m.aimYaw < -Math.PI) m.aimYaw += Math.PI * 2;
    m.aimPitch = Math.min(1.2, Math.max(0.03, m.aimPitch + dpitch));
  }

  tpsFire(): { ok: boolean; reason?: string } {
    return this.match.manualFire();
  }

  resetMatch(match: MatchSim): void {
    this.match = match;
    this.playing = false;
    this.celebrated = false;
    this.repeatT = -1;
    this.pushHud(true);
  }

  dispose(): void {
    cancelAnimationFrame(this.raf);
    this.raf = 0;
    removeEventListener('resize', this.onResize);
    this.canvas.remove();
  }

  // ---- 内部 ----
  private onResize = (): void => {
    this.resize();
  };

  private resize(): void {
    const dpr = Math.min(devicePixelRatio, 2);
    this.canvas.width = Math.max(1, Math.floor(this.elems.container.clientWidth * dpr));
    this.canvas.height = Math.max(1, Math.floor(this.elems.container.clientHeight * dpr));
  }

  private loop = (now: number): void => {
    this.raf = requestAnimationFrame(this.loop);
    const frameStart = performance.now();
    const rawDt = (now - this.lastNow) / 1000;
    const dt = Math.min(0.05, rawDt);
    this.lastNow = now;

    let simMs = 0;
    if (this.playing) {
      const simStart = performance.now();
      let sim = dt * this.speed;
      while (sim > 1e-6) {
        const h = Math.min(1 / 60, sim);
        this.match.step(h);
        sim -= h;
      }
      simMs = performance.now() - simStart;
    }
    this.audio?.update(this.match, dt, this.playing);

    const renderStart = performance.now();
    this.draw();
    const renderMs = performance.now() - renderStart;

    this.hudTimer -= dt;
    if (this.hudTimer <= 0) {
      this.hudTimer = 0.1;
      this.pushHud(false);
    }
    this.updateAutoRepeat(dt);
    this.recordPerf(rawDt, performance.now() - frameStart, simMs, renderMs);
  };

  /** 試合終了から5秒後に自動リセット (SimViewer と同じ挙動) */
  private updateAutoRepeat(dt: number): void {
    const m = this.match;
    if (this.celebrated && this.repeatT >= 0 && this.playing) {
      this.repeatT += dt;
      if (this.repeatT >= 5) {
        this.repeatT = -1;
        this.onAutoRepeat?.();
      }
    }
    if ((m.over || m.timeExpired) && !this.celebrated) {
      this.celebrated = true;
      this.repeatT = 0;
    }
  }

  private draw(): void {
    const g = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.fillStyle = COLOR.bg;
    g.fillRect(0, 0, W, H);

    // ワールド → 画面: x は右、z は下 (青陣 z>0 が下)。フィールド全体が収まる等倍スケール
    const margin = 24;
    const s = Math.min((W - margin * 2) / FIELD.w, (H - margin * 2) / FIELD.l);
    this.scale = s;
    g.translate(W / 2, H / 2);
    g.scale(s, s);
    g.lineWidth = 1.5 / s;

    const hx = FIELD.w / 2;
    const hz = FIELD.l / 2;
    g.fillStyle = COLOR.floor;
    g.fillRect(-hx, -hz, FIELD.w, FIELD.l);
    // 教壇 (中央帯)
    g.fillStyle = COLOR.podium;
    g.fillRect(-hx, -FIELD.podium.d / 2, FIELD.w, FIELD.podium.d);
    // 陣地の縁 (青 z>0 / 赤 z<0)
    g.strokeStyle = COLOR.blue;
    g.lineWidth = 4 / s;
    g.strokeRect(-hx, FIELD.podium.d / 2, FIELD.w, hz - FIELD.podium.d / 2);
    g.strokeStyle = COLOR.red;
    g.strokeRect(-hx, -hz, FIELD.w, hz - FIELD.podium.d / 2);
    g.lineWidth = 1.5 / s;

    for (const team of ['red', 'blue'] as const) this.drawProps(team);
    for (const r of this.match.hung) {
      g.fillStyle = COLOR.rag;
      g.strokeStyle = COLOR.line;
      g.beginPath();
      g.rect(r.x - 0.08, r.z - 0.08, 0.16, 0.16);
      g.fill();
      g.stroke();
    }
    this.drawRobot('red');
    this.drawRobot('blue');
    for (const p of this.match.projectiles) {
      // 高さ y を円の大きさで表現 (高いほど大きく)
      const rr = 0.07 + Math.max(0, p.y) * 0.03;
      g.fillStyle = p.team === 'blue' ? COLOR.blue : COLOR.red;
      g.globalAlpha = 0.85;
      g.beginPath();
      g.arc(p.x, p.z, rr, 0, Math.PI * 2);
      g.fill();
      g.globalAlpha = 1;
    }
  }

  /** 競技物品 (バケツ・旗・机・椅子・スタート/補充/CS) を俯瞰で描く */
  private drawProps(team: TeamId): void {
    const g = this.ctx;
    const S = sideOf(team);
    const tint = team === 'blue' ? COLOR.blue : COLOR.red;

    for (const key of ['b1', 'b2', 'b3'] as const) {
      const p = S[key];
      g.fillStyle = COLOR.bucket;
      g.strokeStyle = tint;
      g.beginPath();
      g.arc(p.x, p.z, DIMS.bucketRimR, 0, Math.PI * 2);
      g.fill();
      g.stroke();
    }
    for (const key of ['desk1', 'desk2'] as const) {
      const p = S[key];
      g.fillStyle = COLOR.prop;
      g.fillRect(p.x - DIMS.desk.wx / 2, p.z - DIMS.desk.wz / 2, DIMS.desk.wx, DIMS.desk.wz);
    }
    const c = S.chair;
    g.fillStyle = COLOR.prop;
    g.fillRect(c.x - DIMS.chair.wx / 2, c.z - DIMS.chair.wz / 2, DIMS.chair.wx, DIMS.chair.wz);
    const rd = S.resup;
    g.fillRect(rd.x - DIMS.resupDesk.wx / 2, rd.z - DIMS.resupDesk.wz / 2, DIMS.resupDesk.wx, DIMS.resupDesk.wz);
    // 旗ポール (横棒を陣の向きに突き出す)
    const f = S.flag;
    g.strokeStyle = tint;
    g.lineWidth = 5 / this.scale;
    g.beginPath();
    g.moveTo(f.x - DIMS.flagBarHalfW, f.z);
    g.lineTo(f.x + DIMS.flagBarHalfW, f.z);
    g.stroke();
    g.fillStyle = tint;
    g.beginPath();
    g.arc(f.x, f.z, 0.08, 0, Math.PI * 2);
    g.fill();
    // スタートゾーン
    g.strokeStyle = tint;
    g.lineWidth = 1.5 / this.scale;
    g.setLineDash([0.12, 0.1]);
    g.strokeRect(S.start.x - 0.5, S.start.z - 0.5, 1.0, 1.0);
    g.setLineDash([]);
  }

  private drawRobot(team: TeamId): void {
    const g = this.ctx;
    const r = this.match[team];
    const color = team === 'blue' ? COLOR.blue : COLOR.red;
    g.save();
    g.translate(r.x, r.z);
    // 画面では z が下向きなので、ワールドのヨー θ (前方 = sinθ, cosθ) は -θ で対応する
    g.rotate(-r.theta);
    g.fillStyle = color;
    g.fillRect(-0.25, -0.25, 0.5, 0.5);
    // 前方マーカー (機首)
    g.fillStyle = COLOR.text;
    g.beginPath();
    g.moveTo(0, 0.42);
    g.lineTo(-0.12, 0.2);
    g.lineTo(0.12, 0.2);
    g.closePath();
    g.fill();
    // 砲塔の向き (車体相対ヨー)
    g.rotate(-r.turretYaw);
    g.strokeStyle = COLOR.text;
    g.lineWidth = 0.05;
    g.beginPath();
    g.moveTo(0, 0);
    g.lineTo(0, 0.6);
    g.stroke();
    g.restore();
  }

  private pushHud(force: boolean): void {
    if (!this.onHud) return;
    const m = this.match;
    this.onHud({
      t: m.t,
      remain: Math.max(0, 180 - m.t),
      over: m.over || m.timeExpired,
      score: { ...m.score },
      breakdown: m.breakdown,
      status: { blue: m.blue.status, red: m.red.status },
      events: force ? [...m.events] : m.events,
      metrics: m.metrics(),
      manualPower: m.manual.power,
      fireReady: m.manual.fireCooldown <= 0 && m.player.ammo > 0,
      held: {
        blue: m.blue.ammo,
        blueSuper: m.blue.superAmmo,
        red: m.red.ammo,
        redSuper: m.red.superAmmo,
      },
    });
  }

  private recordPerf(rawDt: number, frameMs: number, simMs: number, renderMs: number): void {
    this.perfFrames++;
    this.perfFrameMs += frameMs;
    this.perfSimMs += simMs;
    this.perfRenderMs += renderMs;
    this.perfMaxFrameMs = Math.max(this.perfMaxFrameMs, frameMs);
    this.perfWindow += rawDt;
    if (this.perfWindow < 0.5 || !this.onPerf) return;
    const n = Math.max(1, this.perfFrames);
    const frame = this.perfFrameMs / n;
    this.onPerf({
      fps: this.perfFrames / this.perfWindow,
      frameMs: frame,
      simMs: this.perfSimMs / n,
      visualMs: 0,
      renderMs: this.perfRenderMs / n,
      maxFrameMs: this.perfMaxFrameMs,
      budgetPct: (frame / 16.67) * 100,
      headroomMs: 16.67 - frame,
      drawCalls: 0,
      triangles: 0,
      points: 0,
    });
    this.perfWindow = 0;
    this.perfFrames = 0;
    this.perfFrameMs = 0;
    this.perfSimMs = 0;
    this.perfRenderMs = 0;
    this.perfMaxFrameMs = 0;
  }
}
