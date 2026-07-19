import type { MatchSim } from '../sim/match';

/**
 * Web Audio 合成による効果音 (外部アセット不要)
 *  - 駆動モーター: 車速連動のホイーン音
 *  - 射出ローラー: 回転数連動の高周波ホイーン + 射出バースト
 *  - 観客: 常時ノイズ + 得点で歓声
 *  - ブザー: 補充・時間経過
 */
export class AudioEngine {
  private ctx: AudioContext | null = null;
  private master!: GainNode;
  private crowd!: GainNode;
  private drive!: { osc: OscillatorNode; gain: GainNode };
  private roller!: { osc: OscillatorNode; gain: GainNode };
  private noiseBuf!: AudioBuffer;
  private excitement = 0;
  private lastEvents = 0;
  private lastProj = 0;
  private enabled = true;

  /** ユーザー操作 (再生ボタン) の後に呼ぶこと */
  ensure(): void {
    if (this.ctx) {
      void this.ctx.resume();
      return;
    }
    const ctx = new AudioContext();
    this.ctx = ctx;
    this.master = ctx.createGain();
    this.master.gain.value = this.enabled ? 0.9 : 0;
    this.master.connect(ctx.destination);

    // ノイズバッファ
    const buf = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate);
    const d = buf.getChannelData(0);
    let last = 0;
    for (let i = 0; i < d.length; i++) {
      // ブラウンノイズ寄り
      last = (last + (Math.random() * 2 - 1) * 0.18) * 0.985;
      d[i] = last * 2.2;
    }
    this.noiseBuf = buf;

    // 観客 (常時ループ)
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.loop = true;
    const lp = ctx.createBiquadFilter();
    lp.type = 'lowpass';
    lp.frequency.value = 600;
    this.crowd = ctx.createGain();
    this.crowd.gain.value = 0.01; // アイドル時はほぼ無音 (T24)
    src.connect(lp).connect(this.crowd).connect(this.master);
    src.start();

    // 駆動モーター
    const dOsc = ctx.createOscillator();
    dOsc.type = 'triangle';
    dOsc.frequency.value = 70;
    const dGain = ctx.createGain();
    dGain.gain.value = 0;
    dOsc.connect(dGain).connect(this.master);
    dOsc.start();
    this.drive = { osc: dOsc, gain: dGain };

    // 射出ローラー (ブラシレスホイーン)
    const rOsc = ctx.createOscillator();
    rOsc.type = 'sawtooth';
    rOsc.frequency.value = 200;
    const rGain = ctx.createGain();
    rGain.gain.value = 0;
    const hp = ctx.createBiquadFilter();
    hp.type = 'highpass';
    hp.frequency.value = 300;
    rOsc.connect(hp).connect(rGain).connect(this.master);
    rOsc.start();
    this.roller = { osc: rOsc, gain: rGain };
  }

  setEnabled(on: boolean): void {
    this.enabled = on;
    if (!this.ctx) return;
    this.master.gain.value = on ? 0.9 : 0;
    if (!on) {
      // 即時に無音化 (歓声の余韻・連続音も含めて確実に止める)
      this.excitement = 0;
      this.crowd.gain.value = 0.008;
      this.drive.gain.gain.value = 0;
      this.roller.gain.gain.value = 0;
    }
  }

  private burst(freq: number, dur: number, gain: number, type: 'noise' | 'tone' = 'noise'): void {
    const ctx = this.ctx;
    if (!ctx) return;
    const g = ctx.createGain();
    g.gain.setValueAtTime(gain, ctx.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + dur);
    if (type === 'noise') {
      const s = ctx.createBufferSource();
      s.buffer = this.noiseBuf;
      s.playbackRate.value = 0.6 + Math.random() * 0.8;
      const bp = ctx.createBiquadFilter();
      bp.type = 'bandpass';
      bp.frequency.value = freq;
      bp.Q.value = 0.8;
      s.connect(bp).connect(g).connect(this.master);
      s.start(ctx.currentTime, Math.random());
      s.stop(ctx.currentTime + dur + 0.05);
    } else {
      const o = ctx.createOscillator();
      o.type = 'square';
      o.frequency.value = freq;
      o.connect(g).connect(this.master);
      o.start();
      o.stop(ctx.currentTime + dur);
    }
  }

  /** 原点出し時の起動音 (DJIドローンの起動音のような上昇するベル風アルペジオ) */
  private djiStartup(): void {
    const ctx = this.ctx;
    if (!ctx) return;
    const notes = [523.25, 659.25, 783.99, 1046.5, 1318.5]; // C5 E5 G5 C6 E6
    notes.forEach((f, i) => {
      const t0 = ctx.currentTime + i * 0.11;
      const o = ctx.createOscillator();
      o.type = 'triangle';
      o.frequency.setValueAtTime(f, t0);
      // 軽いビブラート
      const lfo = ctx.createOscillator();
      lfo.frequency.value = 6;
      const lfoG = ctx.createGain();
      lfoG.gain.value = f * 0.006;
      lfo.connect(lfoG).connect(o.frequency);
      const g = ctx.createGain();
      g.gain.setValueAtTime(0, t0);
      g.gain.linearRampToValueAtTime(0.16, t0 + 0.012);
      g.gain.exponentialRampToValueAtTime(0.0008, t0 + 0.4);
      o.connect(g).connect(this.master);
      o.start(t0);
      o.stop(t0 + 0.45);
      lfo.start(t0);
      lfo.stop(t0 + 0.45);
    });
    // 締めの上ハモ和音 (起動完了)
    const t1 = ctx.currentTime + notes.length * 0.11 + 0.05;
    for (const f of [1046.5, 1318.5, 1568]) {
      const o = ctx.createOscillator();
      o.type = 'triangle';
      o.frequency.value = f;
      const g = ctx.createGain();
      g.gain.setValueAtTime(0, t1);
      g.gain.linearRampToValueAtTime(0.09, t1 + 0.02);
      g.gain.exponentialRampToValueAtTime(0.0008, t1 + 0.6);
      o.connect(g).connect(this.master);
      o.start(t1);
      o.stop(t1 + 0.65);
    }
  }

  private cheer(big: boolean): void {
    const ctx = this.ctx;
    if (!ctx) return;
    this.excitement = Math.min(1, this.excitement + (big ? 1 : 0.35));
    // 「おおーっ」の声援うねり (帯域を人声域に絞ったノイズスウェル)
    if (big) {
      const g = ctx.createGain();
      g.gain.setValueAtTime(0.001, ctx.currentTime);
      g.gain.exponentialRampToValueAtTime(0.28, ctx.currentTime + 0.18);
      g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 1.4);
      const s = ctx.createBufferSource();
      s.buffer = this.noiseBuf;
      s.playbackRate.value = 0.5;
      const bp = ctx.createBiquadFilter();
      bp.type = 'bandpass';
      bp.frequency.setValueAtTime(320, ctx.currentTime);
      bp.frequency.exponentialRampToValueAtTime(520, ctx.currentTime + 0.5);
      bp.Q.value = 1.4;
      s.connect(bp).connect(g).connect(this.master);
      s.start(ctx.currentTime, Math.random());
      s.stop(ctx.currentTime + 1.6);
    }
    // 拍手: 短いノイズティックを散らす
    const n = big ? 34 : 10;
    for (let i = 0; i < n; i++) {
      const t = Math.random() * (big ? 0.9 : 0.5);
      const g = ctx.createGain();
      g.gain.setValueAtTime(0, ctx.currentTime + t);
      g.gain.linearRampToValueAtTime(big ? 0.12 : 0.06, ctx.currentTime + t + 0.005);
      g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + t + 0.05);
      const s = ctx.createBufferSource();
      s.buffer = this.noiseBuf;
      s.playbackRate.value = 1.6 + Math.random();
      const bp = ctx.createBiquadFilter();
      bp.type = 'bandpass';
      bp.frequency.value = 1500 + Math.random() * 1500;
      s.connect(bp).connect(g).connect(this.master);
      s.start(ctx.currentTime + t, Math.random());
      s.stop(ctx.currentTime + t + 0.1);
    }
  }

  /** Q21: 勝利ファンファーレ — 上昇アルペジオ + 三和音 (Web Audio合成) */
  private fanfare(): void {
    const ctx = this.ctx;
    if (!ctx) return;
    const notes = [523.25, 659.25, 783.99, 1046.5]; // C5 E5 G5 C6
    notes.forEach((f, i) => {
      const t0 = ctx.currentTime + i * 0.16;
      const o = ctx.createOscillator();
      o.type = 'triangle';
      o.frequency.value = f;
      const g = ctx.createGain();
      g.gain.setValueAtTime(0.0001, t0);
      g.gain.exponentialRampToValueAtTime(0.17, t0 + 0.03);
      g.gain.exponentialRampToValueAtTime(0.001, t0 + (i === notes.length - 1 ? 1.7 : 0.5));
      o.connect(g).connect(this.master);
      o.start(t0);
      o.stop(t0 + 1.9);
    });
    // 締めの三和音
    for (const f of [523.25, 659.25, 783.99]) {
      const t0 = ctx.currentTime + 0.64;
      const o = ctx.createOscillator();
      o.type = 'square';
      o.frequency.value = f;
      const g = ctx.createGain();
      g.gain.setValueAtTime(0.0001, t0);
      g.gain.exponentialRampToValueAtTime(0.05, t0 + 0.05);
      g.gain.exponentialRampToValueAtTime(0.001, t0 + 1.8);
      o.connect(g).connect(this.master);
      o.start(t0);
      o.stop(t0 + 2.0);
    }
  }

  update(m: MatchSim, dt: number, playing: boolean): void {
    const ctx = this.ctx;
    if (!ctx || !this.enabled) return;

    // 駆動音 (青ロボット基準 + 赤を弱く)
    const vb = Math.hypot(m.blue.vx, m.blue.vz);
    const vr = Math.hypot(m.red.vx, m.red.vz);
    const sp = Math.max(vb, vr * 0.5);
    this.drive.osc.frequency.value = 65 + sp * 160;
    // 停止中は完全無音 (アイドルノイズ対策 T24)
    this.drive.gain.gain.value = playing && sp > 0.06 ? Math.min(0.045, sp * 0.032) : 0;

    // ローラー音
    const rpm = Math.max(m.blue.rollerRpm, m.red.rollerRpm);
    this.roller.osc.frequency.value = 180 + rpm * 0.5;
    this.roller.gain.gain.value = playing ? Math.min(0.035, (rpm / 2500) * 0.035) : 0;

    // 射出検知 (弾数増加)
    const totalProj = m.projectiles.length;
    if (totalProj > this.lastProj) {
      this.burst(900, 0.12, 0.18); // シュッ
      this.burst(120, 0.18, 0.12); // 低音
    }
    this.lastProj = totalProj;

    // イベント → 歓声/ブザー
    while (this.lastEvents < m.events.length) {
      const e = m.events[this.lastEvents]!;
      this.lastEvents++;
      const isEnd =
        e.cls === 'sys' && (e.text.startsWith('競技終了') || e.text.startsWith('タイマー0:00'));
      if (isEnd) {
        // 終了ファンファーレは一度だけ鳴らす
        this.fanfare();
        this.cheer(true);
        this.cheer(true);
      } else if (!m.timeExpired) {
        // 0:00 以降のフリー走行中は歓声/効果音を出さない (鳴り続けない)
        if (e.cls === 'hit') this.cheer(e.text.includes('+100') || e.text.includes('+200'));
        else if (e.cls === 'opp' && e.text.includes('+')) this.cheer(false);
        else if (e.cls === 'ev' && e.text.includes('原点出し')) this.djiStartup();
        else if (e.cls === 'ev') this.cheer(true);
        else if (e.cls === 'miss') this.burst(500, 0.25, 0.05);
        else if (e.cls === 'sys' && e.text.startsWith('ブザー')) this.burst(880, 0.5, 0.12, 'tone');
      }
    }

    // 観客の熱量: 平常時ほぼ無音 → 得点で声援が立ち上がる
    this.excitement = Math.max(0, this.excitement - dt * 0.45);
    this.crowd.gain.value = 0.008 + this.excitement * 0.3;
  }
}
