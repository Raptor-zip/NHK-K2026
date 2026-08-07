<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import { get } from 'svelte/store';
  import { MatchSim } from './sim/match';
  import { SimViewer } from './render/three-scene';
  import { Canvas2DViewer } from './render/canvas2d-scene';
  import { AudioEngine } from './render/audio';
  import { hud, perf, prefs, teamNames } from './ui/hudStore';
  import ScoreBug from './ui/ScoreBug.svelte';
  import SidePanel from './ui/SidePanel.svelte';
  import Dock from './ui/Dock.svelte';
  import SettingsDrawer from './ui/SettingsDrawer.svelte';
  import PerfPanel from './ui/PerfPanel.svelte';
  import CommandPad from './ui/CommandPad.svelte';
  import { fade, fly } from 'svelte/transition';
  import { cubicOut } from 'svelte/easing';
  import { Spring, SPRING_UI, VelocityTracker, prefersReducedMotion } from './ui/spring';
  import {
    analyticsAvailable,
    getAnalyticsConsent,
    initAnalytics,
    setAnalyticsConsent,
    trackEvent,
  } from './analytics';
  import type { ControlCommand } from './sim/types';

  type AnalyticsChoice = 'granted' | 'denied';

  let container: HTMLDivElement;
  let pipBox: HTMLDivElement;
  let pipOverlay: HTMLCanvasElement;
  let viewer: SimViewer | Canvas2DViewer | null = null;
  let unsub: (() => void) | null = null;
  let unsubNames: (() => void) | null = null;
  // Q28: タッチ主体デバイス (スマホ/タブレット) 判定
  let isCoarse = false;
  let audioOn = false;
  let pointerLocked = false;
  // 操作フィードバック (撃てない理由などを必ず見せる)
  let toast = '';
  let toastOk = true;
  let toastTimer: ReturnType<typeof setTimeout> | null = null;
  let gaEnabled = false;
  let gaConsent: AnalyticsChoice | null = null;
  let drawerOpen = false;
  // WebGL コンテキストがリトライしても張れなかった (GPUプロセス不調・WebGL無効など)
  let glFailed = false;
  // CPU フォールバック中 (WebGL 不可 → Canvas 2D の俯瞰表示)
  let cpuMode = false;

  function showToast(msg: string, ok: boolean): void {
    toast = msg;
    toastOk = ok;
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(() => (toast = ''), 1400);
  }

  function newMatch(): MatchSim {
    const p = get(prefs);
    return new MatchSim({
      seed: Math.floor(Math.random() * 1e9) + 1,
      variant: p.variant,
      redVariant: p.redVariant,
      params: JSON.parse(JSON.stringify(p.params)),
      oppSkill: p.oppSkill,
      blueCycleScale: p.blueCycleScale,
      redCycleScale: p.redCycleScale,
      blueProbMul: p.blueProbMul,
      redProbMul: p.oppSkill * p.redProbMul,
      blueDriveMul: p.blueDriveMul,
      redDriveMul: p.redDriveMul,
      blueMuzzleY: p.blueMuzzleY,
      redMuzzleY: p.redMuzzleY,
      blueSwerve: p.blueSwerve,
      redSwerve: p.redSwerve,
      controlMode: p.controlMode,
      playerTeam: p.playerTeam,
    });
  }

  const audio = new AudioEngine();

  /**
   * WebGL 初期化をリトライしながら 3D Viewer を作る。
   *
   * リロード直後は Chrome の GPU プロセスがまだ用意できておらず、コンテキスト生成が
   * "BindToCurrentSequence failed" で落ちることが稀にある。数百ms 待って作り直せばまず通るので、
   * 一発で諦めずバックオフしながら再試行する。全滅した場合 (ハードウェアアクセラレーション無効・
   * WebGL 非対応環境) は null を返し、呼び出し側が CPU の 2D 描画へフォールバックする。
   */
  async function create3DViewer(): Promise<SimViewer | null> {
    const waits = [0, 200, 500, 1000, 2000];
    for (const ms of waits) {
      if (ms) await new Promise((r) => setTimeout(r, ms));
      try {
        return new SimViewer({ container, pipBox, pipOverlay }, newMatch());
      } catch (e) {
        console.warn('WebGL の初期化に失敗。リトライします', e);
      }
    }
    return null;
  }

  onMount(async () => {
    gaEnabled = analyticsAvailable();
    gaConsent = getAnalyticsConsent();
    initAnalytics();
    // ?cpu=1 で 3D を使わず CPU 2D 描画を強制できる (低スペック機での閲覧・動作確認用)
    const forceCpu = new URLSearchParams(location.search).has('cpu');
    let created: SimViewer | Canvas2DViewer | null = forceCpu ? null : await create3DViewer();
    if (!created) {
      // WebGL が使えない環境: シムは CPU で回るので、Canvas 2D の俯瞰表示に切り替える
      try {
        created = new Canvas2DViewer({ container, pipBox, pipOverlay }, newMatch());
        cpuMode = true;
        console.warn('WebGL が使えないため CPU 2D 描画へフォールバックしました');
      } catch (e) {
        console.error('2D 描画のフォールバックにも失敗しました', e);
      }
    }
    if (!created) {
      glFailed = true;
      return;
    }
    viewer = created;
    viewer.onHud = (s) => hud.set(s);
    viewer.onPerf = (s) => perf.set(s);
    viewer.onToast = (msg, ok) => showToast(msg, ok);
    viewer.onAutoRepeat = () => handleReset(); // 3分終了→5秒喜ぶ→自動リセット再開
    viewer.audio = audio;
    viewer.resetMatch(viewer.match);
    (window as unknown as { SIM: unknown }).SIM = viewer; // デバッグ用フック
    isCoarse = matchMedia('(pointer: coarse)').matches;
    unsub = prefs.subscribe((p) => {
      if (!viewer) return;
      viewer.playing = p.playing;
      viewer.speed = p.speed;
      // 没入/放送カメラモードでは LiDAR点群・敵推定BB・推定ポーズ・経路などのデバッグ表示を隠す
      const immersive =
        p.controlMode === 'multicam' ||
        p.controlMode === 'crane' ||
        p.controlMode === 'ragcam' ||
        p.controlMode === 'player';
      viewer.showLidar = p.showLidar && !immersive;
      viewer.showCam = p.showCam && p.controlMode !== 'fps';
      viewer.showPath = p.showPath && !immersive;
      viewer.fpsActive = p.controlMode === 'fps';
      viewer.tpsActive = p.controlMode === 'tps';
      viewer.playerWalkActive = p.controlMode === 'player';
      viewer.craneControlActive = p.controlMode === 'crane';
      viewer.multiCamActive = p.controlMode === 'multicam';
      viewer.ragCamActive = p.controlMode === 'ragcam';
      viewer.faceCamActive = p.faceCam;
      // 機体を CAD 実形状（URDF）で描くか。読み込み前は簡略形状のまま進む
      if ('setUrdfSkin' in viewer) (viewer as SimViewer).setUrdfSkin(p.cadRobot);
      // 操縦モードは試合リセット無しで反映 (手動↔自動の遷移のみ内部で処理)
      viewer.match.setControlMode(p.controlMode);
      audio.setEnabled(p.sound && audioOn);
      // Q12: パラメータドロワーの変更をリセット無しで進行中の試合へ即反映
      // (par の参照は sim 内部で共有されているため、中身だけ書き換える)
      const par = viewer.match.par;
      const np = p.params;
      Object.assign(par.drive, np.drive);
      Object.assign(par.swerveDrive, np.swerveDrive);
      Object.assign(par.turret, np.turret);
      Object.assign(par.shooter, np.shooter);
      Object.assign(par.pickup, np.pickup);
      Object.assign(par.probs, np.probs);
      Object.assign(par.rag, np.rag);
      Object.assign(par.superRag, np.superRag);
      Object.assign(par.defense, np.defense);
      par.flagCapacity = np.flagCapacity;
      par.flagFallPerSec = np.flagFallPerSec;
      par.ragHazard = np.ragHazard;
      par.avoidFloorRags = np.avoidFloorRags;
      par.aimSpreadDeg = np.aimSpreadDeg;
      // バケツ高さもリセット無しで即反映 (par + 各ロボットの物理値 + 描画は viewer 側で再構築)
      Object.assign(par.bucketTopY, np.bucketTopY);
      viewer.match.blue.bucketTopY = np.bucketTopY.blue;
      viewer.match.red.bucketTopY = np.bucketTopY.red;
      // 足回り(独ステ/メカナム)はリセット無しで即反映
      viewer.match.blue.swerve = p.blueSwerve;
      viewer.match.red.swerve = p.redSwerve;
      trackParamChanges(p); // パラメーター変更を GA へ (デバウンスで最終値のみ)
    });
    // Q22: チーム名を看板ライブTVへ連携
    unsubNames = teamNames.subscribe((v) => {
      if (viewer) viewer.teamNames = { ...v };
    });
    // 自動再生 (T7): シムは即開始。音はブラウザ制約で初回タップ後に有効化。
    // ただし同意ゲートが必要なときは、選択(許可/記録せず)を押すまで開始しない。
    if (!(gaEnabled && gaConsent === null)) {
      prefs.update((p) => ({ ...p, playing: true }));
      trackMatchStart(); // 初回試合の開始を計測 (同意ゲート不要時)
    }
    document.addEventListener('pointerlockchange', onPlChange);
  });

  onDestroy(() => {
    unsub?.();
    unsubNames?.();
    viewer?.dispose();
    document.removeEventListener('pointerlockchange', onPlChange);
    knobSpring.mx.stop();
    knobSpring.my.stop();
    knobSpring.rx.stop();
  });

  function enableAudio(): void {
    audioOn = true;
    audio.ensure();
    audio.setEnabled(get(prefs).sound);
  }

  // ---- GA カスタム計測 (試合実行数・パラメーター変更) ----
  function trackMatchStart(): void {
    const p = get(prefs);
    trackEvent('match_start', {
      blue_strategy: p.variant,
      red_strategy: p.redVariant,
      blue_drive: p.blueSwerve ? 'swerve' : 'mecanum',
      red_drive: p.redSwerve ? 'swerve' : 'mecanum',
      control_mode: p.controlMode,
      speed: p.speed,
    });
  }

  // パラメーター変更の追跡: prefs をフラット化し前回スナップショットと差分を取り、変わったキーだけ
  // param_change として送る。スライダー連続変更はデバウンスで最終値1件に集約する。
  let paramSnap: Record<string, string> | null = null;
  let paramTimer: ReturnType<typeof setTimeout> | undefined;
  const PARAM_IGNORE = new Set(['playing', 'showLidar', 'showCam', 'showPath', 'faceCam', 'sound']);
  function flattenPrefs(o: Record<string, unknown>, pre = ''): Record<string, string> {
    const out: Record<string, string> = {};
    for (const [k, v] of Object.entries(o)) {
      if (!pre && PARAM_IGNORE.has(k)) continue;
      const key = pre ? `${pre}.${k}` : k;
      if (v && typeof v === 'object') Object.assign(out, flattenPrefs(v as Record<string, unknown>, key));
      else out[key] = String(v);
    }
    return out;
  }
  function trackParamChanges(p: unknown): void {
    const flat = flattenPrefs(p as Record<string, unknown>);
    if (paramSnap === null) {
      paramSnap = flat; // 初回はスナップショットのみ (読込時の差分は送らない)
      return;
    }
    clearTimeout(paramTimer);
    paramTimer = setTimeout(() => {
      const prev = paramSnap ?? {};
      for (const k of Object.keys(flat)) {
        if (prev[k] !== flat[k]) {
          const n = Number(flat[k]);
          const isNum =
            flat[k] !== '' && flat[k] !== 'true' && flat[k] !== 'false' && Number.isFinite(n);
          trackEvent('param_change', isNum ? { param: k, value: n } : { param: k, value_text: flat[k] });
        }
      }
      paramSnap = flat;
    }, 700);
  }

  function handleReset(): void {
    prefs.update((p) => ({ ...p, playing: false }));
    viewer?.resetMatch(newMatch());
    prefs.update((p) => ({ ...p, playing: true }));
    trackMatchStart();
  }

  function handleCamera(which: 'broadcast' | 'onboard' | 'top'): void {
    viewer?.cameraPreset(which);
  }

  function handleCommand(cmd: ControlCommand): void {
    viewer?.match.injectCommand(cmd);
  }

  function chooseAnalytics(consent: AnalyticsChoice): void {
    setAnalyticsConsent(consent);
    gaConsent = consent;
    // このクリックはユーザー操作なので、ここで音声解放＋シム開始
    enableAudio();
    prefs.update((p) => ({ ...p, playing: true }));
    trackMatchStart(); // 初回試合の開始を計測 (同意直後)
  }

  // ---------------- キーボード操縦 (T15) ----------------
  const keys = new Set<string>();

  function applyKeys(): void {
    const m = viewer?.match;
    if (!m) return;
    const f = (keys.has('w') ? 1 : 0) + (keys.has('s') ? -1 : 0);
    const s = (keys.has('d') ? 1 : 0) + (keys.has('a') ? -1 : 0);
    if (get(prefs).controlMode === 'player') {
      viewer?.setPlayerWalk(f, s);
      return;
    }
    if (get(prefs).controlMode === 'crane') {
      viewer?.setCraneInput(-s, -f); // A/D=旋回(pan), W/S=俯仰(lift)。向きを直感に合わせて反転
      return;
    }
    m.manual.f = f;
    m.manual.s = s;
    m.manual.rot = (keys.has('q') ? 1 : 0) + (keys.has('e') ? -1 : 0);
  }

  function onKeyDown(ev: KeyboardEvent): void {
    const mode = get(prefs).controlMode;
    const k = ev.key.toLowerCase();
    // 入力欄 (チーム名/ドロワーの数値入力) へのタイピングでロボットを動かさない
    const tag = (ev.target as HTMLElement | null)?.tagName;
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    if (mode === 'auto' || mode === 'semi') return;
    if (mode === 'player') {
      if (['w', 'a', 's', 'd'].includes(k)) {
        keys.add(k);
        applyKeys();
        ev.preventDefault();
      } else if (k === ' ') {
        viewer?.triggerPlayerJump();
        ev.preventDefault();
      }
      return;
    }
    if (mode === 'crane') {
      if (['w', 'a', 's', 'd'].includes(k)) {
        keys.add(k);
        applyKeys();
        ev.preventDefault();
      }
      return;
    }
    if (['w', 'a', 's', 'd', 'q', 'e'].includes(k)) {
      keys.add(k);
      applyKeys();
      ev.preventDefault();
    } else if (k === 'r') {
      const res = viewer?.match.manualPickup();
      if (res) showToast(res.reason ?? (res.ok ? '装填' : '装填不可'), res.ok);
    } else if (k === ' ') {
      const res = viewer?.match.manualFire();
      if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
      ev.preventDefault();
    }
  }

  function onKeyUp(ev: KeyboardEvent): void {
    keys.delete(ev.key.toLowerCase());
    applyKeys();
  }

  // ---------------- 射出クリック / FPS ポインタロック / タッチ操縦 (Q28) ----------------
  // タッチは「タップ=射出 / ドラッグ=視点」を指を離した時に判定する
  let stageDrag: { id: number; x: number; y: number; sx: number; sy: number; t0: number; moved: boolean } | null =
    null;

  function onStagePointerDown(ev: PointerEvent): void {
    enableAudio();
    const mode = get(prefs).controlMode;
    if (ev.pointerType === 'touch') {
      if (stageDrag === null) {
        stageDrag = {
          id: ev.pointerId,
          x: ev.clientX,
          y: ev.clientY,
          sx: ev.clientX,
          sy: ev.clientY,
          t0: performance.now(),
          moved: false,
        };
        viewer?.followPointerDown();
      }
      return;
    }
    viewer?.followPointerDown(); // 自機視点の相対オービット開始
    if (mode === 'tps') {
      const res = viewer?.tpsFire();
      if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
      return;
    }
    if (mode === 'player' || mode === 'crane') {
      if (!pointerLocked) {
        container.requestPointerLock();
        showToast('マウス操作ON (Escで解除)', true);
      }
      return;
    }
    if (mode !== 'fps') return;
    if (!pointerLocked) {
      container.requestPointerLock();
      showToast('照準モード開始 (Escで解除)', true);
    } else {
      const res = viewer?.match.manualFire();
      if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
    }
  }

  function onPlChange(): void {
    pointerLocked = document.pointerLockElement === container;
  }

  function onPointerMove(ev: PointerEvent): void {
    const mode = get(prefs).controlMode;
    if (ev.pointerType === 'touch') {
      if (!stageDrag || ev.pointerId !== stageDrag.id) return;
      const dx = ev.clientX - stageDrag.x;
      const dy = ev.clientY - stageDrag.y;
      stageDrag.x = ev.clientX;
      stageDrag.y = ev.clientY;
      if (Math.hypot(ev.clientX - stageDrag.sx, ev.clientY - stageDrag.sy) > 12) stageDrag.moved = true;
      viewer?.followPointerMove(dx, dy); // 自機視点オービット (内部でモード判定)
      if (mode === 'fps') {
        viewer?.addAim(-dx * 0.0035, -dy * 0.0035);
      } else if (mode === 'tps' && stageDrag.moved) {
        viewer?.setTpsCursor(ev.clientX, ev.clientY);
      }
      return;
    }
    if (mode === 'player') {
      if (pointerLocked) viewer?.lookPlayer(ev.movementX, ev.movementY);
      else viewer?.followPointerMove(ev.movementX, ev.movementY);
      return;
    }
    if (mode === 'crane') {
      viewer?.lookCrane(ev.movementX, ev.movementY);
      return;
    }
    viewer?.followPointerMove(ev.movementX, ev.movementY); // 自機視点ドラッグ (内部でモード判定)
    if (mode === 'tps') {
      viewer?.setTpsCursor(ev.clientX, ev.clientY);
      return;
    }
    if (!pointerLocked || mode !== 'fps') return;
    // 注意: 前方=(sinθ,cosθ) 規約では yaw増加=画面左回り。マウス右→右を向くには符号反転
    viewer?.addAim(-ev.movementX * 0.0021, -ev.movementY * 0.0021);
  }

  function onWheel(ev: WheelEvent): void {
    if (!viewer) return;
    if (get(prefs).controlMode === 'crane') {
      viewer.craneZoom(ev.deltaY);
      return;
    }
    viewer.followWheel(ev.deltaY); // 自機視点の距離調整 (内部でモード判定)
    if (get(prefs).controlMode !== 'fps') return;
    const m = viewer.match.manual;
    m.power = Math.min(14, Math.max(6, m.power - Math.sign(ev.deltaY) * 0.5));
  }

  function onPointerUp(ev: PointerEvent): void {
    viewer?.followPointerUp();
    if (ev.pointerType !== 'touch' || !stageDrag || ev.pointerId !== stageDrag.id) return;
    const tap = !stageDrag.moved && performance.now() - stageDrag.t0 < 350;
    const tapX = ev.clientX;
    const tapY = ev.clientY;
    stageDrag = null;
    if (!tap) return;
    // Q28: 何もない所をタップ=射出 (TPSはタップ地点を狙点にして弾道解算後に射出)
    const mode = get(prefs).controlMode;
    if (mode === 'tps') {
      viewer?.setTpsCursor(tapX, tapY);
      setTimeout(() => {
        const res = viewer?.tpsFire();
        if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
      }, 70);
    } else if (mode === 'fps') {
      const res = viewer?.match.manualFire();
      if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
    }
  }

  // ---------------- バーチャルジョイスティック (Q28) ----------------
  const joy = {
    move: { id: -1, cx: 0, cy: 0 },
    rot: { id: -1, cx: 0, cy: 0 },
  };
  /** reduced motion では移動を伴う演出をやめ、短いクロスフェードに落とす。 */
  const motion = (ms: number): number => (prefersReducedMotion() ? 0 : ms);

  /** ガラス面は「素材が現れる」ように出す。ぼかしとスケールを一緒に動かす。scale(0) からは出さない。 */
  function materialize(_node: Element, { duration = 340 } = {}) {
    return {
      duration: motion(duration),
      easing: cubicOut,
      css: (t: number, u: number) =>
        `opacity:${t}; transform: scale(${0.94 + 0.06 * t}); filter: blur(${u * 10}px);`,
    };
  }

  let moveKnob = { x: 0, y: 0 }; // 入力値 (-1..1)。離した瞬間に 0 = 指令は即切れる
  let moveVis = { x: 0, y: 0 }; // 見た目 (px)。離すと速度を引き継いで中心へ戻る
  let rotKnob = { x: 0, y: 0 };
  let rotVis = { x: 0, y: 0 };

  const KNOB_R = 30; // つまみの可動半径 px
  // 2D は X/Y を独立したスプリングにする (1本にすると軸ごとの速度差でズレる)
  const knobSpring = {
    mx: new Spring(0, (v) => (moveVis = { ...moveVis, x: v }), SPRING_UI),
    my: new Spring(0, (v) => (moveVis = { ...moveVis, y: v }), SPRING_UI),
    rx: new Spring(0, (v) => (rotVis = { ...rotVis, x: v }), SPRING_UI),
  };
  const knobVel = { mx: new VelocityTracker(), my: new VelocityTracker(), rx: new VelocityTracker() };

  function applyJoy(): void {
    const m = viewer?.match;
    if (!m) return;
    const dz = (v: number): number => (Math.abs(v) < 0.14 ? 0 : v);
    if (get(prefs).controlMode === 'player') {
      viewer?.setPlayerWalk(-dz(moveKnob.y), dz(moveKnob.x));
      return;
    }
    m.manual.f = -dz(moveKnob.y);
    m.manual.s = dz(moveKnob.x);
    m.manual.rot = -dz(rotKnob.x); // 右に倒す=右旋回 (rot正=左旋回)
  }

  function joyDown(ev: PointerEvent, which: 'move' | 'rot'): void {
    enableAudio();
    const j = joy[which];
    j.id = ev.pointerId;
    j.cx = ev.clientX;
    j.cy = ev.clientY;
    (ev.currentTarget as HTMLElement).setPointerCapture(ev.pointerId);
    // 戻り途中のつまみでも掴める。現在の表示値から指へ引き継ぐ。
    if (which === 'move') {
      knobSpring.mx.stop();
      knobSpring.my.stop();
      knobVel.mx.clear();
      knobVel.my.clear();
    } else {
      knobSpring.rx.stop();
      knobVel.rx.clear();
    }
  }

  function joyMove(ev: PointerEvent, which: 'move' | 'rot'): void {
    const j = joy[which];
    if (ev.pointerId !== j.id) return;
    const nx = Math.max(-1, Math.min(1, (ev.clientX - j.cx) / 42));
    const ny = Math.max(-1, Math.min(1, (ev.clientY - j.cy) / 42));
    const t = ev.timeStamp / 1000;
    if (which === 'move') {
      moveKnob = { x: nx, y: ny };
      knobSpring.mx.jump(nx * KNOB_R); // ドラッグ中は 1:1 追従 (物理は止める)
      knobSpring.my.jump(ny * KNOB_R);
      knobVel.mx.add(nx * KNOB_R, t);
      knobVel.my.add(ny * KNOB_R, t);
    } else {
      rotKnob = { x: nx, y: ny };
      knobSpring.rx.jump(nx * KNOB_R);
      knobVel.rx.add(nx * KNOB_R, t);
    }
    applyJoy();
  }

  function joyUp(ev: PointerEvent, which: 'move' | 'rot'): void {
    const j = joy[which];
    if (ev.pointerId !== j.id) return;
    j.id = -1;
    // 指令は即ゼロ (操作は歯切れよく)。つまみだけが離した速度で中心へ帰る。
    if (which === 'move') {
      moveKnob = { x: 0, y: 0 };
      knobSpring.mx.set(0, { velocity: knobVel.mx.velocity() });
      knobSpring.my.set(0, { velocity: knobVel.my.velocity() });
    } else {
      rotKnob = { x: 0, y: 0 };
      knobSpring.rx.set(0, { velocity: knobVel.rx.velocity() });
    }
    applyJoy();
  }

  // タッチ用の明示ボタン (スマホには R キー等が無い)
  function touchReload(): void {
    const res = viewer?.match.manualPickup();
    if (res) showToast(res.reason ?? (res.ok ? '装填' : '装填不可'), res.ok);
  }
  function touchFire(): void {
    const mode = get(prefs).controlMode;
    const res = mode === 'tps' ? viewer?.tpsFire() : viewer?.match.manualFire();
    if (res && !res.ok) showToast(res.reason ?? '射出不可', false);
  }
</script>

<svelte:window
  on:keydown={onKeyDown}
  on:keyup={onKeyUp}
  on:pointermove={onPointerMove}
  on:wheel={onWheel}
  on:pointerup={onPointerUp}
  on:pointercancel={onPointerUp}
/>

<div
  class="stage"
  bind:this={container}
  on:pointerdown={onStagePointerDown}
  role="application"
  aria-label="3Dシミュレーション"
></div>

<!-- マルチカムは放送マルチビューなので、スコアバグは時計のみ・ログ等は隠して映像を邪魔しない -->
<ScoreBug compact={$prefs.controlMode === 'multicam'} />
<PerfPanel />
{#if $prefs.controlMode !== 'multicam'}
  <SidePanel />
{/if}
<CommandPad onCommand={handleCommand} />
<Dock onReset={handleReset} onCamera={handleCamera} onOpenSettings={() => (drawerOpen = true)} />
<SettingsDrawer bind:open={drawerOpen} onReset={handleReset} />

{#if cpuMode}
  <div class="cpu-badge" role="status">
    CPU描画モード (2D俯瞰) — WebGLが使えないため簡易表示です。3D表示にはブラウザの「ハードウェア
    アクセラレーション」を有効にして再読み込みしてください。
  </div>
{/if}

{#if glFailed}
  <div class="gl-error" role="alert">
    <h2>表示を開始できませんでした</h2>
    <p>
      WebGL も Canvas 2D も初期化できませんでした。ブラウザを再読み込みするか、別のブラウザで
      お試しください。
    </p>
    <button on:click={() => location.reload()}>再読み込み</button>
  </div>
{/if}

{#if !audioOn}
  <button class="audio-cta" on:click={enableAudio}>タップして音声ON (自動再生中)</button>
{/if}

{#if $prefs.controlMode === 'fps'}
  <div class="fps-hud">
    <div class="cross"></div>
    <div class="fps-info">
      威力 {$hud?.manualPower.toFixed(1) ?? '9.0'} m/s {isCoarse ? '' : '(ホイールで調整)'}
      <span class:ready={$hud?.fireReady}>{$hud?.fireReady ? '射出可' : '装填中…'}</span>
      <br />{#if isCoarse}ドラッグ=照準 / タップ=射出 / 左スティック=移動 / 右=旋回{:else}クリック=射出 / WASD=移動 / QE=旋回 / R=補充 / {pointerLocked ? 'Esc=解除' : 'クリックで照準開始'}{/if}
    </div>
  </div>
{:else if $prefs.controlMode === 'tps'}
  <div class="drive-hint">
    {#if isCoarse}
      ドラッグ=狙点移動 (緑リング=射程内) / 何もない所をタップ=射出
      <br />左スティック=移動 / 右スティック=旋回
    {:else}
      カーソルで狙う (緑リング=射程内・軌道表示 / 赤=射程外) → クリックで射出
      <br />WASD=移動 / QE=旋回 / R=補充スポットで装填 / 残弾は右上パネル
    {/if}
  </div>
{/if}

{#if isCoarse && ($prefs.controlMode === 'fps' || $prefs.controlMode === 'tps' || $prefs.controlMode === 'player')}
  <div
    class="joy left"
    role="slider"
    aria-label="移動スティック"
    aria-valuenow={0}
    tabindex="-1"
    on:pointerdown={(e) => joyDown(e, 'move')}
    on:pointermove={(e) => joyMove(e, 'move')}
    on:pointerup={(e) => joyUp(e, 'move')}
    on:pointercancel={(e) => joyUp(e, 'move')}
  >
    <div class="joy-base">
      <div class="joy-knob" style="transform: translate({moveVis.x}px, {moveVis.y}px)"></div>
    </div>
    <div class="joy-label">{$prefs.controlMode === 'player' ? '歩く' : '移動'}</div>
  </div>
  {#if $prefs.controlMode === 'fps' || $prefs.controlMode === 'tps'}
    <div
      class="joy right"
      role="slider"
      aria-label="旋回スティック"
      aria-valuenow={0}
      tabindex="-1"
      on:pointerdown={(e) => joyDown(e, 'rot')}
      on:pointermove={(e) => joyMove(e, 'rot')}
      on:pointerup={(e) => joyUp(e, 'rot')}
      on:pointercancel={(e) => joyUp(e, 'rot')}
    >
      <div class="joy-base">
        <div class="joy-knob" style="transform: translate({rotVis.x}px, 0px)"></div>
      </div>
      <div class="joy-label">旋回</div>
    </div>
    <div class="touch-actions">
      <button class="tbtn reload" on:click={touchReload}>装填</button>
      <button class="tbtn fire" on:click={touchFire}>射出</button>
    </div>
  {:else if $prefs.controlMode === 'player'}
    <button class="jump-btn" on:click={() => viewer?.triggerPlayerJump()}>ジャンプ</button>
  {/if}
{/if}

{#if $prefs.controlMode === 'multicam'}
  <div class="multicam-labels">
    <span>CAM 1</span><span>CAM 2</span><span>CRANE</span>
    <span>CAM 3</span><span>CAM 4</span><span>WIDE</span>
  </div>
{/if}

{#if $prefs.controlMode === 'crane'}
  <div class="drive-hint">
    クレーン操縦中(先端カメラ視点): A/D=ブーム旋回 ・ W/S=俯仰(上下) ・ クリックでマウス操作(Esc解除)→ヘッドのパン/チルト ・ ホイールでズーム
  </div>
{/if}

{#if $prefs.controlMode === 'player'}
  <div class="drive-hint">
    選手操作中: WASD/左スティックで移動 ・ {isCoarse ? 'ドラッグ' : 'クリックでマウス視点(Esc解除)/ドラッグ'}で視点 ・ スペース{isCoarse ? '/ボタン' : ''}でジャンプ ・ ロボットや観客と当たる
  </div>
{/if}

{#if toast}
  <!-- 出た方向へ帰す (spatial consistency) -->
  <div
    class="toast"
    class:ng={!toastOk}
    in:fly={{ y: 14, duration: motion(280), easing: cubicOut }}
    out:fly={{ y: 14, duration: motion(200), easing: cubicOut }}>
    {toast}
  </div>
{/if}

{#if gaEnabled && gaConsent === null}
  <div class="gate" transition:fade={{ duration: motion(240) }}>
    <div class="gate-card" transition:materialize>
      <div class="gate-title">試合シミュレーションを開始</div>
      <p class="gate-note">サイトの利用状況を匿名で記録します（広告には使いません）。<br />許可すると開始します。</p>
      <div class="gate-btns">
        <button class="primary" on:click={() => chooseAnalytics('granted')}>許可して開始</button>
      </div>
    </div>
  </div>
{/if}

<div
  class="sensors"
  class:hidden={!$prefs.showCam ||
    $prefs.controlMode === 'fps' ||
    $prefs.controlMode === 'multicam' ||
    $prefs.controlMode === 'ragcam' ||
    (isCoarse && $prefs.controlMode === 'tps')}
>
  <div class="panel">
    <div class="pipwrap">
      <div class="pip" bind:this={pipBox}></div>
      <canvas class="pip-ov" bind:this={pipOverlay}></canvas>
    </div>
    <div class="label">照準カメラ</div>
  </div>
</div>

<style>
  .stage {
    position: fixed;
    inset: 0;
    touch-action: none;
  }
  /* Q28: バーチャルジョイスティック */
  .joy {
    position: fixed;
    bottom: 96px;
    z-index: 26;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    touch-action: none;
    user-select: none;
    -webkit-user-select: none;
  }
  .joy.left {
    left: 14px;
  }
  .joy.right {
    right: 14px;
  }
  /* タッチ用の装填/射出ボタン (ジョイスティックの間・操作パネルの上) */
  .touch-actions {
    position: fixed;
    bottom: 100px;
    left: 50%;
    transform: translateX(-50%);
    z-index: 26;
    display: flex;
    gap: 12px;
  }
  .tbtn {
    width: 80px;
    height: 60px;
    border: 0.5px solid var(--panel-edge);
    border-radius: 18px;
    color: #fff;
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 15px;
    letter-spacing: var(--track-body);
    box-shadow: var(--panel-shadow-sm);
    transition: transform var(--dur-press) var(--ease-out);
  }
  .tbtn.reload {
    background: var(--mat-thick-bg);
    backdrop-filter: var(--mat-blur-regular);
    -webkit-backdrop-filter: var(--mat-blur-regular);
  }
  .tbtn.fire {
    background: var(--accent);
    border-color: rgba(255, 255, 255, 0.24);
  }
  .tbtn:active {
    transform: scale(0.94);
  }
  .jump-btn {
    position: fixed;
    bottom: 110px;
    right: 22px;
    z-index: 26;
    width: 96px;
    height: 96px;
    border: 0.5px solid rgba(255, 255, 255, 0.24);
    border-radius: 50%;
    background: rgba(10, 132, 255, 0.8);
    backdrop-filter: var(--mat-blur-thin);
    -webkit-backdrop-filter: var(--mat-blur-thin);
    color: #fff;
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 16px;
    box-shadow: var(--panel-shadow-sm);
    transition: transform var(--dur-press) var(--ease-out);
  }
  .jump-btn:active {
    transform: scale(0.94);
  }
  /* スティック: 台は薄いマテリアル、つまみは指に 1:1 追従し、離すと速度を保ったまま中心へ帰る */
  .joy-base {
    width: 108px;
    height: 108px;
    border-radius: 50%;
    background: var(--mat-thin-bg);
    backdrop-filter: var(--mat-blur-thin);
    -webkit-backdrop-filter: var(--mat-blur-thin);
    border: 0.5px solid rgba(255, 255, 255, 0.18);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.1);
    display: grid;
    place-items: center;
  }
  .joy-knob {
    width: 48px;
    height: 48px;
    border-radius: 50%;
    background: rgba(255, 255, 255, 0.82);
    box-shadow:
      0 2px 10px rgba(0, 0, 0, 0.4),
      inset 0 -1px 2px rgba(0, 0, 0, 0.12);
    will-change: transform;
  }
  .joy-label {
    color: var(--text-mid);
    font-family: var(--font-ui);
    font-size: 11px;
    font-weight: 500;
    letter-spacing: var(--track-caption);
    background: var(--mat-thin-bg);
    backdrop-filter: var(--mat-blur-thin);
    -webkit-backdrop-filter: var(--mat-blur-thin);
    padding: 2px 10px;
    border-radius: var(--pill);
  }
  .cpu-badge {
    position: fixed;
    top: max(8px, env(safe-area-inset-top));
    left: 8px;
    z-index: 40;
    max-width: 320px;
    background: var(--mat-thick-bg);
    backdrop-filter: var(--mat-blur-regular);
    -webkit-backdrop-filter: var(--mat-blur-regular);
    border: 0.5px solid var(--panel-border);
    border-radius: 14px;
    padding: 10px 14px;
    color: var(--text-hi);
    font-family: var(--font-ui);
    font-size: 12px;
    font-weight: 400;
    line-height: 1.65;
    box-shadow: var(--panel-shadow-sm);
  }

  .gl-error {
    position: fixed;
    inset: 0;
    z-index: 90;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 16px;
    padding: 24px;
    text-align: center;
    background: rgba(12, 13, 17, 0.96);
    color: var(--text-hi);
    font-family: var(--font-ui);
  }
  .gl-error h2 {
    margin: 0;
    font-size: 24px;
    font-weight: 640;
    letter-spacing: var(--track-title);
  }
  .gl-error p {
    margin: 0;
    max-width: 520px;
    font-size: 15px;
    line-height: 1.7;
    color: var(--text-mid);
  }
  .gl-error button {
    background: var(--accent);
    color: #fff;
    border: none;
    border-radius: var(--pill);
    padding: 12px 24px;
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 15px;
    cursor: pointer;
    transition: transform var(--dur-press) var(--ease-out);
  }
  .gl-error button:active {
    transform: scale(0.95);
  }

  .audio-cta {
    position: fixed;
    bottom: 78px;
    left: 50%;
    transform: translateX(-50%);
    z-index: 30;
    background: var(--accent);
    color: #fff;
    border: none;
    border-radius: var(--pill);
    padding: 11px 22px;
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 14px;
    letter-spacing: var(--track-body);
    cursor: pointer;
    box-shadow: var(--panel-shadow);
    transition: transform var(--dur-press) var(--ease-out);
  }
  .audio-cta:active {
    transform: translateX(-50%) scale(0.95);
  }
  .fps-hud {
    position: fixed;
    inset: 0;
    z-index: 25;
    pointer-events: none;
  }
  .cross {
    position: absolute;
    left: 50%;
    top: 50%;
    width: 26px;
    height: 26px;
    transform: translate(-50%, -50%);
  }
  .cross::before,
  .cross::after {
    content: '';
    position: absolute;
    background: rgba(255, 255, 255, 0.9);
  }
  .cross::before {
    left: 12px;
    top: 0;
    width: 2px;
    height: 26px;
  }
  .cross::after {
    top: 12px;
    left: 0;
    width: 26px;
    height: 2px;
  }
  /* 操作ヒントは浮いたカプセル。映像の上に薄く乗せる。 */
  .fps-info,
  .drive-hint {
    position: fixed;
    left: 50%;
    top: 122px;
    transform: translateX(-50%);
    max-width: min(92vw, 520px);
    text-align: center;
    color: var(--text-mid);
    font-family: var(--font-ui);
    font-size: 13px;
    font-weight: 400;
    line-height: 1.65;
    background: var(--mat-regular-bg);
    backdrop-filter: var(--mat-blur-regular);
    -webkit-backdrop-filter: var(--mat-blur-regular);
    border: 0.5px solid var(--panel-border);
    border-top-color: var(--panel-edge);
    padding: 8px 16px;
    border-radius: 16px;
    box-shadow: var(--panel-shadow-sm);
  }
  .fps-info .ready {
    color: var(--ok);
    font-weight: 590;
    margin-left: 8px;
  }
  .drive-hint {
    z-index: 24;
    pointer-events: none;
  }
  .multicam-labels {
    position: fixed;
    inset: 0;
    z-index: 22;
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    grid-template-rows: repeat(2, 1fr);
    pointer-events: none;
  }
  .multicam-labels span {
    align-self: start;
    justify-self: start;
    margin: 8px;
    padding: 2px 9px;
    background: rgba(0, 0, 0, 0.6);
    color: #ff4a4a;
    font-family: 'Roboto Mono', monospace;
    font-weight: 700;
    font-size: 12px;
    letter-spacing: 1px;
    border-radius: 3px;
    border: 1px solid rgba(255, 74, 74, 0.4);
  }
  .toast {
    position: fixed;
    left: 50%;
    bottom: 156px;
    transform: translateX(-50%);
    z-index: 40;
    background: var(--mat-thick-bg);
    backdrop-filter: var(--mat-blur-regular);
    -webkit-backdrop-filter: var(--mat-blur-regular);
    border: 0.5px solid var(--panel-border);
    border-top-color: var(--panel-edge);
    color: var(--ok);
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 14px;
    padding: 10px 20px;
    border-radius: var(--pill);
    pointer-events: none;
    box-shadow: var(--panel-shadow);
  }
  .toast.ng {
    color: var(--bad);
  }
  /* 開始ゲート: 選択するまでシムは始まらないブロッキングモーダル。
     モーダルな作業なので、暗くして背後を押し下げる。 */
  .gate {
    position: fixed;
    inset: 0;
    z-index: 60;
    display: grid;
    place-items: center;
    background: rgba(0, 0, 0, 0.5);
    backdrop-filter: blur(6px) saturate(120%);
    -webkit-backdrop-filter: blur(6px) saturate(120%);
    user-select: none;
  }
  .gate-card {
    width: min(90vw, 350px);
    background: var(--mat-thick-bg);
    backdrop-filter: var(--mat-blur-thick);
    -webkit-backdrop-filter: var(--mat-blur-thick);
    border: 0.5px solid var(--panel-border);
    border-top-color: var(--panel-edge);
    border-radius: 22px;
    padding: 24px 22px 18px;
    text-align: center;
    box-shadow: var(--panel-shadow);
    font-family: var(--font-ui);
  }
  .gate-title {
    color: var(--text-hi);
    font-weight: 640;
    font-size: 19px;
    letter-spacing: var(--track-title);
    margin-bottom: 8px;
  }
  .gate-note {
    color: var(--text-mid);
    font-size: 13px;
    line-height: 1.65;
    margin: 0 0 18px;
  }
  .gate-btns {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  .gate-btns button {
    border: 0;
    border-radius: 14px;
    color: #fff;
    font-family: inherit;
    font-weight: 590;
    font-size: 16px;
    padding: 14px 16px;
    cursor: pointer;
    transition: transform var(--dur-press) var(--ease-out);
  }
  .gate-btns .primary {
    background: var(--accent);
  }
  .gate-btns button:active {
    transform: scale(0.96);
  }
  .sensors {
    position: fixed;
    right: 12px;
    bottom: 12px;
    z-index: 20;
    display: flex;
    flex-direction: column;
    gap: 10px;
    align-items: flex-end;
    user-select: none;
  }
  .sensors.hidden {
    display: none;
  }
  /* 照準カメラの映像はメインの WebGL キャンバスにシザーで直接描かれる = このパネルの
     「背後」にある。よって背景を敷いたり backdrop-filter を掛けると映像そのものが
     曇る。枠 (境界線・角丸・影) だけを持たせ、窓の中は完全に素通しにする。 */
  .panel {
    position: relative;
    border: 0.5px solid var(--panel-border);
    border-top-color: var(--panel-edge);
    border-radius: 16px;
    overflow: hidden;
    background: transparent;
    box-shadow: var(--panel-shadow-sm);
  }
  .pipwrap {
    position: relative;
    width: 300px;
    height: 168px;
  }
  .pip {
    width: 100%;
    height: 100%;
  }
  .pip-ov {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }
  .label {
    background: rgba(0, 0, 0, 0.5);
    color: var(--text-mid);
    font-family: var(--font-ui);
    font-size: 11px;
    font-weight: 500;
    letter-spacing: var(--track-caption);
    padding: 4px 10px;
    border-top: 0.5px solid rgba(255, 255, 255, 0.08);
  }
  @media (max-width: 760px) {
    .sensors {
      left: 6px;
      right: auto;
      bottom: 112px;
      gap: 6px;
      flex-direction: row;
      align-items: flex-start;
    }
    .pipwrap {
      width: min(42vw, 176px);
      height: auto;
      aspect-ratio: 16 / 9;
    }
    .label {
      font-size: 8px;
      padding: 2px 5px;
      max-width: 176px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .audio-cta {
      bottom: 96px;
      font-size: 12px;
      padding: 8px 16px;
    }
  }
  @media (max-width: 520px) and (orientation: portrait) {
    .sensors {
      left: 6px;
      right: auto;
      bottom: 172px;
    }
    .pipwrap {
      width: min(49vw, 182px);
      height: auto;
      aspect-ratio: 16 / 9;
    }
    .label {
      max-width: min(49vw, 182px);
      font-size: 7.5px;
    }
  }
</style>
