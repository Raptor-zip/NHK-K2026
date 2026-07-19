<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import {
    faGear,
    faPause,
    faPlay,
    faRotateRight,
    faVolumeHigh,
    faVolumeXmark,
  } from '@fortawesome/free-solid-svg-icons';
  import { prefs } from './hudStore';
  import { segmented } from './segmented';
  import type { TeamId } from '../config/field';

  export let onReset: () => void;
  export let onCamera: (which: 'broadcast' | 'onboard' | 'top') => void;
  export let onOpenSettings: () => void;

  type CameraMode = 'broadcast' | 'onboard' | 'top';
  let cameraMode: CameraMode = 'broadcast';
  // 直接操縦系はカメラが専用になるためプリセット切替を無効化
  $: directView =
    $prefs.controlMode === 'tps' ||
    $prefs.controlMode === 'fps' ||
    $prefs.controlMode === 'player' ||
    $prefs.controlMode === 'crane' ||
    $prefs.controlMode === 'multicam' ||
    $prefs.controlMode === 'ragcam';

  function togglePlay(): void {
    prefs.update((p) => ({ ...p, playing: !p.playing }));
  }

  function setCamera(which: CameraMode): void {
    cameraMode = which;
    onCamera(which);
  }

  function setPlayerTeam(team: TeamId): void {
    if ($prefs.playerTeam === team) return;
    prefs.update((p) => ({ ...p, playerTeam: team }));
    onReset();
  }

  function toggleSound(): void {
    prefs.update((p) => ({ ...p, sound: !p.sound }));
  }
</script>

<div class="dock panel-glass">
  <div class="group">
    <button
      class="ui-btn primary play"
      on:click={togglePlay}
      title={$prefs.playing ? '一時停止' : '再生'}
      aria-label={$prefs.playing ? '一時停止' : '再生'}>
      <span class="ui-icon" aria-hidden="true">
        <FontAwesomeIcon icon={$prefs.playing ? faPause : faPlay} />
      </span>
    </button>
    <button class="ui-btn icon-only" on:click={onReset} title="試合をリセット" aria-label="試合をリセット">
      <span class="ui-icon" aria-hidden="true"><FontAwesomeIcon icon={faRotateRight} /></span>
    </button>
    <select class="ui-select" bind:value={$prefs.speed} title="再生速度" aria-label="再生速度">
      <option value={0.5}>0.5×</option>
      <option value={1}>1×</option>
      <option value={2}>2×</option>
      <option value={4}>4×</option>
    </select>
  </div>

  <div class="divider"></div>

  <div class="group">
    <div class="ui-seg" class:muted={directView} role="group" aria-label="視点" use:segmented>
      <button class:active={cameraMode === 'broadcast'} disabled={directView} on:click={() => setCamera('broadcast')}
        >放送</button>
      <button class:active={cameraMode === 'top'} disabled={directView} on:click={() => setCamera('top')}>俯瞰</button>
      <button class:active={cameraMode === 'onboard'} disabled={directView} on:click={() => setCamera('onboard')}
        >追従</button>
    </div>
    <select class="ui-select mode" bind:value={$prefs.controlMode} title="操縦モード" aria-label="操縦モード">
      <option value="auto">自動 (戦略スクリプト)</option>
      <option value="semi">半自動 (ボタン指令)</option>
      <option value="tps">TPS操縦</option>
      <option value="fps">FPS照準</option>
      <option value="player">選手を歩かせる</option>
      <option value="crane">クレーン操縦</option>
      <option value="multicam">マルチカム</option>
      <option value="ragcam">投擲雑巾視点</option>
    </select>
  </div>

  <div class="divider"></div>

  <div class="group">
    <div class="ui-seg teams" data-team={$prefs.playerTeam} role="group" aria-label="操作チーム" use:segmented>
      <button
        class="team-b"
        class:active={$prefs.playerTeam === 'blue'}
        title="青チームを操作"
        on:click={() => setPlayerTeam('blue')}>青</button>
      <button
        class="team-r"
        class:active={$prefs.playerTeam === 'red'}
        title="赤チームを操作"
        on:click={() => setPlayerTeam('red')}>赤</button>
    </div>
    <button
      class="ui-btn icon-only"
      on:click={toggleSound}
      title={$prefs.sound ? '音を消す' : '音を出す'}
      aria-label={$prefs.sound ? '音を消す' : '音を出す'}>
      <span class="ui-icon" aria-hidden="true">
        <FontAwesomeIcon icon={$prefs.sound ? faVolumeHigh : faVolumeXmark} />
      </span>
    </button>
    <button class="ui-btn settings" on:click={onOpenSettings} title="設定・パラメータ">
      <span class="ui-icon" aria-hidden="true"><FontAwesomeIcon icon={faGear} /></span>
      <span>設定</span>
    </button>
  </div>
</div>

<style>
  /* フローティングのツールバー。コンテンツはこの下を流れる (帯で画面を割らない)。 */
  .dock {
    position: fixed;
    bottom: max(14px, env(safe-area-inset-bottom));
    left: 50%;
    transform: translateX(-50%);
    z-index: 20;
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 9px 12px;
    border-radius: var(--pill);
    max-width: 96vw;
    flex-wrap: wrap;
    justify-content: center;
    user-select: none;
  }
  .group {
    display: flex;
    align-items: center;
    gap: 7px;
  }
  .divider {
    width: 0.5px;
    height: 22px;
    background: rgba(255, 255, 255, 0.14);
  }
  /* 主操作は円形の目立つボタン。押下フィードバックは pointer-down 直後 (app.css)。 */
  .play {
    width: 40px;
    height: 40px;
    min-width: 40px;
    padding: 0;
    border-radius: 50%;
    font-size: 14px;
  }
  .ui-btn.icon-only {
    height: 36px;
    border-radius: var(--pill);
  }
  .settings {
    border-radius: var(--pill);
  }
  .ui-select {
    border-radius: var(--pill);
  }
  .mode {
    max-width: 178px;
  }
  /* 視点セグメントが無効なときは、消すのではなく静かに引く */
  .ui-seg.muted {
    opacity: 0.45;
  }
  /* つまみは action が挿入する要素なので :global で色を差し替える */
  .ui-seg.teams[data-team='blue'] :global(.seg-thumb) {
    background: var(--blue);
  }
  .ui-seg.teams[data-team='red'] :global(.seg-thumb) {
    background: var(--red);
  }

  @media (max-width: 760px) {
    .dock {
      bottom: max(8px, env(safe-area-inset-bottom));
      gap: 6px;
      padding: 7px 9px;
    }
    .divider {
      display: none;
    }
    .mode {
      max-width: 142px;
      font-size: 12px;
    }
  }
</style>
