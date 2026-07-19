<script lang="ts">
  import { hud, prefs } from './hudStore';
  import type { ControlCommand } from '../sim/types';

  // 半自動操縦の指令パッド (T22): スマホから操作する想定の仮想コントローラー。
  export let onCommand: (cmd: ControlCommand) => void;

  $: ammo = $hud?.metrics.ammo ?? 0;
  $: selectedStatus = $prefs.playerTeam === 'blue' ? ($hud?.status.blue ?? '') : ($hud?.status.red ?? '');
  $: busy = selectedStatus !== '' && selectedStatus !== '半自動待機';
  $: opponentName = $prefs.playerTeam === 'blue' ? '赤チーム' : '青チーム';
</script>

{#if $prefs.showPad && $prefs.controlMode === 'semi'}
  <div class="pad">
    <div class="title">半自動指令 {busy ? ` / ${selectedStatus}` : ''}</div>
    <button on:click={() => onCommand('retry')}>リトライ</button>
    <button on:click={() => onCommand('resup')}>補充</button>
    <button class="fire" disabled={ammo <= 0} on:click={() => onCommand('throwFlag')}
      >旗へ射撃 {ammo > 0 ? '' : '(弾切れ)'}</button>
    <button class="fire" disabled={ammo <= 0} on:click={() => onCommand('throwMoving')}
      >{opponentName}バケツ {ammo > 0 ? '' : '(弾切れ)'}</button>
    <button class="small" disabled={ammo <= 0} on:click={() => onCommand('throwB1')}>固定1</button>
    <button class="small" disabled={ammo <= 0} on:click={() => onCommand('throwB2')}>固定2</button>
    <button class="small" disabled={ammo <= 0} on:click={() => onCommand('throwB3')}>固定3</button>
    <button class="small" disabled={ammo <= 0} on:click={() => onCommand('throwDesk1')}>机1</button>
    <button class="small" disabled={ammo <= 0} on:click={() => onCommand('throwDesk2')}>机2</button>
    <button class="stop" on:click={() => onCommand('halt')}>停止</button>
  </div>
{/if}

<style>
  .pad {
    position: fixed;
    right: 14px;
    top: 50%;
    transform: translateY(-50%);
    z-index: 18;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    background: var(--panel-bg);
    border: 0.5px solid var(--panel-border);
    border-top-color: var(--panel-edge);
    border-radius: var(--panel-radius);
    backdrop-filter: var(--panel-blur);
    -webkit-backdrop-filter: var(--panel-blur);
    box-shadow: var(--panel-shadow);
    padding: 10px;
    width: 196px;
    user-select: none;
  }
  .title {
    grid-column: 1 / -1;
    font-family: var(--font-ui);
    font-size: 11px;
    font-weight: 500;
    letter-spacing: var(--track-caption);
    color: var(--text-mid);
    text-align: center;
    padding-bottom: 2px;
  }
  button {
    background: var(--ctl-bg);
    border: none;
    border-radius: var(--ctl-radius);
    color: #fff;
    padding: 12px 6px;
    cursor: pointer;
    font-family: var(--font-ui);
    font-weight: 590;
    font-size: 13px;
    letter-spacing: var(--track-body);
    /* 押した瞬間に返す。指を離してからでは遅い。 */
    transition:
      background var(--dur-fast) var(--ease-out),
      transform var(--dur-press) var(--ease-out);
  }
  button:hover:not(:disabled) {
    background: var(--ctl-bg-hover);
  }
  button:not(.small),
  button.stop {
    grid-column: 1 / -1;
  }
  button:active:not(:disabled) {
    transform: scale(0.94);
  }
  button:disabled {
    opacity: 0.35;
    cursor: not-allowed;
  }
  button.fire {
    background: var(--accent);
  }
  button.stop {
    background: rgba(255, 69, 58, 0.85);
  }
  @media (max-width: 760px) {
    .pad {
      right: 6px;
      top: auto;
      bottom: 112px;
      transform: none;
      width: min(46vw, 180px);
      padding: 6px;
      gap: 5px;
    }
    button {
      padding: 8px 4px;
      font-size: 11px;
    }
    .title {
      font-size: 8px;
    }
  }
  @media (max-width: 520px) and (orientation: portrait) {
    .pad {
      bottom: 126px;
      width: 44vw;
      max-height: 38vh;
      overflow: auto;
    }
    button {
      padding: 7px 3px;
      font-size: 10px;
    }
  }
</style>
