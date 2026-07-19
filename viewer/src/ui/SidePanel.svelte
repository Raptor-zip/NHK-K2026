<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faChevronDown, faList } from '@fortawesome/free-solid-svg-icons';
  import { hud, prefs } from './hudStore';
  import { segmented } from './segmented';

  // イベントログ + テレメトリをタブで切り替える左下パネル。
  // モバイルでは初期折りたたみで映像を邪魔しない。
  let tab: 'log' | 'telemetry' = 'log';
  let collapsed = matchMedia('(max-width: 760px)').matches;

  function fmt(t: number): string {
    const x = Math.max(0, Math.floor(t));
    return `${Math.floor(x / 60)}:${String(x % 60).padStart(2, '0')}`;
  }

  $: lines = ($hud?.events ?? []).slice(-8);
  $: m = $hud?.metrics;
  $: opponentName = $prefs.playerTeam === 'blue' ? '赤チーム' : '青チーム';
  const num = (v: number | null | undefined, unit = ''): string =>
    v === null || v === undefined ? 'ー' : `${Math.round(v)}${unit}`;
</script>

<div class="side panel-glass" class:collapsed>
  <div class="head">
    {#if !collapsed}
      <div class="ui-seg tabs" role="tablist" use:segmented>
        <button role="tab" class:active={tab === 'log'} on:click={() => (tab = 'log')}>ログ</button>
        <button role="tab" class:active={tab === 'telemetry'} on:click={() => (tab = 'telemetry')}
          >テレメトリ</button>
      </div>
    {/if}
    <button class="fold" on:click={() => (collapsed = !collapsed)} aria-label={collapsed ? '開く' : '折りたたむ'}>
      {#if collapsed}
        <span class="ui-icon" aria-hidden="true"><FontAwesomeIcon icon={faList} /></span>
        <span>ログ</span>
      {:else}
        <span class="ui-icon" aria-hidden="true"><FontAwesomeIcon icon={faChevronDown} /></span>
      {/if}
    </button>
  </div>

  {#if !collapsed}
    {#if tab === 'log'}
      <div class="log" role="log">
        {#each lines as e (e.t + e.text)}
          <div class="line {e.cls}"><span class="t">{fmt(e.t)}</span>{e.text}</div>
        {:else}
          <div class="line empty">まだイベントはありません</div>
        {/each}
      </div>
    {:else if m}
      <div class="telemetry">
        <div class="row">
          <span>推定ポーズ</span>
          <b>X{m.estX.toFixed(2)} Z{m.estZ.toFixed(2)} θ{m.estThetaDeg.toFixed(1)}°</b>
        </div>
        <div class="row"><span>速度 / 加速度</span><b>{m.speed.toFixed(2)} m/s ・ {m.accel.toFixed(1)} m/s²</b></div>
        <div class="row"><span>射出 / 仰角</span><b>{m.muzzleSpeed.toFixed(1)} m/s ・ {m.muzzlePitchDeg.toFixed(0)}°</b></div>
        <div class="row"><span>自己位置誤差</span><b>{num(m.estErrMm, 'mm')}</b></div>
        <div class="row"><span>ICP残差 (既知地図)</span><b>{num(m.icpRmseMm, 'mm')}</b></div>
        <div class="row"><span>{opponentName}推定誤差</span><b>{num(m.oppErrMm, 'mm')}</b></div>
        <div class="row"><span>p(旗) / p(移動)</span><b>{(m.pFlag * 100).toFixed(0)}% ・ {(m.pMoving * 100).toFixed(0)}%</b></div>
        {#if m.lastShot}
          <div class="row shot"><span>直近射撃 {m.lastShot.label}</span><b>命中率 {(m.lastShot.prob * 100).toFixed(0)}%</b></div>
        {/if}
        <div class="row"><span>残弾</span><b>{m.ammo}{m.superAmmo > 0 ? ` +S${m.superAmmo}` : ''}</b></div>
        {#if m.nextEvent}
          <div class="row"><span>次: {m.nextEvent.label}</span><b>{Math.ceil(m.nextEvent.inSec)}s</b></div>
        {/if}
      </div>
    {/if}
  {/if}
</div>

<style>
  .side {
    position: fixed;
    left: 14px;
    bottom: max(14px, env(safe-area-inset-bottom));
    z-index: 19;
    width: 336px;
    max-width: min(88vw, 336px);
    color: var(--text-mid);
    user-select: none;
    overflow: hidden;
  }
  /* 畳んだ状態は「開く」だけのピル。映像を邪魔しない薄いマテリアル。 */
  .side.collapsed {
    width: auto;
    border-radius: var(--pill);
    background: var(--mat-thin-bg);
    backdrop-filter: var(--mat-blur-thin);
    -webkit-backdrop-filter: var(--mat-blur-thin);
    box-shadow: var(--panel-shadow-sm);
  }
  .head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    padding: 7px 8px 6px;
  }
  .collapsed .head {
    padding: 0;
  }
  /* タブは選択中のカプセルが滑るセグメンテッドコントロール (app.css + segmented.ts) */
  .tabs :global(.seg-thumb) {
    background: rgba(255, 255, 255, 0.16);
  }
  .fold {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    background: transparent;
    border: none;
    color: var(--text-dim);
    font-family: var(--font-ui);
    font-size: 12px;
    font-weight: 590;
    cursor: pointer;
    padding: 7px 10px;
    border-radius: var(--pill);
    transition:
      color var(--dur-fast) var(--ease-out),
      transform var(--dur-press) var(--ease-out);
  }
  .fold:hover {
    color: var(--text-hi);
  }
  .fold:active {
    transform: scale(0.94);
  }
  .log {
    padding: 2px 13px 10px;
    font-size: 12px;
    font-weight: 400;
    line-height: 1.7;
    max-height: 168px;
    overflow-y: auto;
    overscroll-behavior: contain;
    /* 上端は罫線ではなくフェードで切る */
    mask-image: linear-gradient(to bottom, transparent 0, #000 8px);
  }
  .t {
    color: var(--text-dim);
    margin-right: 7px;
    font-variant-numeric: tabular-nums;
  }
  .line.hit {
    color: #6cb2ff;
  }
  .line.miss {
    color: #c69ba0;
  }
  .line.ev {
    color: var(--gold);
  }
  .line.opp {
    color: #ff9a9a;
  }
  .line.empty {
    color: var(--text-dim);
  }
  .telemetry {
    display: flex;
    flex-direction: column;
    gap: 4px;
    padding: 2px 13px 11px;
    font-size: 12px;
  }
  .telemetry .row {
    display: flex;
    justify-content: space-between;
    gap: 10px;
  }
  .telemetry b {
    color: var(--text-hi);
    font-weight: 700;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .telemetry .shot,
  .telemetry .shot b {
    color: #ffd98a;
  }

  @media (max-width: 760px) {
    .side {
      left: 6px;
      bottom: 64px;
      width: 250px;
    }
    .log {
      max-height: 108px;
      font-size: 9.5px;
      line-height: 1.5;
    }
    .telemetry {
      font-size: 9.5px;
    }
  }
</style>
