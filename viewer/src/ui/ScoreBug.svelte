<script lang="ts">
  import { tick } from 'svelte';
  import { hud, teamNames } from './hudStore';

  /** multicam などで映像を邪魔しないコンパクト表示 (時計のみ) */
  export let compact = false;

  let expanded = false;
  let editing: 'red' | 'blue' | null = null;
  let draft = '';
  let nameInput: HTMLInputElement;

  $: remain = $hud?.remain ?? 180;
  $: clockText = `${Math.floor(remain / 60)}:${String(Math.max(0, Math.ceil(remain % 60)) % 60).padStart(2, '0')}`;
  $: urgent = remain <= 10 && !$hud?.over;
  // 競技終了時、勝利チーム側を金色に
  $: winner =
    $hud?.over && $hud.score.red !== $hud.score.blue
      ? $hud.score.red > $hud.score.blue
        ? 'red'
        : 'blue'
      : null;

  const sides = ['red', 'blue'] as const;
  type BdKey = 'moving' | 'flag' | 'desk' | 'fixed' | 'superHits';
  const rows: Array<[BdKey, string]> = [
    ['moving', '移動バケツ'],
    ['flag', '旗'],
    ['desk', '机'],
    ['fixed', '固定バケツ'],
    ['superHits', 'スーパー命中'],
  ];

  function heldText(side: 'red' | 'blue'): string {
    const h = side === 'red' ? ($hud?.held.red ?? 0) : ($hud?.held.blue ?? 0);
    const s = side === 'red' ? ($hud?.held.redSuper ?? 0) : ($hud?.held.blueSuper ?? 0);
    return `${h}枚${s > 0 ? ` +S${s}` : ''}`;
  }

  function startEdit(side: 'red' | 'blue'): void {
    draft = side === 'red' ? $teamNames.red : $teamNames.blue;
    editing = side;
    void tick().then(() => nameInput?.select());
  }

  function commit(): void {
    if (!editing) return;
    const side = editing;
    const next = draft.trim() || (side === 'red' ? '赤チーム' : '青チーム');
    teamNames.update((v) => ({ ...v, [side]: next }));
    editing = null;
  }

  function onNameKeydown(ev: KeyboardEvent): void {
    if (ev.key === 'Enter') commit();
    else if (ev.key === 'Escape') editing = null;
  }
</script>

{#if compact}
  <div class="bug compact panel-glass">
    <div class="clock" class:urgent>{clockText}</div>
  </div>
{:else}
  <div class="wrap">
    <div class="bug panel-glass">
      {#each sides as side (side)}
        <div class="team {side}" class:win={winner === side} style:order={side === 'red' ? 0 : 2}>
          {#if winner === side}<div class="winlabel">WINNER</div>{/if}
          <button
            class="name"
            on:click={() => startEdit(side)}
            aria-label={`${side === 'red' ? $teamNames.red : $teamNames.blue} のチーム名を変更`}
          >
            {#if editing === side}
              <input
                bind:this={nameInput}
                bind:value={draft}
                on:click|stopPropagation
                on:blur={commit}
                on:keydown={onNameKeydown}
                maxlength="18"
              />
            {:else}
              {side === 'red' ? $teamNames.red : $teamNames.blue}
            {/if}
          </button>
          <div class="score">{$hud?.score[side] ?? 0}</div>
          <div class="status">{$hud?.status[side] ?? ''}</div>
        </div>
      {/each}
      <div class="center" style:order={1}>
        <div class="clock" class:urgent>{clockText}</div>
        <button class="expand" class:on={expanded} on:click={() => (expanded = !expanded)} aria-expanded={expanded}>
          <span>{expanded ? '詳細を閉じる' : '得点詳細'}</span>
          <span class="chev" aria-hidden="true"></span>
        </button>
      </div>
    </div>

    <div class="detail-wrap" class:open={expanded}>
      <div class="detail panel-glass">
        <div class="col red">
          {#each rows as [key, label] (key)}
            <div class="row"><span>{label}</span><b>×{$hud?.breakdown.red[key] ?? 0}</b></div>
          {/each}
          <div class="row bonus" class:on={$hud?.breakdown.red.bonus}>
            <span>全ゴールボーナス</span><b>{$hud?.breakdown.red.bonus ? '+100' : 'ー'}</b>
          </div>
          <div class="row held"><span>保持雑巾</span><b>{heldText('red')}</b></div>
        </div>
        <div class="sep"></div>
        <div class="col blue">
          {#each rows as [key, label] (key)}
            <div class="row"><b>×{$hud?.breakdown.blue[key] ?? 0}</b><span>{label}</span></div>
          {/each}
          <div class="row bonus" class:on={$hud?.breakdown.blue.bonus}>
            <b>{$hud?.breakdown.blue.bonus ? '+100' : 'ー'}</b><span>全ゴールボーナス</span>
          </div>
          <div class="row held"><b>{heldText('blue')}</b><span>保持雑巾</span></div>
        </div>
      </div>
    </div>
  </div>
{/if}

<style>
  .wrap {
    position: fixed;
    top: max(10px, env(safe-area-inset-top));
    left: 50%;
    transform: translateX(-50%);
    z-index: 20;
    display: flex;
    flex-direction: column;
    align-items: center;
    user-select: none;
    max-width: 96vw;
  }
  .bug {
    display: flex;
    align-items: stretch;
    overflow: hidden;
  }
  .bug.compact {
    position: fixed;
    top: max(10px, env(safe-area-inset-top));
    left: 50%;
    transform: translateX(-50%);
    z-index: 20;
    padding: 8px 22px 6px;
    border-radius: var(--pill);
  }

  /* チームブロック */
  .team {
    position: relative;
    display: grid;
    grid-template-rows: auto 1fr auto;
    width: 190px;
  }
  .team.red {
    border-top: 3px solid var(--red);
  }
  .team.blue {
    border-top: 3px solid var(--blue);
  }
  /* 勝者は色で祝う。動きは足さない (見終わった後のUIに動きは要らない) */
  .team.win {
    box-shadow: inset 0 0 30px rgba(255, 214, 10, 0.22);
  }
  .team.win .score {
    color: var(--gold);
  }
  .winlabel {
    position: absolute;
    top: 3px;
    left: 50%;
    transform: translateX(-50%);
    background: var(--gold);
    color: #241d00;
    font-family: var(--font-ui);
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.12em;
    padding: 2px 10px;
    border-radius: 0 0 8px 8px;
    z-index: 1;
  }
  .name {
    display: block;
    width: 100%;
    border: none;
    color: #fff;
    font-family: var(--font-ui);
    font-weight: 640;
    font-size: 15px;
    letter-spacing: var(--track-title);
    line-height: 1.3;
    text-align: center;
    padding: 6px 8px 4px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    cursor: pointer;
    background: transparent;
    transition: transform var(--dur-press) var(--ease-out);
  }
  .name:active {
    transform: scale(0.97);
  }
  /* 色は不透明なレイヤに乗せる。半透明の前景に色を置くと沈む。 */
  .red .name {
    background: linear-gradient(180deg, rgba(255, 69, 58, 0.95), rgba(190, 40, 32, 0.9));
  }
  .blue .name {
    background: linear-gradient(180deg, rgba(10, 132, 255, 0.95), rgba(6, 90, 180, 0.9));
  }
  .name input {
    width: 92%;
    min-width: 0;
    border: 1px solid rgba(255, 255, 255, 0.55);
    border-radius: 4px;
    background: rgba(0, 0, 0, 0.42);
    color: #fff;
    font: inherit;
    text-align: center;
    user-select: text;
    -webkit-user-select: text;
  }
  /* 大きい数字ほどトラッキングを詰める。桁が変わっても揺れない tabular-nums。 */
  .score {
    font-family: var(--font-display);
    font-size: 46px;
    font-weight: 700;
    line-height: 1.02;
    text-align: center;
    color: var(--text-hi);
    padding: 2px 6px 0;
    letter-spacing: var(--track-display);
    font-variant-numeric: tabular-nums;
  }
  .status {
    color: var(--gold);
    font-size: 10.5px;
    font-weight: 590;
    letter-spacing: var(--track-caption);
    text-align: center;
    padding: 1px 6px 5px;
    min-height: 15px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  /* 中央: 時計 + 詳細トグル */
  .center {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 2px;
    padding: 9px 18px 6px;
    border-left: 0.5px solid rgba(255, 255, 255, 0.1);
    border-right: 0.5px solid rgba(255, 255, 255, 0.1);
    background: rgba(0, 0, 0, 0.22);
  }
  .clock {
    font-family: var(--font-digit);
    font-weight: 600;
    font-size: 34px;
    color: var(--text-hi);
    font-variant-numeric: tabular-nums;
    letter-spacing: var(--track-title);
    line-height: 1;
    transition: color var(--dur) var(--ease-out);
  }
  .clock.urgent {
    color: var(--bad);
  }
  .expand {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    background: transparent;
    border: none;
    color: var(--text-dim);
    font-family: var(--font-ui);
    font-size: 10.5px;
    font-weight: 590;
    cursor: pointer;
    padding: 3px 6px;
    border-radius: var(--pill);
    transition:
      color var(--dur-fast) var(--ease-out),
      transform var(--dur-press) var(--ease-out);
  }
  .expand:hover {
    color: var(--text-hi);
  }
  .expand:active {
    transform: scale(0.94);
  }
  .expand .chev {
    width: 6px;
    height: 6px;
    border-right: 1.5px solid currentColor;
    border-bottom: 1.5px solid currentColor;
    transform: rotate(45deg) translate(-1px, -1px);
    transition: transform var(--dur) var(--ease-spring);
  }
  .expand.on .chev {
    transform: rotate(225deg) translate(-1px, -1px);
  }

  /* 詳細ストリップ: 高さ 0fr→1fr をスプリング曲線で開く (アコーディオン) */
  .detail-wrap {
    display: grid;
    grid-template-rows: 0fr;
    opacity: 0;
    transform: translateY(-6px);
    transition:
      grid-template-rows var(--dur) var(--ease-standard),
      opacity var(--dur-fast) var(--ease-out),
      transform var(--dur) var(--ease-standard);
  }
  .detail-wrap.open {
    grid-template-rows: 1fr;
    opacity: 1;
    transform: none;
  }
  .detail-wrap > .detail {
    box-sizing: border-box;
    overflow: hidden;
    min-height: 0;
  }
  .detail-wrap:not(.open) {
    pointer-events: none;
  }
  .detail-wrap.open {
    padding-top: 6px;
  }
  .detail {
    display: flex;
    gap: 12px;
    padding: 8px 14px;
    font-size: 11.5px;
    font-weight: 700;
    color: var(--text-mid);
  }
  .detail .col {
    display: flex;
    flex-direction: column;
    gap: 2px;
    width: 170px;
  }
  .detail .row {
    display: flex;
    justify-content: space-between;
    gap: 10px;
    line-height: 1.5;
  }
  .detail b {
    color: var(--text-hi);
    font-variant-numeric: tabular-nums;
  }
  .detail .col.red .row b {
    color: var(--red-hi);
  }
  .detail .col.blue .row b {
    color: var(--blue-hi);
  }
  .detail .row.bonus {
    color: var(--text-dim);
  }
  .detail .row.bonus b {
    color: var(--text-dim);
  }
  .detail .row.bonus.on,
  .detail .row.bonus.on b {
    color: var(--gold);
  }
  .detail .row.held {
    border-top: 1px solid var(--panel-border);
    margin-top: 3px;
    padding-top: 3px;
  }
  .detail .row.held b {
    color: #8fd0ff;
  }
  .sep {
    width: 1px;
    background: var(--panel-border);
  }

  @media (max-width: 760px) {
    .wrap {
      top: 6px;
    }
    .team {
      width: 118px;
    }
    .name {
      font-size: 11px;
      padding: 3px 6px 2px;
    }
    .score {
      font-size: 30px;
    }
    .status {
      font-size: 8px;
      min-height: 11px;
    }
    .center {
      padding: 6px 10px 4px;
    }
    .clock {
      font-size: 22px;
    }
    .expand {
      font-size: 8.5px;
    }
    .detail {
      font-size: 9.5px;
      padding: 6px 10px;
    }
    .detail .col {
      width: 124px;
    }
  }
  @media (max-width: 520px) and (orientation: portrait) {
    .team {
      width: 96px;
    }
    .score {
      font-size: 25px;
    }
    .clock {
      font-size: 19px;
    }
  }
</style>
