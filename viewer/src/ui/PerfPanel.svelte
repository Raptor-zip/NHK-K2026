<script lang="ts">
  import { perf } from './hudStore';

  $: p = $perf;
  $: load = p?.budgetPct ?? 0;
  $: state = load > 85 ? 'hot' : load > 65 ? 'warn' : 'ok';

  function n(v: number | undefined, digits = 1): string {
    return v === undefined ? '--' : v.toFixed(digits);
  }
</script>

<details class="perf {state}">
  <summary>
    <span>perf</span>
    <b>{n(p?.fps, 0)}fps</b>
    <span>{n(p?.frameMs)}ms</span>
  </summary>
  <div class="grid">
    <span>frame</span><b>{n(p?.frameMs)}ms</b>
    <span>sim</span><b>{n(p?.simMs)}ms</b>
    <span>visual</span><b>{n(p?.visualMs)}ms</b>
    <span>render</span><b>{n(p?.renderMs)}ms</b>
    <span>budget</span><b>{n(p?.budgetPct, 0)}%</b>
    <span>headroom</span><b>{n(p?.headroomMs)}ms</b>
  </div>
  <div class="foot">draw {p?.drawCalls ?? 0} / tri {p?.triangles ?? 0}</div>
</details>

<style>
  .perf {
    position: fixed;
    top: max(12px, env(safe-area-inset-top));
    right: 12px;
    z-index: 21;
    width: max-content;
    max-width: 220px;
    background: var(--mat-thin-bg);
    backdrop-filter: var(--mat-blur-thin);
    -webkit-backdrop-filter: var(--mat-blur-thin);
    border: 0.5px solid var(--panel-border);
    border-radius: var(--pill);
    padding: 3px 10px;
    color: var(--text-dim);
    font-family: var(--font-ui);
    font-size: 10px;
    font-variant-numeric: tabular-nums;
    user-select: none;
    opacity: 0.78;
    transition: opacity var(--dur-fast) var(--ease-out);
  }
  .perf:hover {
    opacity: 1;
  }
  .perf[open] {
    width: 196px;
    background: var(--panel-bg);
    backdrop-filter: var(--mat-blur-regular);
    -webkit-backdrop-filter: var(--mat-blur-regular);
    border-radius: 14px;
    opacity: 1;
    padding: 7px 10px 8px;
  }
  summary {
    display: flex;
    gap: 6px;
    align-items: center;
    list-style: none;
    cursor: pointer;
    letter-spacing: 0;
  }
  summary::-webkit-details-marker {
    display: none;
  }
  summary b {
    color: rgba(125, 255, 165, 0.76);
    font-weight: 700;
  }
  .warn summary b {
    color: rgba(255, 210, 62, 0.78);
  }
  .hot summary b {
    color: rgba(255, 112, 112, 0.78);
  }
  .grid {
    display: grid;
    grid-template-columns: 1fr auto 1fr auto;
    gap: 2px 8px;
    align-items: center;
    margin-top: 5px;
  }
  span,
  .foot {
    color: rgba(143, 155, 173, 0.76);
  }
  b {
    color: rgba(238, 243, 248, 0.86);
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  .foot {
    margin-top: 4px;
    border-top: 1px solid rgba(143, 155, 173, 0.22);
    padding-top: 3px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  @media (max-width: 760px) {
    .perf {
      top: 6px;
      right: 6px;
      padding: 1px 5px;
      font-size: 7.5px;
      opacity: 0.55;
    }
    .grid {
      grid-template-columns: 1fr auto 1fr auto;
      gap: 1px 5px;
    }
    .perf[open] {
      width: 148px;
      padding: 4px 5px;
      opacity: 0.82;
    }
    .foot {
      display: none;
    }
  }
  @media (max-width: 520px) and (orientation: portrait) {
    .perf {
      /* 縦持ちではスコアバグと重なるため、その下へ */
      top: 68px;
      font-size: 7px;
    }
  }
</style>
