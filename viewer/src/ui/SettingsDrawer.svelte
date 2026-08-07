<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faXmark } from '@fortawesome/free-solid-svg-icons';
  import { prefs } from './hudStore';
  import { sheet } from './sheet';
  import { clonedParams, type StrategyVariant } from '../config/params';

  export let open = false;
  export let onReset: () => void;

  const variants: Array<[StrategyVariant, string]> = [
    ['standard', '標準 (ボーナス走→旗+スマート切替)'],
    ['flagOnly', '旗オンリー'],
    ['bucketFocus', '移動バケツ集中'],
    ['fixedOnly', '固定バケツ・机のみ'],
    ['optimal', '最適 (期待値ソルバ)'],
  ];

  let sheetEl: HTMLElement;
  let scrimEl: HTMLElement;
  // 広い画面は右から、狭い画面は下から。出るときも同じ方向へ帰す。
  const narrow = matchMedia('(max-width: 760px)');
  let axis: 'x' | 'y' = narrow.matches ? 'y' : 'x';
  narrow.addEventListener('change', (e) => (axis = e.matches ? 'y' : 'x'));

  /** 閉じる要求。実際の unmount はシートが着地してから (dismissed)。 */
  function close(): void {
    sheetEl?.dispatchEvent(new CustomEvent('sheet-close'));
  }

  function dismissed(): void {
    open = false;
  }

  function onKeydown(ev: KeyboardEvent): void {
    if (ev.key === 'Escape' && open) close();
  }

  function resetParams(): void {
    prefs.update((p) => ({
      ...p,
      params: clonedParams(),
      oppSkill: 0.85,
      blueCycleScale: 1,
      redCycleScale: 1.15,
      blueProbMul: 1,
      redProbMul: 1,
      blueDriveMul: 1,
      redDriveMul: 1,
      blueMuzzleY: 1.5,
      redMuzzleY: 1.2,
    }));
  }
</script>

<svelte:window on:keydown={onKeydown} />

{#if open}
  <div class="backdrop" bind:this={scrimEl} on:pointerdown={close} aria-hidden="true"></div>
  <aside
    class="drawer panel-glass"
    class:bottom={axis === 'y'}
    aria-label="設定"
    bind:this={sheetEl}
    use:sheet={{ axis, scrim: scrimEl, onDismiss: dismissed }}>
    <header data-drag-handle>
      <div class="grabber" aria-hidden="true"></div>
      <h2>設定・パラメータ</h2>
      <button class="ui-btn close icon-only" on:click={close} aria-label="閉じる">
        <span class="ui-icon" aria-hidden="true"><FontAwesomeIcon icon={faXmark} /></span>
      </button>
    </header>

    <div class="body">
      <details open>
        <summary>対戦カード</summary>
        <div class="sec">
          <label class="field wide">
            <span>青チーム戦略</span>
            <select class="ui-select" bind:value={$prefs.variant} on:change={onReset}>
              {#each variants as [v, label] (v)}<option value={v}>{label}</option>{/each}
            </select>
          </label>
          <label class="field wide">
            <span>赤チーム戦略</span>
            <select class="ui-select" bind:value={$prefs.redVariant} on:change={onReset}>
              {#each variants as [v, label] (v)}<option value={v}>{label}</option>{/each}
            </select>
          </label>
        </div>
      </details>

      <details>
        <summary>表示・デバッグ</summary>
        <div class="sec">
          <label class="field toggle">
            <span>選択チームLiDAR点群</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.showLidar} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>選択チーム照準カメラ</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.showCam} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>選択チームA*経路</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.showPath} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>半自動指令パッド</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.showPad} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>自分の顔 (要カメラ)</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.faceCam} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>機体をCAD実形状で描く</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.cadRobot} /><span class="knob"></span></span>
          </label>
          <label class="field">
            <span>投擲角ばらつき°</span>
            <input class="ui-num" type="number" min="0.5" max="8" step="0.1" bind:value={$prefs.params.aimSpreadDeg} />
          </label>
        </div>
      </details>

      <details>
        <summary><span class="dot blue"></span>青チーム</summary>
        <div class="sec">
          <label class="field">
            <span>足回り</span>
            <select class="ui-select" bind:value={$prefs.blueSwerve}>
              <option value={false}>メカナム</option>
              <option value={true}>独ステ</option>
            </select>
          </label>
          <label class="field"><span>走行倍率</span><input class="ui-num" type="number" min="0.4" max="2.5" step="0.05" bind:value={$prefs.blueDriveMul} /></label>
          <label class="field"><span>射出サイクル</span><input class="ui-num" type="number" min="0.4" max="3" step="0.05" bind:value={$prefs.blueCycleScale} /></label>
          <label class="field"><span>命中倍率</span><input class="ui-num" type="number" min="0.2" max="1.6" step="0.05" bind:value={$prefs.blueProbMul} /></label>
          <label class="field"><span>砲口高 m</span><input class="ui-num" type="number" min="0.8" max="1.8" step="0.02" bind:value={$prefs.blueMuzzleY} /></label>
          <label class="field"><span>バケツ高 m</span><input class="ui-num" type="number" min="1.2" max="2.1" step="0.02" bind:value={$prefs.params.bucketTopY.blue} /></label>
        </div>
      </details>

      <details>
        <summary><span class="dot red"></span>赤チーム</summary>
        <div class="sec">
          <label class="field">
            <span>足回り</span>
            <select class="ui-select" bind:value={$prefs.redSwerve}>
              <option value={false}>メカナム</option>
              <option value={true}>独ステ</option>
            </select>
          </label>
          <label class="field"><span>走行倍率</span><input class="ui-num" type="number" min="0.4" max="2.5" step="0.05" bind:value={$prefs.redDriveMul} /></label>
          <label class="field"><span>射出サイクル</span><input class="ui-num" type="number" min="0.4" max="3.5" step="0.05" bind:value={$prefs.redCycleScale} /></label>
          <label class="field"><span>命中倍率</span><input class="ui-num" type="number" min="0.2" max="1.6" step="0.05" bind:value={$prefs.redProbMul} /></label>
          <label class="field"><span>赤チーム腕前</span><input class="ui-num" type="number" min="0.2" max="1.2" step="0.05" bind:value={$prefs.oppSkill} /></label>
          <label class="field"><span>砲口高 m</span><input class="ui-num" type="number" min="0.8" max="1.8" step="0.02" bind:value={$prefs.redMuzzleY} /></label>
          <label class="field"><span>バケツ高 m</span><input class="ui-num" type="number" min="1.2" max="2.1" step="0.02" bind:value={$prefs.params.bucketTopY.red} /></label>
        </div>
      </details>

      <details>
        <summary>走行・射出</summary>
        <div class="sec">
          <label class="field"><span>メカナム最高速 m/s</span><input class="ui-num" type="number" min="0.5" max="4" step="0.05" bind:value={$prefs.params.drive.vmax} /></label>
          <label class="field"><span>メカナム加速度 m/s²</span><input class="ui-num" type="number" min="0.4" max="5" step="0.05" bind:value={$prefs.params.drive.acc} /></label>
          <label class="field"><span>独ステ最高速 m/s</span><input class="ui-num" type="number" min="0.5" max="4.5" step="0.05" bind:value={$prefs.params.swerveDrive.vmax} /></label>
          <label class="field"><span>独ステ加速度 m/s²</span><input class="ui-num" type="number" min="0.4" max="5" step="0.05" bind:value={$prefs.params.swerveDrive.acc} /></label>
          <label class="field"><span>照準 s</span><input class="ui-num" type="number" min="0.2" max="4" step="0.05" bind:value={$prefs.params.shooter.aim} /></label>
          <label class="field"><span>送給 s</span><input class="ui-num" type="number" min="0.1" max="3" step="0.05" bind:value={$prefs.params.shooter.feed} /></label>
          <label class="field"><span>復帰 s</span><input class="ui-num" type="number" min="0.1" max="3" step="0.05" bind:value={$prefs.params.shooter.recover} /></label>
          <label class="field"><span>スピン rpm/s</span><input class="ui-num" type="number" min="500" max="7000" step="100" bind:value={$prefs.params.shooter.spinupRpmPerSec} /></label>
          <label class="field"><span>ローラー径 m</span><input class="ui-num" type="number" min="0.04" max="0.2" step="0.005" bind:value={$prefs.params.shooter.rollerDia} /></label>
          <label class="field"><span>ジャム率</span><input class="ui-num" type="number" min="0" max="0.25" step="0.005" bind:value={$prefs.params.shooter.jamProb} /></label>
        </div>
      </details>

      <details>
        <summary>命中率</summary>
        <div class="sec">
          <label class="field"><span>旗</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.flag} /></label>
          <label class="field"><span>旗ロブ</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.flagLob} /></label>
          <label class="field"><span>静止バケツ</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.bucketStill} /></label>
          <label class="field"><span>移動バケツ</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.bucketMove} /></label>
          <label class="field"><span>机</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.desk} /></label>
          <label class="field"><span>固定バケツ</span><input class="ui-num" type="number" min="0" max="1" step="0.01" bind:value={$prefs.params.probs.fixedB} /></label>
        </div>
      </details>

      <details>
        <summary>雑巾物理</summary>
        <div class="sec">
          <label class="field"><span>通常 kg</span><input class="ui-num" type="number" min="0.02" max="0.12" step="0.001" bind:value={$prefs.params.rag.m} /></label>
          <label class="field"><span>通常 抗力</span><input class="ui-num" type="number" min="0" max="0.05" step="0.001" bind:value={$prefs.params.rag.k} /></label>
          <label class="field"><span>通常 揚力</span><input class="ui-num" type="number" min="0" max="1.2" step="0.01" bind:value={$prefs.params.rag.lift} /></label>
          <label class="field"><span>スーパー kg</span><input class="ui-num" type="number" min="0.06" max="0.3" step="0.005" bind:value={$prefs.params.superRag.m} /></label>
          <label class="field"><span>スーパー 抗力</span><input class="ui-num" type="number" min="0" max="0.08" step="0.001" bind:value={$prefs.params.superRag.k} /></label>
          <label class="field"><span>スーパー 揚力</span><input class="ui-num" type="number" min="0" max="1.2" step="0.01" bind:value={$prefs.params.superRag.lift} /></label>
        </div>
      </details>

      <details>
        <summary>ルール・ハザード</summary>
        <div class="sec">
          <label class="field"><span>旗の目安容量 (枚)</span><input class="ui-num" type="number" min="1" max="40" step="1" bind:value={$prefs.params.flagCapacity} /></label>
          <label class="field"><span>旗の落下率 /秒·枚</span><input class="ui-num" type="number" min="0" max="0.5" step="0.01" bind:value={$prefs.params.flagFallPerSec} /></label>
          <label class="field toggle">
            <span>雑巾踏みハザード</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.params.ragHazard} /><span class="knob"></span></span>
          </label>
          <label class="field toggle">
            <span>床雑巾を避ける経路</span>
            <span class="ui-switch"><input type="checkbox" bind:checked={$prefs.params.avoidFloorRags} /><span class="knob"></span></span>
          </label>
        </div>
      </details>
    </div>

    <footer>
      <button class="ui-btn primary" on:click={onReset}>適用してリセット</button>
      <button class="ui-btn" on:click={resetParams}>初期値に戻す</button>
    </footer>
  </aside>
{/if}

<style>
  /* モーダルな作業なのでスクリムで背後を落とす。不透明度はドラッグに 1:1 で追従 (sheet.ts)。 */
  .backdrop {
    position: fixed;
    inset: 0;
    z-index: 34;
    background: rgba(0, 0, 0, 0.4);
    opacity: 0;
  }
  .drawer {
    position: fixed;
    top: 10px;
    right: 10px;
    bottom: 10px;
    z-index: 35;
    width: min(94vw, 380px);
    display: flex;
    flex-direction: column;
    color: var(--text-mid);
    touch-action: none;
    will-change: transform;
  }
  /* 狭い画面は下からのシート。画面下端に張り付き、上端だけ丸める。 */
  .drawer.bottom {
    top: auto;
    left: 0;
    right: 0;
    bottom: 0;
    width: 100%;
    max-height: 86vh;
    border-radius: 22px 22px 0 0;
    padding-bottom: env(safe-area-inset-bottom);
  }
  header {
    position: relative;
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 14px 12px;
    cursor: grab;
  }
  header:active {
    cursor: grabbing;
  }
  /* 掴めることを示すつまみ。下シートのときだけ出す。 */
  .grabber {
    display: none;
    position: absolute;
    top: 6px;
    left: 50%;
    transform: translateX(-50%);
    width: 36px;
    height: 5px;
    border-radius: var(--pill);
    background: rgba(255, 255, 255, 0.3);
  }
  .drawer.bottom .grabber {
    display: block;
  }
  .drawer.bottom header {
    padding-top: 18px;
  }
  h2 {
    margin: 0;
    font-size: 17px;
    font-weight: 640;
    letter-spacing: var(--track-title);
    color: var(--text-hi);
  }
  .close {
    width: 30px;
    height: 30px;
    min-width: 30px;
    padding: 0;
    border-radius: 50%;
    color: var(--text-mid);
  }
  .body {
    flex: 1;
    overflow-y: auto;
    padding: 0 16px 12px;
    overscroll-behavior: contain;
    touch-action: pan-y;
    /* 固定ヘッダとの境目は 1px の罫線ではなく、重なる所だけのフェード */
    mask-image: linear-gradient(to bottom, transparent 0, #000 10px);
  }
  details {
    border-bottom: 0.5px solid rgba(255, 255, 255, 0.08);
  }
  summary {
    display: flex;
    align-items: center;
    gap: 8px;
    cursor: pointer;
    font-size: 13px;
    font-weight: 590;
    letter-spacing: var(--track-body);
    color: var(--text-hi);
    padding: 12px 2px;
    list-style: none;
  }
  summary::-webkit-details-marker {
    display: none;
  }
  summary::after {
    content: '';
    margin-left: auto;
    width: 7px;
    height: 7px;
    border-right: 1.6px solid var(--text-dim);
    border-bottom: 1.6px solid var(--text-dim);
    transform: rotate(45deg) translate(-2px, -2px);
    transition: transform var(--dur) var(--ease-spring);
  }
  details[open] summary::after {
    transform: rotate(225deg) translate(-2px, -2px);
  }
  .dot {
    width: 9px;
    height: 9px;
    border-radius: 50%;
    flex: none;
  }
  .dot.blue {
    background: var(--blue-hi);
  }
  .dot.red {
    background: var(--red-hi);
  }
  .sec {
    display: flex;
    flex-direction: column;
    gap: 9px;
    padding: 0 2px 14px;
  }
  .field {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    font-size: 13px;
  }
  .field > span:first-child {
    color: var(--text-mid);
  }
  .field.wide {
    flex-direction: column;
    align-items: stretch;
    gap: 5px;
  }
  .field.wide .ui-select {
    width: 100%;
  }
  footer {
    display: flex;
    gap: 8px;
    padding: 12px 16px 14px;
    border-top: 0.5px solid rgba(255, 255, 255, 0.08);
  }
  footer .ui-btn {
    flex: 1;
    padding: 11px 10px;
    border-radius: var(--ctl-radius);
  }
</style>
