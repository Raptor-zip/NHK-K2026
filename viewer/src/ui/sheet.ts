import { Spring, SPRING_SHEET, VelocityTracker, project, rubberband, prefersReducedMotion } from './spring';

export interface SheetParams {
  /** ドラッグ軸。右からのシートは x、下からのシートは y。入りと出は同じ経路 (spatial consistency)。 */
  axis: 'x' | 'y';
  /** 背後を暗くするスクリム。ドラッグ量に 1:1 で追従させる。 */
  scrim?: HTMLElement | null;
  /** 実際に閉じる (DOM から外す) タイミングで呼ばれる。 */
  onDismiss: () => void;
}

/**
 * ドラッグで閉じられる Apple 流シート。
 *
 * - 指に 1:1 で追従し、掴んだ位置のオフセットを保つ
 * - 開く方向へ引っ張ると徐々に効かなくなる (rubber-banding)
 * - 離した瞬間の速度で着地点を予測し (momentum projection)、その速度をスプリングに引き渡す
 * - アニメーション中に掴み直せる (現在の表示値から続く)
 *
 * `node.dispatchEvent(new CustomEvent('sheet-close'))` で外から閉じられる。
 */
export function sheet(node: HTMLElement, params: SheetParams) {
  let { axis, scrim, onDismiss } = params;
  let dim = 1;
  let dragging = false;
  let grabOffset = 0;
  let pointerId = -1;
  const tracker = new VelocityTracker();

  const measure = (): void => {
    // 画面端の余白ぶん余計に逃がす。半分見えたまま消える、を避ける。
    dim = (axis === 'x' ? node.offsetWidth : node.offsetHeight) + 24;
  };

  const paint = (v: number): void => {
    node.style.transform = axis === 'x' ? `translate3d(${v}px,0,0)` : `translate3d(0,${v}px,0)`;
    if (scrim) {
      const p = Math.max(0, Math.min(1, 1 - v / dim));
      scrim.style.opacity = String(p);
    }
  };

  let pendingDismiss = false;
  // 閉じる向きのスプリングが着地したら DOM から外す。
  const spring = new Spring(0, paint, SPRING_SHEET, () => {
    if (!pendingDismiss) return;
    pendingDismiss = false;
    onDismiss();
  });

  measure();
  // 入場: 出ていく先と同じ経路から現れる。scale(0) からは決して出さない。
  spring.jump(prefersReducedMotion() ? 0 : dim);
  spring.set(0);

  function dismiss(velocity: number): void {
    pendingDismiss = true;
    spring.set(dim, { velocity });
  }

  function pos(ev: PointerEvent): number {
    return axis === 'x' ? ev.clientX : ev.clientY;
  }

  function onPointerDown(ev: PointerEvent): void {
    const target = ev.target as HTMLElement;
    if (!target.closest('[data-drag-handle]')) return;
    if (ev.button !== 0 && ev.pointerType === 'mouse') return;
    // 走っているスプリングを掴む。現在の表示値からそのまま指へ渡す。
    // 閉じかけていても掴み直せば取り消される (interruptible)。
    pendingDismiss = false;
    spring.stop();
    dragging = true;
    pointerId = ev.pointerId;
    node.setPointerCapture(ev.pointerId);
    measure();
    grabOffset = pos(ev) - spring.value; // 掴んだ位置を保つ (中心へスナップしない)
    tracker.clear();
    tracker.add(spring.value, ev.timeStamp / 1000);
  }

  function onPointerMove(ev: PointerEvent): void {
    if (!dragging || ev.pointerId !== pointerId) return;
    const raw = pos(ev) - grabOffset;
    // 開く方向 (負) へは抵抗を増やしながら少しだけ動く
    const v = raw >= 0 ? raw : -rubberband(-raw, dim);
    spring.jump(v);
    tracker.add(v, ev.timeStamp / 1000);
  }

  function onPointerUp(ev: PointerEvent): void {
    if (!dragging || ev.pointerId !== pointerId) return;
    dragging = false;
    node.releasePointerCapture(ev.pointerId);
    const velocity = tracker.velocity();
    // 離した点ではなく「velocity が連れて行く先」で判定する
    const projected = spring.value + project(velocity);
    if (projected > dim * 0.4) dismiss(velocity);
    else spring.set(0, { velocity }); // 戻すときも速度を継承 = 継ぎ目が出ない
  }

  const onClose = (): void => dismiss(0);
  const onResize = (): void => {
    measure();
    if (!dragging) paint(spring.value);
  };

  node.addEventListener('pointerdown', onPointerDown);
  node.addEventListener('pointermove', onPointerMove);
  node.addEventListener('pointerup', onPointerUp);
  node.addEventListener('pointercancel', onPointerUp);
  node.addEventListener('sheet-close', onClose);
  window.addEventListener('resize', onResize);

  return {
    update(next: SheetParams): void {
      axis = next.axis;
      scrim = next.scrim;
      onDismiss = next.onDismiss;
      onResize();
    },
    destroy(): void {
      spring.stop();
      node.removeEventListener('pointerdown', onPointerDown);
      node.removeEventListener('pointermove', onPointerMove);
      node.removeEventListener('pointerup', onPointerUp);
      node.removeEventListener('pointercancel', onPointerUp);
      node.removeEventListener('sheet-close', onClose);
      window.removeEventListener('resize', onResize);
    },
  };
}
