import { Spring, SPRING_UI, prefersReducedMotion } from './spring';

/**
 * .ui-seg にスプリング駆動のつまみを差し込む Svelte action。
 * X 位置と幅を別スプリングにして、途中で選択が変わっても現在値から追従させる
 * (目標値だけ差し替わるので割り込みが滑らかに繋がる)。
 */
export function segmented(node: HTMLElement) {
  const thumb = document.createElement('span');
  thumb.className = 'seg-thumb';
  node.prepend(thumb);

  const x = new Spring(0, (v) => (thumb.style.transform = `translateX(${v}px)`), SPRING_UI);
  const w = new Spring(0, (v) => (thumb.style.width = `${v}px`), SPRING_UI);
  let first = true;

  function sync(): void {
    const active = node.querySelector<HTMLElement>('button.active');
    node.dataset.empty = active ? 'false' : 'true';
    if (!active) return;
    const left = active.offsetLeft;
    const width = active.offsetWidth;
    if (first || prefersReducedMotion()) {
      // 初回は scale(0) 相当の"生まれ方"をさせない。所定の位置に置くだけ。
      x.jump(left);
      w.jump(width);
      first = false;
      return;
    }
    x.set(left);
    w.set(width);
  }

  sync();
  const mo = new MutationObserver(sync);
  mo.observe(node, { attributes: true, attributeFilter: ['class'], subtree: true });
  const ro = new ResizeObserver(sync);
  ro.observe(node);

  return {
    destroy(): void {
      mo.disconnect();
      ro.disconnect();
      x.stop();
      w.stop();
      thumb.remove();
    },
  };
}
