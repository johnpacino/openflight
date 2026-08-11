import { useEffect } from 'react';

export const DRAG_SCROLL_SELECTOR = [
  '.shot-list__rows',
  '.debug-panel__tab-content',
  '.trigger-history',
  '.club-select__panel',
  '.display-mode',
  '.camera-settings',
  '.camera-feed__workspace',
].join(', ');

const DRAG_THRESHOLD_PX = 6;
const TEXT_INPUT_SELECTOR = "input, textarea, select, [contenteditable='true']";

type PointerStart = Pick<PointerEvent, 'button' | 'isPrimary' | 'pointerType'>;

export function shouldStartPointerDrag(event: PointerStart): boolean {
  return event.pointerType === 'mouse' && event.isPrimary && event.button === 0;
}

export function dragScrollTop(startScrollTop: number, startY: number, currentY: number): number {
  return startScrollTop + startY - currentY;
}

function findScrollableRegion(target: EventTarget | null): HTMLElement | null {
  if (!(target instanceof Element) || target.closest(TEXT_INPUT_SELECTOR)) {
    return null;
  }

  let region = target.closest<HTMLElement>(DRAG_SCROLL_SELECTOR);
  while (region && region.scrollHeight <= region.clientHeight + 1) {
    region = region.parentElement?.closest<HTMLElement>(DRAG_SCROLL_SELECTOR) ?? null;
  }
  return region;
}

export function usePointerDragScroll(): void {
  useEffect(() => {
    let pointerId: number | null = null;
    let region: HTMLElement | null = null;
    let startX = 0;
    let startY = 0;
    let startScrollTop = 0;
    let dragging = false;

    const reset = () => {
      pointerId = null;
      region = null;
      dragging = false;
    };

    const handlePointerDown = (event: PointerEvent) => {
      if (!shouldStartPointerDrag(event)) {
        return;
      }

      const scrollRegion = findScrollableRegion(event.target);
      if (!scrollRegion) {
        return;
      }

      pointerId = event.pointerId;
      region = scrollRegion;
      startX = event.clientX;
      startY = event.clientY;
      startScrollTop = scrollRegion.scrollTop;
      dragging = false;
    };

    const handlePointerMove = (event: PointerEvent) => {
      if (event.pointerId !== pointerId || !region) {
        return;
      }

      if (!dragging) {
        const distance = Math.hypot(event.clientX - startX, event.clientY - startY);
        if (distance < DRAG_THRESHOLD_PX) {
          return;
        }
        dragging = true;
      }

      event.preventDefault();
      region.scrollTop = dragScrollTop(startScrollTop, startY, event.clientY);
    };

    const handlePointerUp = (event: PointerEvent) => {
      if (event.pointerId !== pointerId) {
        return;
      }

      const draggedRegion = region;
      const shouldSuppressClick = dragging;
      reset();

      if (!shouldSuppressClick || !draggedRegion) {
        return;
      }

      const suppressDragClick = (clickEvent: MouseEvent) => {
        if (clickEvent.target instanceof Node && draggedRegion.contains(clickEvent.target)) {
          clickEvent.preventDefault();
          clickEvent.stopImmediatePropagation();
        }
      };
      document.addEventListener('click', suppressDragClick, { capture: true, once: true });
      window.setTimeout(() => document.removeEventListener('click', suppressDragClick, true), 0);
    };

    const handlePointerCancel = (event: PointerEvent) => {
      if (event.pointerId === pointerId) {
        reset();
      }
    };

    document.addEventListener('pointerdown', handlePointerDown, true);
    document.addEventListener('pointermove', handlePointerMove, { capture: true, passive: false });
    document.addEventListener('pointerup', handlePointerUp, true);
    document.addEventListener('pointercancel', handlePointerCancel, true);

    return () => {
      document.removeEventListener('pointerdown', handlePointerDown, true);
      document.removeEventListener('pointermove', handlePointerMove, true);
      document.removeEventListener('pointerup', handlePointerUp, true);
      document.removeEventListener('pointercancel', handlePointerCancel, true);
    };
  }, []);
}
