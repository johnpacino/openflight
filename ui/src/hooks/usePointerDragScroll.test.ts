import { describe, expect, it } from 'vitest';
import {
  DRAG_SCROLL_SELECTOR,
  dragScrollTop,
  shouldStartPointerDrag,
} from './usePointerDragScroll';

describe('pointer drag scrolling', () => {
  it('moves content opposite the pointer drag', () => {
    expect(dragScrollTop(120, 100, 140)).toBe(80);
    expect(dragScrollTop(120, 100, 60)).toBe(160);
  });

  it('only handles primary mouse-emulated drags', () => {
    expect(shouldStartPointerDrag({ button: 0, isPrimary: true, pointerType: 'mouse' })).toBe(true);
    expect(shouldStartPointerDrag({ button: 1, isPrimary: true, pointerType: 'mouse' })).toBe(false);
    expect(shouldStartPointerDrag({ button: 0, isPrimary: false, pointerType: 'mouse' })).toBe(false);
    expect(shouldStartPointerDrag({ button: 0, isPrimary: true, pointerType: 'touch' })).toBe(false);
  });

  it('registers every shared scroll region, including optional camera panels', () => {
    for (const className of [
      'shot-list__rows',
      'debug-panel__tab-content',
      'trigger-history',
      'club-select__panel',
      'display-mode',
      'camera-settings',
      'camera-feed__workspace',
    ]) {
      expect(DRAG_SCROLL_SELECTOR).toContain(`.${className}`);
    }
  });
});
