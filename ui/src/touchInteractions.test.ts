/// <reference types="node" />

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';

const appCss = readFileSync(fileURLToPath(new URL('./App.css', import.meta.url)), 'utf8');

describe('touch interaction policy', () => {
  it('prevents accidental selection while preserving editable controls', () => {
    expect(appCss).toMatch(/\.app\s*{[^}]*-webkit-user-select:\s*none;[^}]*user-select:\s*none;/s);
    expect(appCss).toMatch(/input:not\(\[type=['"]range['"]\]\)[^{]*{[^}]*user-select:\s*text;/s);
  });

  it('enables vertical touch panning on shared scroll regions', () => {
    for (const className of [
      'shot-list__rows',
      'debug-panel__tab-content',
      'trigger-history',
      'club-select__panel',
      'display-mode',
    ]) {
      expect(appCss).toContain(`.${className}`);
    }
    expect(appCss).toMatch(/touch-action:\s*pan-y;/);
    expect(appCss).toMatch(/overscroll-behavior-y:\s*contain;/);
  });
});
