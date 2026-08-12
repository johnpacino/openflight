import { renderToString } from 'react-dom/server';
import { describe, expect, it, vi } from 'vitest';
import type { CameraCaptureSettings, CameraStatus } from '../stores/useCameraStore';
import { verticalViewTargets } from '../utils/cameraView';
import { CameraFeed } from './CameraFeed';

const cameraStatus: CameraStatus = {
  available: true,
  enabled: true,
  streaming: false,
  ball_detected: false,
  ball_confidence: 0,
};

const captureSettings: CameraCaptureSettings = {
  available: true,
  enabled: true,
  running: true,
  armed: true,
  width: 320,
  height: 200,
  fps: 600,
  pre_ms: 150,
  post_ms: 50,
  pre_frames: 90,
  post_frames: 30,
  exposure_us: 500,
  max_exposure_us: 1666,
  gain: 2,
  rotate_180: true,
  alignment_x_pct: 48,
  alignment_y_pct: 55,
  raw_crop_adjustable: true,
  vertical_offset_px: -20,
  vertical_offset_min_px: -70,
  vertical_offset_max_px: 70,
  vertical_offset_step_px: 10,
};

describe('CameraFeed', () => {
  it('maps view direction through the 180-degree mount rotation', () => {
    expect(verticalViewTargets(0, 10, true)).toEqual({ up: 10, down: -10 });
    expect(verticalViewTargets(0, 10, false)).toEqual({ up: -10, down: 10 });
  });

  it('renders the dominant preview workspace and operator settings', () => {
    const html = renderToString(
      <CameraFeed
        cameraStatus={cameraStatus}
        captureSettings={captureSettings}
        captureSettingsError={null}
        onToggleCamera={vi.fn()}
        onToggleStream={vi.fn()}
        onUpdateCaptureSettings={vi.fn()}
      />
    );

    expect(html).toContain('camera-feed__workspace');
    expect(html).toContain('Camera setup');
    expect(html).toContain('Environment profile');
    expect(html).toContain('Exposure check');
    expect(html).toContain('camera-feed__exposure-quality');
    expect(html).toContain('Darker');
    expect(html).toContain('Brighter');
    expect(html).toContain('Outdoor sun');
    expect(html).toContain('250<!-- --> µs · <!-- -->4<!-- -->×');
    expect(html).toContain('Outdoor shade');
    expect(html).toContain('Evening');
    expect(html).toContain('Indoor bright');
    expect(html).toContain('Indoor dark');
    expect(html).toContain('Night');
    expect(html).toContain('1000<!-- --> µs · <!-- -->20<!-- -->×');
    expect(html).toContain('Ball placement guide');
    expect(html).toContain('50% across · 78% down');
    expect(html).not.toContain('type="range"');
    expect(html).not.toContain('Apply alignment guide');
    expect(html).toContain('Sensor view');
    expect(html).toContain('View up');
    expect(html).toContain('View down');
    expect(html).toContain('-20 px');
    expect(html).toContain('320 × 200');
    expect(html).toContain('600 fps');
    expect(html).toContain('Armed');
  });
});
