import { renderToString } from 'react-dom/server';
import { describe, expect, it, vi } from 'vitest';
import type { CameraCaptureSettings, CameraStatus } from '../stores/useCameraStore';
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
  alignment_x_pct: 48,
  alignment_y_pct: 55,
};

describe('CameraFeed', () => {
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
    expect(html).toContain('Exposure');
    expect(html).toContain('Analog gain');
    expect(html).toContain('Alignment guide');
    expect(html).toContain('320 × 200');
    expect(html).toContain('600 fps');
    expect(html).toContain('Armed');
  });
});
