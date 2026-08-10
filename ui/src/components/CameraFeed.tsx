import { useEffect, useState, type CSSProperties, type FormEvent } from 'react';
import type { CameraCaptureSettings, CameraStatus } from '../stores/useCameraStore';
import { getServerOrigin } from '../utils/serverOrigin';
import './CameraFeed.css';

interface CameraFeedProps {
  cameraStatus: CameraStatus;
  captureSettings: CameraCaptureSettings;
  captureSettingsError: string | null;
  onToggleCamera: () => void;
  onToggleStream: () => void;
  onUpdateCaptureSettings: (settings: Partial<CameraCaptureSettings>) => void;
}

interface CaptureSettingsPanelProps {
  settings: CameraCaptureSettings;
  error: string | null;
  onUpdate: (settings: Partial<CameraCaptureSettings>) => void;
}

const STREAM_URL = `${getServerOrigin()}/camera/stream`;
const PREVIEW_URL = `${getServerOrigin()}/api/camera/preview.jpg`;
const PREVIEW_REFRESH_MS = 5000;

type PreviewState = 'checking' | 'available' | 'unavailable';

function CaptureSettingsPanel({ settings, error, onUpdate }: CaptureSettingsPanelProps) {
  const [exposureUs, setExposureUs] = useState(settings.exposure_us ?? 500);
  const [gain, setGain] = useState(settings.gain ?? 2);
  const [alignmentX, setAlignmentX] = useState(settings.alignment_x_pct ?? 50);
  const [alignmentY, setAlignmentY] = useState(settings.alignment_y_pct ?? 50);

  const maxExposureUs = Math.max(25, settings.max_exposure_us ?? 3000);
  const isDirty =
    exposureUs !== (settings.exposure_us ?? 500) ||
    gain !== (settings.gain ?? 2) ||
    alignmentX !== (settings.alignment_x_pct ?? 50) ||
    alignmentY !== (settings.alignment_y_pct ?? 50);

  const apply = (event: FormEvent) => {
    event.preventDefault();
    onUpdate({
      exposure_us: exposureUs,
      gain,
      alignment_x_pct: alignmentX,
      alignment_y_pct: alignmentY,
    });
  };

  return (
    <aside className="camera-settings">
      <div className="camera-settings__heading">
        <div>
          <span className="camera-settings__eyebrow">Capture controls</span>
          <h3>Camera setup</h3>
        </div>
        <span className={`camera-settings__armed ${settings.armed ? 'camera-settings__armed--ready' : ''}`}>
          {settings.armed ? 'Armed' : settings.running ? 'Filling' : 'Offline'}
        </span>
      </div>

      <form onSubmit={apply}>
        <section className="camera-settings__section">
          <div className="camera-settings__section-title">
            <span>Image</span>
            <small>applies live</small>
          </div>
          <label className="camera-settings__field">
            <span>Exposure</span>
            <div className="camera-settings__value">
              <input
                type="number"
                min={25}
                max={maxExposureUs}
                step={25}
                value={exposureUs}
                onChange={(event) => setExposureUs(Number(event.target.value))}
              />
              <em>µs</em>
            </div>
            <input
              type="range"
              min={25}
              max={maxExposureUs}
              step={25}
              value={Math.min(exposureUs, maxExposureUs)}
              onChange={(event) => setExposureUs(Number(event.target.value))}
            />
          </label>
          <label className="camera-settings__field">
            <span>Analog gain</span>
            <div className="camera-settings__value">
              <input
                type="number"
                min={1}
                max={16}
                step={0.5}
                value={gain}
                onChange={(event) => setGain(Number(event.target.value))}
              />
              <em>×</em>
            </div>
            <input
              type="range"
              min={1}
              max={16}
              step={0.5}
              value={gain}
              onChange={(event) => setGain(Number(event.target.value))}
            />
          </label>
        </section>

        <section className="camera-settings__section">
          <div className="camera-settings__section-title">
            <span>Alignment guide</span>
            <small>preview overlay</small>
          </div>
          <label className="camera-settings__compact-field">
            <span>Horizontal</span>
            <input
              type="range"
              min={0}
              max={100}
              step={1}
              value={alignmentX}
              onChange={(event) => setAlignmentX(Number(event.target.value))}
            />
            <output>{alignmentX.toFixed(0)}%</output>
          </label>
          <label className="camera-settings__compact-field">
            <span>Vertical</span>
            <input
              type="range"
              min={0}
              max={100}
              step={1}
              value={alignmentY}
              onChange={(event) => setAlignmentY(Number(event.target.value))}
            />
            <output>{alignmentY.toFixed(0)}%</output>
          </label>
          <p className="camera-settings__note">
            This moves the alignment guide. The raw high-speed crop remains fixed by the camera mode.
          </p>
        </section>

        <section className="camera-settings__section camera-settings__section--summary">
          <div className="camera-settings__section-title">
            <span>Capture</span>
            <small>restart required to change</small>
          </div>
          <dl className="camera-settings__summary">
            <div>
              <dt>Mode</dt>
              <dd>{settings.width && settings.height ? `${settings.width} × ${settings.height}` : '—'}</dd>
            </div>
            <div>
              <dt>Requested</dt>
              <dd>{settings.fps ? `${settings.fps.toFixed(0)} fps` : '—'}</dd>
            </div>
            <div>
              <dt>Buffer</dt>
              <dd>
                {settings.pre_ms !== undefined && settings.post_ms !== undefined
                  ? `${settings.pre_ms.toFixed(0)} / ${settings.post_ms.toFixed(0)} ms`
                  : '—'}
              </dd>
            </div>
            <div>
              <dt>Frames</dt>
              <dd>
                {settings.pre_frames !== undefined && settings.post_frames !== undefined
                  ? `${settings.pre_frames} + ${settings.post_frames}`
                  : '—'}
              </dd>
            </div>
          </dl>
        </section>

        {error && <p className="camera-settings__error">{error}</p>}
        <button type="submit" className="camera-settings__apply" disabled={!settings.available || !isDirty}>
          {isDirty ? 'Apply camera settings' : 'Settings applied'}
        </button>
      </form>
    </aside>
  );
}

/**
 * Camera tab.
 *
 * When the high-speed capture runtime is active (--camera-capture), shows a
 * still refreshed every 5 s from the concurrent preview stream. The raw
 * rolling buffer keeps running, so shots are never missed while viewing.
 */
export function CameraFeed({
  cameraStatus,
  captureSettings,
  captureSettingsError,
  onToggleCamera,
  onToggleStream,
  onUpdateCaptureSettings,
}: CameraFeedProps) {
  const [previewState, setPreviewState] = useState<PreviewState>('checking');
  const [previewSrc, setPreviewSrc] = useState<string | null>(null);
  const [lastUpdated, setLastUpdated] = useState<Date | null>(null);
  const [streamError, setStreamError] = useState(false);
  const [prevStreaming, setPrevStreaming] = useState(false);
  const { available, enabled, streaming, ball_detected, ball_confidence } = cameraStatus;
  const alignmentX = captureSettings.alignment_x_pct ?? 50;
  const alignmentY = captureSettings.alignment_y_pct ?? 50;
  const crosshairStyle = {
    '--camera-crosshair-x': `${alignmentX}%`,
    '--camera-crosshair-y': `${alignmentY}%`,
  } as CSSProperties;

  useEffect(() => {
    let cancelled = false;
    let objectUrl: string | null = null;
    let timer: ReturnType<typeof setInterval> | null = null;

    const refresh = async () => {
      try {
        const response = await fetch(`${PREVIEW_URL}?t=${Date.now()}`, { cache: 'no-store' });
        if (cancelled) return;
        if (response.status === 404) {
          setPreviewState('unavailable');
          if (timer) clearInterval(timer);
          return;
        }
        if (!response.ok) return;
        const blob = await response.blob();
        if (cancelled) return;
        const nextUrl = URL.createObjectURL(blob);
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        objectUrl = nextUrl;
        setPreviewSrc(nextUrl);
        setLastUpdated(new Date());
        setPreviewState('available');
      } catch {
        // Keep the last frame and retry after a transient network failure.
      }
    };

    refresh();
    timer = setInterval(refresh, PREVIEW_REFRESH_MS);
    return () => {
      cancelled = true;
      if (timer) clearInterval(timer);
      if (objectUrl) URL.revokeObjectURL(objectUrl);
    };
  }, []);

  if (previewState === 'available' || previewState === 'checking') {
    return (
      <div className="camera-feed camera-feed--capture">
        <div className="camera-feed__header">
          <div>
            <span className="camera-feed__eyebrow">High-speed capture</span>
            <h2 className="camera-feed__title">Camera alignment</h2>
          </div>
          {lastUpdated && <span className="camera-feed__timestamp">preview {lastUpdated.toLocaleTimeString()}</span>}
        </div>
        <div className="camera-feed__workspace">
          <div className="camera-feed__preview-column">
            {previewSrc ? (
              <div className="camera-feed__stream" style={crosshairStyle}>
                <img src={previewSrc} alt="Camera preview" className="camera-feed__video" />
                <svg
                  className="camera-feed__crosshair"
                  viewBox="0 0 100 100"
                  preserveAspectRatio="none"
                  aria-hidden="true"
                >
                  <line
                    x1={alignmentX}
                    y1="0"
                    x2={alignmentX}
                    y2="100"
                    vectorEffect="non-scaling-stroke"
                    className="camera-feed__hairline"
                  />
                  <line
                    x1="0"
                    y1={alignmentY}
                    x2="100"
                    y2={alignmentY}
                    vectorEffect="non-scaling-stroke"
                    className="camera-feed__hairline"
                  />
                </svg>
                <div className="camera-feed__center-cross" aria-hidden="true" />
                <div className="camera-feed__overlay">
                  <div className="camera-feed__status">Rolling buffer remains armed</div>
                </div>
              </div>
            ) : (
              <div className="camera-feed__message camera-feed__message--preview">
                <h3>Waiting for camera</h3>
                <p>Fetching the first preview from the capture runtime</p>
              </div>
            )}
          </div>
          <CaptureSettingsPanel
            key={`${captureSettings.exposure_us}-${captureSettings.gain}-${captureSettings.alignment_x_pct}-${captureSettings.alignment_y_pct}`}
            settings={captureSettings}
            error={captureSettingsError}
            onUpdate={onUpdateCaptureSettings}
          />
        </div>
      </div>
    );
  }

  // Legacy detection-camera UI (no high-speed capture runtime on this server).
  if (streaming && !prevStreaming) {
    setStreamError(false);
  }
  if (streaming !== prevStreaming) {
    setPrevStreaming(streaming);
  }

  if (!available) {
    return (
      <div className="camera-feed camera-feed--unavailable">
        <div className="camera-feed__message">
          <h3>Camera Not Available</h3>
          <p>Start the server with --camera-capture (preview) or --camera (detection)</p>
        </div>
      </div>
    );
  }

  return (
    <div className="camera-feed">
      <div className="camera-feed__header">
        <h2 className="camera-feed__title">Camera Feed</h2>
        <div className="camera-feed__controls">
          <button
            className={`camera-feed__button ${enabled ? 'camera-feed__button--active' : ''}`}
            onClick={onToggleCamera}
          >
            {enabled ? 'Disable Camera' : 'Enable Camera'}
          </button>
          {enabled && (
            <button
              className={`camera-feed__button ${streaming ? 'camera-feed__button--streaming' : ''}`}
              onClick={onToggleStream}
            >
              {streaming ? 'Stop Stream' : 'Start Stream'}
            </button>
          )}
        </div>
      </div>

      <div className="camera-feed__content">
        {!enabled ? (
          <div className="camera-feed__message">
            <h3>Camera Disabled</h3>
            <p>Enable the camera to start ball detection</p>
          </div>
        ) : !streaming ? (
          <div className="camera-feed__message">
            <h3>Stream Paused</h3>
            <p>Ball detection remains active while the stream is paused.</p>
            <div className={`camera-feed__detection ${ball_detected ? 'camera-feed__detection--detected' : ''}`}>
              {ball_detected ? `Ball Detected (${Math.round(ball_confidence * 100)}%)` : 'No Ball Detected'}
            </div>
          </div>
        ) : streamError ? (
          <div className="camera-feed__message camera-feed__message--error">
            <h3>Stream Error</h3>
            <p>Could not load camera stream</p>
            <button className="camera-feed__button" onClick={() => setStreamError(false)}>
              Retry
            </button>
          </div>
        ) : (
          <div className="camera-feed__stream">
            <img
              src={STREAM_URL}
              alt="Camera Feed"
              className="camera-feed__video"
              onError={() => setStreamError(true)}
            />
          </div>
        )}
      </div>
    </div>
  );
}
