import { useEffect, useState } from 'react';
import type { CameraStatus } from '../stores/useCameraStore';
import { getServerOrigin } from '../utils/serverOrigin';
import './CameraFeed.css';

interface CameraFeedProps {
  cameraStatus: CameraStatus;
  onToggleCamera: () => void;
  onToggleStream: () => void;
}

const STREAM_URL = `${getServerOrigin()}/camera/stream`;
const PREVIEW_URL = `${getServerOrigin()}/api/camera/preview.jpg`;
const PREVIEW_REFRESH_MS = 5000;

type PreviewState = 'checking' | 'available' | 'unavailable';

/**
 * Camera tab.
 *
 * When the high-speed capture runtime is active (--camera-capture), shows a
 * still refreshed every 5 s from the concurrent preview stream — the raw
 * rolling buffer keeps running, so shots are never missed while viewing.
 * Falls back to the legacy detection-camera UI when the capture runtime is
 * not present.
 */
export function CameraFeed({ cameraStatus, onToggleCamera, onToggleStream }: CameraFeedProps) {
  const [previewState, setPreviewState] = useState<PreviewState>('checking');
  const [previewSrc, setPreviewSrc] = useState<string | null>(null);
  const [lastUpdated, setLastUpdated] = useState<Date | null>(null);
  const [streamError, setStreamError] = useState(false);
  const [prevStreaming, setPrevStreaming] = useState(false);
  const { available, enabled, streaming, ball_detected, ball_confidence } = cameraStatus;

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
        if (!response.ok) return; // 503 while camera warms up: keep polling
        const blob = await response.blob();
        if (cancelled) return;
        const nextUrl = URL.createObjectURL(blob);
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        objectUrl = nextUrl;
        setPreviewSrc(nextUrl);
        setLastUpdated(new Date());
        setPreviewState('available');
      } catch {
        // network hiccup: keep the last frame and keep polling
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
      <div className="camera-feed">
        <div className="camera-feed__header">
          <h2 className="camera-feed__title">Camera</h2>
          {lastUpdated && (
            <span className="camera-feed__timestamp">
              updated {lastUpdated.toLocaleTimeString()}
            </span>
          )}
        </div>
        <div className="camera-feed__content">
          {previewSrc ? (
            <div className="camera-feed__stream">
              <img src={previewSrc} alt="Camera preview" className="camera-feed__video" />
              <svg
                className="camera-feed__crosshair"
                viewBox="0 0 100 100"
                preserveAspectRatio="none"
                aria-hidden="true"
              >
                {/* full-frame hairlines */}
                <line x1="50" y1="0" x2="50" y2="100" vectorEffect="non-scaling-stroke" className="camera-feed__hairline" />
                <line x1="0" y1="50" x2="100" y2="50" vectorEffect="non-scaling-stroke" className="camera-feed__hairline" />
              </svg>
              <div className="camera-feed__center-cross" aria-hidden="true" />
              <div className="camera-feed__overlay">
                <div className="camera-feed__status">Rolling buffer armed · still every 5s</div>
              </div>
            </div>
          ) : (
            <div className="camera-feed__message">
              <span className="camera-feed__icon">📷</span>
              <h3>Waiting for camera…</h3>
              <p>Fetching preview from the capture runtime</p>
            </div>
          )}
        </div>
      </div>
    );
  }

  // Legacy detection-camera UI (no capture runtime on this server)
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
          <span className="camera-feed__icon">📷</span>
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
            <span className="camera-feed__icon">📷</span>
            <h3>Camera Disabled</h3>
            <p>Click "Enable Camera" to start ball detection</p>
          </div>
        ) : !streaming ? (
          <div className="camera-feed__message">
            <span className="camera-feed__icon">🎥</span>
            <h3>Stream Paused</h3>
            <p>Ball detection is active. Click "Start Stream" to view live feed.</p>
            <div className={`camera-feed__detection ${ball_detected ? 'camera-feed__detection--detected' : ''}`}>
              {ball_detected ? `Ball Detected (${Math.round(ball_confidence * 100)}%)` : 'No Ball Detected'}
            </div>
          </div>
        ) : streamError ? (
          <div className="camera-feed__message camera-feed__message--error">
            <span className="camera-feed__icon">⚠️</span>
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
            <div className="camera-feed__overlay">
              <div className={`camera-feed__status ${ball_detected ? 'camera-feed__status--detected' : ''}`}>
                {ball_detected ? `Ball: ${Math.round(ball_confidence * 100)}%` : 'Searching...'}
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
