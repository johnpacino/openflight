import { create } from 'zustand';

export interface CameraStatus {
  available: boolean;
  enabled: boolean;
  streaming: boolean;
  ball_detected: boolean;
  ball_confidence: number;
}

export interface CameraCaptureSettings {
  available: boolean;
  enabled?: boolean;
  running?: boolean;
  armed?: boolean;
  width?: number;
  height?: number;
  fps?: number;
  pre_ms?: number;
  post_ms?: number;
  pre_frames?: number;
  post_frames?: number;
  buffered_frames?: number;
  required_pre_frames?: number;
  exposure_us?: number;
  max_exposure_us?: number;
  gain?: number;
  stream?: string;
  rotate_180?: boolean;
  mirror_horizontal?: boolean;
  roll_correction_deg?: number;
  alignment_x_pct?: number;
  alignment_y_pct?: number;
  raw_crop_adjustable?: boolean;
  vertical_offset_px?: number;
  vertical_offset_min_px?: number;
  vertical_offset_max_px?: number;
  vertical_offset_step_px?: number;
}

interface CameraState {
  cameraStatus: CameraStatus;
  captureSettings: CameraCaptureSettings;
  captureSettingsError: string | null;
  setCameraStatus: (status: Partial<CameraStatus>) => void;
  setCaptureSettings: (settings: CameraCaptureSettings) => void;
  setCaptureSettingsError: (error: string | null) => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  cameraStatus: {
    available: false,
    enabled: false,
    streaming: false,
    ball_detected: false,
    ball_confidence: 0,
  },
  captureSettings: { available: false },
  captureSettingsError: null,
  setCameraStatus: (status) =>
    set((state) => ({
      cameraStatus: { ...state.cameraStatus, ...status },
    })),
  setCaptureSettings: (settings) => set({ captureSettings: settings, captureSettingsError: null }),
  setCaptureSettingsError: (error) => set({ captureSettingsError: error }),
}));
