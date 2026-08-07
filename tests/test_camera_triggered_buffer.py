"""Tests for the high-speed camera trigger ring."""

import threading

import numpy as np
import pytest

from openflight.camera.capture_runtime import parse_scaler_crop
from openflight.camera.triggered_buffer import (
    CameraFrame,
    TriggeredFrameBuffer,
    timing_summary,
    unpack_r8_frame,
    unpack_yuv420_y_plane,
)


def make_frame(index: int, interval_ns: int = 4_000_000) -> CameraFrame:
    """Create a tiny deterministic camera frame."""
    return CameraFrame(
        image=np.full((2, 3), index, dtype=np.uint8),
        sensor_timestamp_ns=index * interval_ns,
        host_timestamp_ns=index * interval_ns + 100,
        exposure_us=1000,
        analogue_gain=4.0,
    )


def test_ring_freezes_latest_pre_frames_and_post_tail():
    ring = TriggeredFrameBuffer(pre_trigger_frames=3, post_trigger_frames=2)
    for index in range(5):
        ring.add_frame(make_frame(index))

    assert ring.trigger(host_timestamp_ns=19_000_000)
    ring.add_frame(make_frame(5))
    ring.add_frame(make_frame(6))

    capture = ring.wait_for_capture(timeout_s=0.01)
    assert capture is not None
    assert [int(frame.image[0, 0]) for frame in capture.frames] == [2, 3, 4, 5, 6]
    assert capture.pre_trigger_count == 3
    assert capture.post_trigger_count == 2
    assert capture.trigger_host_timestamp_ns == 19_000_000


def test_ring_rejects_overlapping_trigger():
    ring = TriggeredFrameBuffer(pre_trigger_frames=2, post_trigger_frames=2)
    ring.add_frame(make_frame(0))
    ring.add_frame(make_frame(1))
    assert ring.trigger()
    assert not ring.trigger()


def test_ring_rejects_trigger_until_pre_buffer_is_full():
    ring = TriggeredFrameBuffer(pre_trigger_frames=3, post_trigger_frames=1)
    ring.add_frame(make_frame(0))
    ring.add_frame(make_frame(1))
    assert not ring.trigger()

    ring.add_frame(make_frame(2))
    assert ring.trigger()


def test_wait_for_capture_wakes_on_completed_tail():
    ring = TriggeredFrameBuffer(pre_trigger_frames=1, post_trigger_frames=1)
    ring.add_frame(make_frame(0))
    ring.trigger()
    thread = threading.Thread(target=lambda: ring.add_frame(make_frame(1)))
    thread.start()
    capture = ring.wait_for_capture(timeout_s=0.1)
    thread.join()
    assert capture is not None


def test_pop_capture_consumes_ready_capture_without_blocking():
    ring = TriggeredFrameBuffer(pre_trigger_frames=1, post_trigger_frames=1)
    ring.add_frame(make_frame(0))
    assert ring.pop_capture() is None

    ring.trigger()
    ring.add_frame(make_frame(1))

    capture = ring.pop_capture()
    assert capture is not None
    assert ring.pop_capture() is None


def test_unpack_r8_from_pisp_high_bytes():
    pixels = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    raw = np.zeros((2, 6), dtype=np.uint8)
    raw[:, 1::2] = pixels
    assert np.array_equal(unpack_r8_frame(raw, 3, 2, False), pixels)
    assert np.array_equal(unpack_r8_frame(raw, 3, 2, True), pixels[::-1, ::-1])


def test_unpack_r8_rejects_short_frame():
    with pytest.raises(ValueError, match="unexpected raw frame"):
        unpack_r8_frame(np.zeros((1, 2), dtype=np.uint8), 3, 2, False)


def test_unpack_yuv420_y_plane_with_stride_and_chroma_rows():
    main = np.zeros((4, 5), dtype=np.uint8)
    main[:2, :3] = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)

    assert np.array_equal(
        unpack_yuv420_y_plane(main, 3, 2, False),
        np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8),
    )
    assert np.array_equal(
        unpack_yuv420_y_plane(main, 3, 2, True),
        np.array([[6, 5, 4], [3, 2, 1]], dtype=np.uint8),
    )


def test_unpack_yuv420_y_plane_rejects_short_frame():
    with pytest.raises(ValueError, match="unexpected YUV420 frame"):
        unpack_yuv420_y_plane(np.zeros((1, 2), dtype=np.uint8), 3, 2, False)


def test_parse_scaler_crop():
    assert parse_scaler_crop("256,160,768,480") == (256, 160, 768, 480)
    assert parse_scaler_crop(None) is None
    with pytest.raises(ValueError, match="X,Y,W,H"):
        parse_scaler_crop("1,2,3")
    with pytest.raises(ValueError, match="positive"):
        parse_scaler_crop("1,2,0,4")


def test_timing_summary_reports_fps_and_gap():
    frames = [make_frame(index) for index in range(5)]
    summary = timing_summary(frames)
    assert summary["delivered_fps"] == pytest.approx(250.0)
    assert summary["median_interval_ms"] == pytest.approx(4.0)
    assert summary["gap_count"] == 0

    frames[-1] = make_frame(6)
    summary = timing_summary(frames)
    assert summary["gap_count"] == 1
