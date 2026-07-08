"""Unit tests for ball_capture.frame_index_row (hardware-free).

The serial layer is exercised on hardware; here we lock down the one piece of
logic that turns a parsed frame + wall-clock time into a CSV index row, since
that row is what aligns the TI stream to the OPS trigger timestamps later.
"""

import numpy as np

import ball_capture
import iwr6843_uart as uart


def _frame(tlvs, frame_number):
    return next(uart.iter_frames(uart.build_frame(tlvs, frame_number=frame_number)))


class TestFrameIndexRow:
    def test_picks_nearest_point_by_xy_range(self):
        # pt0 at hypot(0.1,3.0)=3.00, pt1 at hypot(0,1.5)=1.50 -> pt1 is nearest
        pts = np.array([[0.1, 3.0, 0.2, -5.0],
                        [0.0, 1.5, 0.1, 2.0]], dtype=np.float32)
        frame = _frame({uart.TLV_DETECTED_POINTS: pts.tobytes()}, 42)
        assert ball_capture.frame_index_row(frame, 1000.5) == \
            "42,1000.5000,2,1.500,+2.00"

    def test_no_detected_points_tlv_yields_empty_fields(self):
        frame = _frame({uart.TLV_STATS: b"\x00" * 24}, 7)
        assert ball_capture.frame_index_row(frame, 5.0) == "7,5.0000,0,,"

    def test_empty_point_cloud_yields_zero_and_empty(self):
        frame = _frame({uart.TLV_DETECTED_POINTS: b""}, 9)
        assert ball_capture.frame_index_row(frame, 12.25) == "9,12.2500,0,,"

    def test_timestamp_has_sub_ms_resolution(self):
        # 0.1 ms matters: cross-device alignment window is ~±60 ms
        frame = _frame({uart.TLV_STATS: b""}, 1)
        assert ball_capture.frame_index_row(frame, 1720000000.1234) == \
            "1,1720000000.1234,0,,"


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])
