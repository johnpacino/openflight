"""Tests for the IWR6843 ground-multipath array model."""

import numpy as np
import pytest

from openflight.iwr6843.multipath import (
    ballistic_trajectory_from_range,
    collapse_reciprocal_cross_paths,
    ground_path_geometry,
    leave_one_channel_out_error,
    mimo_four_path_dictionary,
    receive_two_path_dictionary,
)
from openflight.iwr6843.music import steer


def test_ground_path_apparent_ranges_use_half_round_trip_distance():
    geometry = ground_path_geometry(2.0, 0.0, 0.5, 0.2)

    delta = geometry.image_distance_m - geometry.direct_distance_m
    assert geometry.cross_range_excess_m == pytest.approx(delta / 2.0)
    assert geometry.double_range_excess_m == pytest.approx(delta)
    assert geometry.reflection_point_m == pytest.approx(2.0 * 0.2 / 0.7)


def test_zero_height_target_collapses_direct_and_image_ranges():
    geometry = ground_path_geometry(2.0, 0.3, 0.0, 0.2)

    assert geometry.cross_range_excess_m == pytest.approx(0.0)
    assert geometry.double_range_excess_m == pytest.approx(0.0)


def test_ballistic_range_inversion_recovers_synthetic_trajectory():
    launch = np.radians(14.0)
    speed = 45.0
    tee_x = 1.57
    launch_height = 0.04
    radar_height = 0.1524
    lateral = 0.064
    expected_x = np.asarray([2.0, 2.8, 3.6, 4.4])
    dx = expected_x - tee_x
    time_s = dx / (speed * np.cos(launch))
    expected_height = launch_height + np.tan(launch) * dx - 0.5 * 9.81 * time_s**2
    ranges = np.sqrt(expected_x**2 + lateral**2 + (expected_height - radar_height) ** 2)

    x_m, height_m, direct_vr, image_vr = ballistic_trajectory_from_range(
        launch,
        ranges,
        speed_ms=speed,
        tee_x_m=tee_x,
        launch_height_m=launch_height,
        radar_height_m=radar_height,
        lateral_offset_m=lateral,
    )

    np.testing.assert_allclose(x_m, expected_x, atol=1e-9)
    np.testing.assert_allclose(height_m, expected_height, atol=1e-9)
    assert np.all(direct_vr > 0.0)
    assert np.all(image_vr > 0.0)
    assert np.all(image_vr > direct_vr)


def test_dd_and_gg_match_the_eight_element_virtual_ula():
    direct = np.radians(8.0)
    image = np.radians(-14.0)
    dictionary = mimo_four_path_dictionary(direct, image)

    np.testing.assert_allclose(dictionary[:, 0], steer(direct, 8))
    np.testing.assert_allclose(dictionary[:, 3], steer(image, 8))


def test_cross_paths_have_distinct_tx_and_rx_phase_progressions():
    direct = np.radians(9.0)
    image = np.radians(-15.0)
    dictionary = mimo_four_path_dictionary(direct, image)

    expected_rx_direct = receive_two_path_dictionary(direct, image)[:, 0]
    expected_rx_image = receive_two_path_dictionary(direct, image)[:, 1]
    np.testing.assert_allclose(dictionary[:4, 1], expected_rx_image)
    np.testing.assert_allclose(dictionary[:4, 2], expected_rx_direct)
    assert not np.allclose(dictionary[:, 1], steer(direct, 8))
    assert not np.allclose(dictionary[:, 2], steer(image, 8))


def test_tdm_residual_only_rotates_reflected_later_tx_components():
    dictionary = mimo_four_path_dictionary(
        np.radians(5.0),
        np.radians(-10.0),
        later_tx_index=0,
        cross_tdm_phase_rad=0.2,
        double_tdm_phase_rad=0.4,
    )
    baseline = mimo_four_path_dictionary(np.radians(5.0), np.radians(-10.0))

    np.testing.assert_allclose(dictionary[:4, 0], baseline[:4, 0])
    np.testing.assert_allclose(dictionary[:4, 1:3], baseline[:4, 1:3] * np.exp(0.2j))
    np.testing.assert_allclose(dictionary[:4, 3], baseline[:4, 3] * np.exp(0.4j))
    np.testing.assert_allclose(dictionary[4:], baseline[4:])


def test_tdm_residual_can_rotate_second_physical_tx_block():
    dictionary = mimo_four_path_dictionary(
        np.radians(5.0),
        np.radians(-10.0),
        later_tx_index=1,
        cross_tdm_phase_rad=0.2,
        double_tdm_phase_rad=0.4,
    )
    baseline = mimo_four_path_dictionary(np.radians(5.0), np.radians(-10.0))

    np.testing.assert_allclose(dictionary[:4], baseline[:4])
    np.testing.assert_allclose(dictionary[4:, 0], baseline[4:, 0])
    np.testing.assert_allclose(dictionary[4:, 1:3], baseline[4:, 1:3] * np.exp(0.2j))
    np.testing.assert_allclose(dictionary[4:, 3], baseline[4:, 3] * np.exp(0.4j))


def test_reciprocal_collapse_combines_only_cross_columns():
    dictionary = mimo_four_path_dictionary(np.radians(4.0), np.radians(-8.0))
    collapsed = collapse_reciprocal_cross_paths(dictionary)

    assert collapsed.shape == (8, 3)
    np.testing.assert_allclose(collapsed[:, 0], dictionary[:, 0])
    np.testing.assert_allclose(collapsed[:, 1], dictionary[:, 1] + dictionary[:, 2])
    np.testing.assert_allclose(collapsed[:, 2], dictionary[:, 3])


def test_reciprocal_collapse_rejects_non_four_path_input():
    with pytest.raises(ValueError, match="four columns"):
        collapse_reciprocal_cross_paths(np.ones((8, 3), dtype=complex))


def test_leave_one_channel_out_error_rewards_predictive_model():
    direct = np.radians(7.0)
    image = np.radians(-12.0)
    dictionary = receive_two_path_dictionary(direct, image)
    snapshot = dictionary @ np.array([1.0 + 0.2j, 0.35 - 0.1j])
    wrong = receive_two_path_dictionary(np.radians(18.0), np.radians(-25.0))

    assert leave_one_channel_out_error(snapshot, dictionary) < 1e-12
    assert leave_one_channel_out_error(snapshot, wrong) > 1e-3


def test_leave_one_channel_out_error_supports_batched_dictionaries():
    dictionary = receive_two_path_dictionary(np.radians(5.0), np.radians(-9.0))
    dictionaries = np.stack([dictionary, dictionary])
    snapshots = np.stack(
        [
            dictionary @ np.array([1.0, 0.2j]),
            dictionary @ np.array([0.7j, -0.3]),
        ]
    )

    errors = leave_one_channel_out_error(snapshots, dictionaries)
    np.testing.assert_allclose(errors, 0.0, atol=1e-12)


def test_leave_one_channel_out_error_rejects_dimension_mismatch():
    with pytest.raises(ValueError, match="channel dimension"):
        leave_one_channel_out_error(np.ones(4), np.ones((8, 2)))
