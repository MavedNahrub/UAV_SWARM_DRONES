"""
test_path_loss.py — Unit tests for the RF path-loss model.

Tests the pure-math functions in path_loss.py without ROS dependencies.
"""

import math
import pytest
from comm_gateway.path_loss import (
    LinkParams,
    compute_rssi,
    is_connected,
    compute_packet_loss_prob,
    compute_latency,
    compute_link_quality,
)


@pytest.fixture
def default_params():
    """Default link parameters for testing."""
    return LinkParams()


class TestComputeRSSI:
    """Tests for RSSI computation."""

    def test_zero_distance_returns_tx_power(self, default_params):
        """Co-located nodes should get max signal."""
        rssi = compute_rssi(0.0, default_params, shadowing_sample=0.0)
        assert rssi == default_params.tx_power_dbm

    def test_rssi_decreases_with_distance(self, default_params):
        """RSSI must decrease as distance increases."""
        rssi_10m = compute_rssi(10.0, default_params, shadowing_sample=0.0)
        rssi_100m = compute_rssi(100.0, default_params, shadowing_sample=0.0)
        rssi_500m = compute_rssi(500.0, default_params, shadowing_sample=0.0)
        assert rssi_10m > rssi_100m > rssi_500m

    def test_known_value_at_reference_distance(self, default_params):
        """At d0, path loss should equal PL0."""
        rssi = compute_rssi(default_params.d0_m, default_params, shadowing_sample=0.0)
        expected = default_params.tx_power_dbm - default_params.pl0_dbm
        assert abs(rssi - expected) < 0.01

    def test_known_value_at_10m(self, default_params):
        """Hand-computed: RSSI at 10m with n=3, PL0=40, tx=20, no shadowing."""
        # path_loss = 40 + 10 * 3 * log10(10/1) = 40 + 30 = 70
        # rssi = 20 - 70 = -50
        rssi = compute_rssi(10.0, default_params, shadowing_sample=0.0)
        assert abs(rssi - (-50.0)) < 0.01

    def test_shadowing_shifts_rssi(self, default_params):
        """Positive shadowing should increase RSSI, negative should decrease it."""
        rssi_base = compute_rssi(10.0, default_params, shadowing_sample=0.0)
        rssi_plus = compute_rssi(10.0, default_params, shadowing_sample=5.0)
        rssi_minus = compute_rssi(10.0, default_params, shadowing_sample=-5.0)
        assert rssi_plus > rssi_base > rssi_minus


class TestIsConnected:
    """Tests for connection check."""

    def test_above_threshold_connected(self, default_params):
        assert is_connected(-80.0, default_params) is True

    def test_at_threshold_connected(self, default_params):
        assert is_connected(-85.0, default_params) is True

    def test_below_threshold_disconnected(self, default_params):
        assert is_connected(-90.0, default_params) is False


class TestPacketLoss:
    """Tests for packet loss probability."""

    def test_strong_signal_low_loss(self, default_params):
        """Well above threshold → near 0% loss."""
        prob = compute_packet_loss_prob(-50.0, default_params)
        assert prob < 0.01

    def test_at_threshold_50_percent(self, default_params):
        """At threshold → 50% loss."""
        prob = compute_packet_loss_prob(-85.0, default_params)
        assert abs(prob - 0.5) < 0.01

    def test_weak_signal_high_loss(self, default_params):
        """Well below threshold → near 100% loss."""
        prob = compute_packet_loss_prob(-120.0, default_params)
        assert prob > 0.99

    def test_probability_bounded(self, default_params):
        """Loss probability must be in [0, 1]."""
        for rssi in [-200.0, -85.0, 0.0, 50.0]:
            prob = compute_packet_loss_prob(rssi, default_params)
            assert 0.0 <= prob <= 1.0


class TestComputeLatency:
    """Tests for latency computation."""

    def test_latency_at_least_base(self, default_params):
        """Latency should always be >= base_latency_s."""
        latency = compute_latency(0.0, default_params)
        assert latency >= default_params.base_latency_s

    def test_latency_increases_with_distance(self, default_params):
        """Latency should increase with distance."""
        lat_10 = compute_latency(10.0, default_params)
        lat_1000 = compute_latency(1000.0, default_params)
        assert lat_1000 > lat_10


class TestComputeLinkQuality:
    """Tests for the combined link quality computation."""

    def test_close_range_connected(self, default_params):
        """At 10m, should be connected with low loss."""
        result = compute_link_quality(10.0, default_params, shadowing_sample=0.0)
        assert result['connected'] is True
        assert result['packet_loss_prob'] < 0.1

    def test_max_range_disconnected(self, default_params):
        """Beyond max_range_m, should be disconnected."""
        result = compute_link_quality(
            default_params.max_range_m + 100, default_params, shadowing_sample=0.0
        )
        assert result['connected'] is False
        assert result['packet_loss_prob'] == 1.0

    def test_result_keys(self, default_params):
        """Result dict should have all expected keys."""
        result = compute_link_quality(50.0, default_params, shadowing_sample=0.0)
        expected_keys = {'distance_m', 'rssi_dbm', 'connected', 'packet_loss_prob', 'latency_s'}
        assert set(result.keys()) == expected_keys
