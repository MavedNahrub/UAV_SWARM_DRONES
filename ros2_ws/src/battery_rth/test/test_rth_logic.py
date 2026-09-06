"""
test_rth_logic.py — Unit tests for battery reachability and RTH trigger logic.
"""

import pytest
from battery_rth.rth_logic import (
    compute_distance_3d,
    estimate_battery_required,
    is_reachable,
    should_trigger_rth,
)


def test_compute_distance():
    p1 = (0.0, 0.0, 0.0)
    p2 = (3.0, 4.0, 0.0)
    assert compute_distance_3d(p1, p2) == 5.0


def test_estimate_battery_required():
    current_pos = (0.0, 0.0, 0.0)
    target_pos = (100.0, 0.0, 0.0)
    gcs_pos = (0.0, 0.0, 0.0)
    # Total distance = 100 + 100 = 200m. At 10m/s -> 20s.
    # Survey duration = 20s -> total time = 40s.
    # Drain 0.001/s -> battery = 0.04 (4%)
    req = estimate_battery_required(
        current_pos, target_pos, gcs_pos,
        speed_mps=10.0, survey_duration_s=20.0, drain_rate_per_sec=0.001
    )
    assert abs(req - 0.04) < 1e-4


def test_is_reachable_true():
    current_pos = (0.0, 0.0, 0.0)
    target_pos = (100.0, 0.0, 0.0)
    gcs_pos = (0.0, 0.0, 0.0)
    # 80% battery available, critical is 20% -> 60% headroom, 4% needed
    assert is_reachable(
        current_pos, target_pos, gcs_pos,
        current_battery_frac=0.80, critical_frac=0.20,
        speed_mps=10.0, survey_duration_s=20.0, drain_rate_per_sec=0.001
    ) is True


def test_is_reachable_false_low_battery():
    current_pos = (0.0, 0.0, 0.0)
    target_pos = (100.0, 0.0, 0.0)
    gcs_pos = (0.0, 0.0, 0.0)
    # 22% battery available, critical 20% -> only 2% headroom, 4% needed
    assert is_reachable(
        current_pos, target_pos, gcs_pos,
        current_battery_frac=0.22, critical_frac=0.20,
        speed_mps=10.0, survey_duration_s=20.0, drain_rate_per_sec=0.001
    ) is False


def test_should_trigger_rth():
    current_pos = (200.0, 0.0, 0.0)
    gcs_pos = (0.0, 0.0, 0.0)
    # Distance to GCS = 200m. At 10m/s -> 20s. Drain 0.001/s -> 0.02 needed.
    # If critical = 0.20:
    # At 0.25 battery, 0.25 - 0.02 = 0.23 > 0.20 -> False
    assert should_trigger_rth(
        current_pos, gcs_pos, current_battery_frac=0.25,
        critical_frac=0.20, speed_mps=10.0, drain_rate_per_sec=0.001
    ) is False

    # At 0.21 battery, 0.21 - 0.02 = 0.19 <= 0.20 -> True
    assert should_trigger_rth(
        current_pos, gcs_pos, current_battery_frac=0.21,
        critical_frac=0.20, speed_mps=10.0, drain_rate_per_sec=0.001
    ) is True
