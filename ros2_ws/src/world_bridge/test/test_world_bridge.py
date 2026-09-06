"""
test_world_bridge.py — Unit tests for virtual battery and geometry utilities.
"""

import pytest
from world_bridge.battery_model import VirtualBatteryModel
from world_bridge.geometry_utils import (
    point_in_polygon_2d,
    check_obstacle_collision,
    compute_pairwise_separation,
)


def test_virtual_battery_drain():
    model = VirtualBatteryModel(initial_frac=1.0, base_drain_per_sec=0.01)

    # 10s as SURVEYOR (multiplier 1.2) -> 10 * 0.01 * 1.2 = 0.12 drain -> 0.88
    b = model.step(10.0, role=0)
    assert abs(b - 0.88) < 1e-4

    # 10s as RESERVE (multiplier 0.0) -> 0 drain -> 0.88
    b = model.step(10.0, role=3)
    assert abs(b - 0.88) < 1e-4


def test_point_in_polygon():
    # Square [-10, 10] x [-10, 10]
    poly = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]

    assert point_in_polygon_2d((0.0, 0.0), poly) is True
    assert point_in_polygon_2d((5.0, 5.0), poly) is True
    assert point_in_polygon_2d((15.0, 0.0), poly) is False
    assert point_in_polygon_2d((-12.0, -12.0), poly) is False


def test_check_obstacle_collision():
    center = (50.0, 50.0, 20.0)
    radius = 10.0

    # Inside obstacle
    assert check_obstacle_collision((52.0, 50.0, 20.0), center, radius) is True

    # Outside obstacle
    assert check_obstacle_collision((70.0, 50.0, 20.0), center, radius) is False


def test_compute_pairwise_separation():
    positions = {
        'uav_00': (0.0, 0.0, 0.0),
        'uav_01': (3.0, 4.0, 0.0),
        'uav_02': (10.0, 0.0, 0.0),
    }
    pairs = compute_pairwise_separation(positions)
    assert len(pairs) == 3

    # (uav_00, uav_01) -> dist 5.0
    pair_01 = [p for p in pairs if p[0] == 'uav_00' and p[1] == 'uav_01'][0]
    assert abs(pair_01[2] - 5.0) < 1e-4
