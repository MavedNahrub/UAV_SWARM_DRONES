"""
test_apf_safety.py — Unit tests for Artificial Potential Field safety filtering.
"""

import pytest
from safety_supervisor.apf_safety import (
    compute_apf_safe_velocity,
    norm_3d,
    clamp_magnitude,
)


def test_clamp_magnitude():
    v = (10.0, 0.0, 0.0)
    clamped = clamp_magnitude(v, 5.0)
    assert abs(norm_3d(clamped) - 5.0) < 1e-4

    v_small = (2.0, 0.0, 0.0)
    assert clamp_magnitude(v_small, 5.0) == v_small


def test_apf_nominal_no_hazards():
    proposed_vel = (5.0, 0.0, 0.0)
    current_pos = (0.0, 0.0, 20.0)
    peers = {'uav_01': (100.0, 100.0, 20.0)}  # far away
    obstacles = [((200.0, 200.0, 20.0), 10.0)]
    geofence = [(-500.0, -500.0), (500.0, -500.0), (500.0, 500.0), (-500.0, 500.0)]

    safe_vel, violations = compute_apf_safe_velocity(
        uav_id='uav_00',
        proposed_vel=proposed_vel,
        current_pos=current_pos,
        peer_positions=peers,
        obstacles=obstacles,
        geofence_polygon=geofence,
        min_separation_m=8.0,
    )

    # Velocity should be essentially unperturbed
    assert abs(safe_vel[0] - 5.0) < 0.1
    assert abs(safe_vel[1]) < 0.1
    assert len(violations) == 0


def test_apf_peer_repulsion():
    # UAV 00 moving towards UAV 01 (distance = 5m < min_separation 8m)
    proposed_vel = (5.0, 0.0, 0.0)
    current_pos = (0.0, 0.0, 20.0)
    peers = {'uav_01': (5.0, 0.0, 20.0)}
    obstacles = []
    geofence = []

    safe_vel, violations = compute_apf_safe_velocity(
        uav_id='uav_00',
        proposed_vel=proposed_vel,
        current_pos=current_pos,
        peer_positions=peers,
        obstacles=obstacles,
        geofence_polygon=geofence,
        min_separation_m=8.0,
    )

    # Should detect proximity violation
    assert any("PEER_PROXIMITY" in v for v in violations)
    # Forward velocity should be reduced or repelled backwards (negative x)
    assert safe_vel[0] < proposed_vel[0]


def test_apf_obstacle_repulsion():
    # Obstacle directly ahead at x=12m (radius 10m -> boundary at 2m < safety_margin 5m)
    proposed_vel = (5.0, 0.0, 0.0)
    current_pos = (0.0, 0.0, 20.0)
    peers = {}
    obstacles = [((12.0, 0.0, 20.0), 10.0)]
    geofence = []

    safe_vel, violations = compute_apf_safe_velocity(
        uav_id='uav_00',
        proposed_vel=proposed_vel,
        current_pos=current_pos,
        peer_positions=peers,
        obstacles=obstacles,
        geofence_polygon=geofence,
        obstacle_safety_margin_m=5.0,
    )

    assert any("OBSTACLE_PROXIMITY" in v for v in violations)
    assert safe_vel[0] < proposed_vel[0]
