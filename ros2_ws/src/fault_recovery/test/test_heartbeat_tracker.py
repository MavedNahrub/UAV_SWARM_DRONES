"""
test_heartbeat_tracker.py — Unit tests for fault recovery heartbeat tracking.
"""

import pytest
from fault_recovery.heartbeat_tracker import HeartbeatTracker


def test_heartbeat_registration_and_normal():
    tracker = HeartbeatTracker(my_id='uav_00', timeout_s=5.0)
    tracker.register_peer('uav_01', current_time=0.0)

    # Within timeout
    faults = tracker.check_timeouts(current_time=3.0)
    assert len(faults) == 0
    assert not tracker.is_faulted('uav_01')

    # Heartbeat arrives
    recovery = tracker.record_heartbeat('uav_01', current_time=4.0)
    assert recovery is None  # not recovering an active fault


def test_timeout_triggers_fault():
    tracker = HeartbeatTracker(my_id='uav_00', timeout_s=5.0)
    tracker.register_peer('uav_01', current_time=0.0)

    # Exceed timeout
    faults = tracker.check_timeouts(current_time=5.5)
    assert len(faults) == 1
    assert faults[0] == ('uav_01', 5.5)
    assert tracker.is_faulted('uav_01')

    # Subsequent check doesn't re-trigger same fault
    faults_again = tracker.check_timeouts(current_time=6.0)
    assert len(faults_again) == 0


def test_heartbeat_recovery():
    tracker = HeartbeatTracker(my_id='uav_00', timeout_s=5.0)
    tracker.register_peer('uav_01', current_time=0.0)
    tracker.check_timeouts(current_time=6.0)
    assert tracker.is_faulted('uav_01')

    # Peer comes back
    recovery = tracker.record_heartbeat('uav_01', current_time=8.0)
    assert recovery is not None
    peer_id, detected_time, recovered_time = recovery
    assert peer_id == 'uav_01'
    assert detected_time == 6.0
    assert recovered_time == 8.0
    assert not tracker.is_faulted('uav_01')


def test_multiple_peers():
    tracker = HeartbeatTracker(my_id='uav_00', timeout_s=5.0)
    tracker.register_peer('uav_01', current_time=0.0)
    tracker.register_peer('uav_02', current_time=0.0)

    # uav_01 updates at 4.0, uav_02 does not
    tracker.record_heartbeat('uav_01', current_time=4.0)

    # At 5.5, only uav_02 times out
    faults = tracker.check_timeouts(current_time=5.5)
    assert len(faults) == 1
    assert faults[0][0] == 'uav_02'
    assert tracker.is_faulted('uav_02')
    assert not tracker.is_faulted('uav_01')
