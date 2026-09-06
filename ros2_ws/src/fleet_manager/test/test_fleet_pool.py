"""
test_fleet_pool.py — Unit tests for FleetPool.
"""

import pytest
from fleet_manager.fleet_pool import FleetPool


def test_initial_state():
    pool = FleetPool(active_uav_ids=['uav_00', 'uav_01'], reserve_uav_ids=['uav_02', 'uav_03'])
    assert pool.get_active() == ['uav_00', 'uav_01']
    assert pool.get_reserves() == ['uav_02', 'uav_03']
    assert pool.get_failed() == []


def test_swap_in_success():
    pool = FleetPool(active_uav_ids=['uav_00', 'uav_01'], reserve_uav_ids=['uav_02'])
    accepted, activated_id = pool.request_swap_in('uav_00', reason='RTH')

    assert accepted is True
    assert activated_id == 'uav_02'
    assert 'uav_00' not in pool.get_active()
    assert 'uav_02' in pool.get_active()
    assert pool.get_reserves() == []


def test_swap_in_failure():
    pool = FleetPool(active_uav_ids=['uav_00'], reserve_uav_ids=['uav_01'])
    # First swap succeeds
    accepted, activated = pool.request_swap_in('uav_00', reason='FAILURE')
    assert accepted is True
    assert activated == 'uav_01'
    assert 'uav_00' in pool.get_failed()

    # Second swap has no reserves left
    accepted2, activated2 = pool.request_swap_in('uav_01', reason='FAILURE')
    assert accepted2 is False
    assert activated2 == ""
