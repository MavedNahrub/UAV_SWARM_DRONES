"""
fleet_pool.py — Fleet pool management pure logic.

Decoupled from ROS for unit testing.
"""

from typing import List, Dict, Tuple, Optional, Set


class FleetPool:
    """Manages active vs reserve UAV pools and swap-ins."""

    def __init__(self, active_uav_ids: List[str], reserve_uav_ids: List[str]):
        self.active_uavs: Set[str] = set(active_uav_ids)
        self.reserve_uavs: List[str] = list(reserve_uav_ids)
        self.failed_uavs: Set[str] = set()
        self.rth_uavs: Set[str] = set()

    def request_swap_in(self, departing_uav_id: str, reason: str = 'RTH') -> Tuple[bool, str]:
        """
        Processes a swap-in request for departing_uav_id.
        Returns (accepted: bool, activated_uav_id: str).
        """
        if departing_uav_id in self.active_uavs:
            self.active_uavs.remove(departing_uav_id)
            if reason == 'FAILURE':
                self.failed_uavs.add(departing_uav_id)
            else:
                self.rth_uavs.add(departing_uav_id)

        if self.reserve_uavs:
            activated_id = self.reserve_uavs.pop(0)
            self.active_uavs.add(activated_id)
            return (True, activated_id)
        else:
            return (False, "")

    def get_reserves(self) -> List[str]:
        return list(self.reserve_uavs)

    def get_active(self) -> List[str]:
        return sorted(self.active_uavs)

    def get_failed(self) -> List[str]:
        return sorted(self.failed_uavs)
