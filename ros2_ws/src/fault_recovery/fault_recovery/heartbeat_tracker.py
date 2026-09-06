"""
heartbeat_tracker.py — Pure logic for heartbeat tracking and fault detection.

Decoupled from ROS for unit testing.
"""

from typing import Dict, List, Optional, Tuple, Set


class HeartbeatTracker:
    """Tracks heartbeats from peer UAVs and identifies link loss or node failure."""

    def __init__(self, my_id: str, timeout_s: float = 5.0):
        self.my_id = my_id
        self.timeout_s = timeout_s
        self.last_heartbeat: Dict[str, float] = {}
        self.active_faults: Dict[str, float] = {}  # uav_id -> detected_time
        self.known_peers: Set[str] = set()

    def register_peer(self, peer_id: str, current_time: float = 0.0):
        if peer_id != self.my_id:
            self.known_peers.add(peer_id)
            if peer_id not in self.last_heartbeat:
                self.last_heartbeat[peer_id] = current_time

    def record_heartbeat(self, peer_id: str, current_time: float) -> Optional[Tuple[str, float, float]]:
        """
        Record receiving a heartbeat from peer_id.
        Returns (peer_id, detected_time, recovered_time) if this heartbeat recovers an active fault.
        """
        if peer_id == self.my_id:
            return None

        self.known_peers.add(peer_id)
        self.last_heartbeat[peer_id] = current_time

        if peer_id in self.active_faults:
            detected_time = self.active_faults.pop(peer_id)
            return (peer_id, detected_time, current_time)
        return None

    def check_timeouts(self, current_time: float) -> List[Tuple[str, float]]:
        """
        Check which known peers have timed out and not yet been flagged as active fault.
        Returns list of newly detected faults: [(peer_id, detected_time), ...]
        """
        new_faults = []
        for peer_id in sorted(self.known_peers):
            if peer_id == self.my_id:
                continue
            last = self.last_heartbeat.get(peer_id, 0.0)
            elapsed = current_time - last
            if elapsed >= self.timeout_s:
                if peer_id not in self.active_faults:
                    self.active_faults[peer_id] = current_time
                    new_faults.append((peer_id, current_time))
        return new_faults

    def is_faulted(self, peer_id: str) -> bool:
        return peer_id in self.active_faults

    def get_active_faults(self) -> Dict[str, float]:
        return dict(self.active_faults)
