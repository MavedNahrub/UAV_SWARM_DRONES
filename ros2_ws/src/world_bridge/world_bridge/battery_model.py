"""
battery_model.py — Virtual role-conditioned battery drain pure logic.

Decoupled from ROS for unit testing.
"""

from typing import Dict

# Role enum mapping per UAVState.msg:
# SURVEYOR=0, RELAY=1, RTH=2, RESERVE=3, GROUNDED=4, IDLE=5
DEFAULT_ROLE_DRAIN_MULTIPLIERS = {
    0: 1.2,  # SURVEYOR
    1: 1.0,  # RELAY
    2: 1.1,  # RTH
    3: 0.0,  # RESERVE
    4: 0.0,  # GROUNDED
    5: 0.2,  # IDLE
}


class VirtualBatteryModel:
    """Simulates role-dependent battery discharge."""

    def __init__(
        self,
        initial_frac: float = 1.0,
        base_drain_per_sec: float = 0.001,
        role_multipliers: Dict[int, float] = None
    ):
        self.battery_frac = max(0.0, min(1.0, initial_frac))
        self.base_drain_per_sec = base_drain_per_sec
        self.role_multipliers = role_multipliers or DEFAULT_ROLE_DRAIN_MULTIPLIERS

    def step(self, dt_sec: float, role: int) -> float:
        """Advance time by dt_sec under specified role."""
        multiplier = self.role_multipliers.get(role, 1.0)
        drain = self.base_drain_per_sec * multiplier * max(0.0, dt_sec)
        self.battery_frac = max(0.0, self.battery_frac - drain)
        return self.battery_frac

    def set_battery(self, frac: float):
        self.battery_frac = max(0.0, min(1.0, frac))
