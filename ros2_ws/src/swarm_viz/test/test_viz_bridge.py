"""
test_viz_bridge.py — Unit test for VizBridgeNode structure.
"""

import pytest
from swarm_viz.viz_bridge_node import ROLE_COLORS
from swarm_msgs.msg import UAVState


def test_role_colors_mapping():
    assert UAVState.SURVEYOR in ROLE_COLORS
    assert UAVState.RELAY in ROLE_COLORS
    assert UAVState.RTH in ROLE_COLORS
    assert UAVState.RESERVE in ROLE_COLORS
