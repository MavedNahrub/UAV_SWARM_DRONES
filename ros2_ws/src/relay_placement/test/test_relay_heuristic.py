"""test_relay_heuristic.py — Unit tests for relay placement logic."""
import pytest
import networkx as nx
from relay_placement.relay_heuristic import (
    check_gcs_connectivity, compute_relay_positions
)

class TestGCSConnectivity:
    def test_all_connected(self):
        g = nx.Graph()
        g.add_edge('uav_00', 'GCS', weight=0.01)
        g.add_edge('uav_01', 'GCS', weight=0.01)
        ok, disc = check_gcs_connectivity(g, ['uav_00', 'uav_01'])
        assert ok is True
        assert len(disc) == 0

    def test_one_disconnected(self):
        g = nx.Graph()
        g.add_edge('uav_00', 'GCS', weight=0.01)
        g.add_node('uav_01')
        ok, disc = check_gcs_connectivity(g, ['uav_00', 'uav_01'])
        assert ok is False
        assert 'uav_01' in disc

    def test_no_gcs(self):
        g = nx.Graph()
        g.add_node('uav_00')
        ok, disc = check_gcs_connectivity(g, ['uav_00'])
        assert ok is False

class TestRelayPositions:
    def test_no_relay_needed_when_connected(self):
        g = nx.Graph()
        g.add_edge('uav_00', 'GCS', weight=0.01)
        positions = {'uav_00': (10, 0, 0)}
        result = compute_relay_positions(
            positions, (0, 0, 0), ['uav_00'], [], g)
        assert len(result) == 0

    def test_relay_assigned_when_disconnected(self):
        g = nx.Graph()
        g.add_node('uav_00')  # surveyor, disconnected
        g.add_node('GCS')
        g.add_node('uav_01')  # available relay
        positions = {'uav_00': (300, 0, 0), 'uav_01': (100, 0, 0)}
        result = compute_relay_positions(
            positions, (0, 0, 0), ['uav_00'], ['uav_01'], g)
        assert len(result) == 1
        assert result[0]['uav_id'] == 'uav_01'

    def test_relay_position_is_midpoint(self):
        g = nx.Graph()
        g.add_node('uav_00')
        g.add_node('GCS')
        g.add_node('uav_01')
        positions = {'uav_00': (200, 0, 0), 'uav_01': (50, 0, 0)}
        result = compute_relay_positions(
            positions, (0, 0, 0), ['uav_00'], ['uav_01'], g)
        assert len(result) == 1
        tp = result[0]['target_position']
        assert abs(tp[0] - 100.0) < 1.0  # midpoint of 200 and GCS at 0
