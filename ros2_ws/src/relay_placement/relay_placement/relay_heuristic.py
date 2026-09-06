"""
relay_heuristic.py — Greedy minimal-connectivity relay placement.

Pure logic, no ROS dependencies.

Given a connectivity graph of UAVs + GCS, determines which UAVs should
hold relay position to maintain GCS connectivity for all surveyors.
"""

import networkx as nx
from typing import Dict, List, Set, Tuple, Optional


def check_gcs_connectivity(graph: nx.Graph, surveyor_ids: List[str],
                           gcs_id: str = 'GCS') -> Tuple[bool, Set[str]]:
    """
    Check if all surveyors can reach GCS through the mesh.

    Returns:
        (all_connected, disconnected_set)
    """
    if gcs_id not in graph:
        return False, set(surveyor_ids)

    disconnected = set()
    for sid in surveyor_ids:
        if sid not in graph:
            disconnected.add(sid)
            continue
        if not nx.has_path(graph, sid, gcs_id):
            disconnected.add(sid)

    return len(disconnected) == 0, disconnected


def compute_relay_positions(
    uav_positions: Dict[str, Tuple[float, float, float]],
    gcs_position: Tuple[float, float, float],
    surveyor_ids: List[str],
    available_relay_ids: List[str],
    connectivity_graph: nx.Graph,
    gcs_id: str = 'GCS',
) -> List[dict]:
    """
    Greedy relay placement heuristic.

    For each disconnected surveyor, find the best available relay UAV
    to position between the disconnected UAV and the nearest connected node.

    Returns list of relay assignments: [{'uav_id': ..., 'target_position': (x,y,z)}]
    """
    all_connected, disconnected = check_gcs_connectivity(
        connectivity_graph, surveyor_ids, gcs_id
    )

    if all_connected:
        return []  # No relays needed

    relay_assignments = []
    used_relays: Set[str] = set()

    for disc_id in sorted(disconnected):
        if not available_relay_ids:
            break

        disc_pos = uav_positions.get(disc_id)
        if disc_pos is None:
            continue

        # Find the nearest connected node to place relay between
        nearest_connected = _find_nearest_connected(
            disc_pos, uav_positions, gcs_position,
            connectivity_graph, gcs_id, disconnected
        )

        if nearest_connected is None:
            nearest_connected_pos = gcs_position
        else:
            if nearest_connected == gcs_id:
                nearest_connected_pos = gcs_position
            else:
                nearest_connected_pos = uav_positions.get(
                    nearest_connected, gcs_position
                )

        # Place relay at midpoint between disconnected UAV and nearest
        # connected node
        relay_pos = (
            (disc_pos[0] + nearest_connected_pos[0]) / 2.0,
            (disc_pos[1] + nearest_connected_pos[1]) / 2.0,
            (disc_pos[2] + nearest_connected_pos[2]) / 2.0,
        )

        # Find the closest available relay UAV to this position
        best_relay = _find_closest_relay(
            relay_pos, available_relay_ids, uav_positions, used_relays
        )

        if best_relay:
            relay_assignments.append({
                'uav_id': best_relay,
                'target_position': relay_pos,
                'for_surveyor': disc_id,
            })
            used_relays.add(best_relay)

    return relay_assignments


def _find_nearest_connected(
    target_pos: Tuple[float, float, float],
    uav_positions: Dict[str, Tuple[float, float, float]],
    gcs_position: Tuple[float, float, float],
    graph: nx.Graph,
    gcs_id: str,
    disconnected: Set[str],
) -> Optional[str]:
    """Find the nearest node that IS connected to GCS."""
    import math

    best_id = None
    best_dist = float('inf')

    # Check GCS
    dist_gcs = math.sqrt(sum((a - b) ** 2 for a, b in zip(target_pos, gcs_position)))
    if gcs_id in graph:
        best_id = gcs_id
        best_dist = dist_gcs

    # Check other connected UAVs
    for uid, pos in uav_positions.items():
        if uid in disconnected:
            continue
        if uid not in graph:
            continue
        if not nx.has_path(graph, uid, gcs_id):
            continue

        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(target_pos, pos)))
        if dist < best_dist:
            best_dist = dist
            best_id = uid

    return best_id


def _find_closest_relay(
    target_pos: Tuple[float, float, float],
    relay_ids: List[str],
    uav_positions: Dict[str, Tuple[float, float, float]],
    used: Set[str],
) -> Optional[str]:
    """Find the closest available relay UAV to the target position."""
    import math

    best_id = None
    best_dist = float('inf')

    for rid in relay_ids:
        if rid in used:
            continue
        pos = uav_positions.get(rid)
        if pos is None:
            continue

        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(target_pos, pos)))
        if dist < best_dist:
            best_dist = dist
            best_id = rid

    return best_id
