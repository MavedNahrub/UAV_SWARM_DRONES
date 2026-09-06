"""
geometry_utils.py — Pure logic geometry checks for geofence, obstacles, and separation.
"""

import math
from typing import List, Tuple


def point_in_polygon_2d(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    """
    Ray-casting algorithm for point-in-polygon test in 2D (x, y).
    Returns True if point is inside or on the boundary.
    """
    if len(polygon) < 3:
        return True  # degenerate polygon treated as unconstrained

    x, y = point[0], point[1]
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0]

    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def check_obstacle_collision(
    point: Tuple[float, float, float],
    obstacle_center: Tuple[float, float, float],
    obstacle_radius_m: float,
    safety_margin_m: float = 0.0
) -> bool:
    """
    Checks if 3D point is within obstacle radius + safety margin.
    """
    dx = point[0] - obstacle_center[0]
    dy = point[1] - obstacle_center[1]
    dz = point[2] - obstacle_center[2]
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    return dist < (obstacle_radius_m + safety_margin_m)


def compute_pairwise_separation(
    positions: dict  # uav_id -> (x, y, z)
) -> List[Tuple[str, str, float]]:
    """
    Computes pairwise 3D Euclidean distances between all UAVs.
    Returns sorted list of (uav_a, uav_b, distance_m) with uav_a < uav_b.
    """
    results = []
    keys = sorted(positions.keys())
    n = len(keys)
    for i in range(n):
        for j in range(i + 1, n):
            uav_a, uav_b = keys[i], keys[j]
            pa, pb = positions[uav_a], positions[uav_b]
            dist = math.sqrt((pa[0] - pb[0])**2 + (pa[1] - pb[1])**2 + (pa[2] - pb[2])**2)
            results.append((uav_a, uav_b, dist))
    return results
