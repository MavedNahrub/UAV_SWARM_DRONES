"""
apf_safety.py — Artificial Potential Field (APF) and velocity clamping logic.

Pure logic, decoupled from ROS for unit testing.
"""

import math
from typing import Dict, List, Tuple, Optional


def norm_3d(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def clamp_magnitude(v: Tuple[float, float, float], max_val: float) -> Tuple[float, float, float]:
    n = norm_3d(v)
    if n > max_val and n > 1e-6:
        scale = max_val / n
        return (v[0] * scale, v[1] * scale, v[2] * scale)
    return v


def compute_apf_safe_velocity(
    uav_id: str,
    proposed_vel: Tuple[float, float, float],
    current_pos: Tuple[float, float, float],
    peer_positions: Dict[str, Tuple[float, float, float]],
    obstacles: List[Tuple[Tuple[float, float, float], float]],  # [((cx, cy, cz), radius), ...]
    geofence_polygon: List[Tuple[float, float]],
    min_separation_m: float = 8.0,
    obstacle_safety_margin_m: float = 5.0,
    max_speed_mps: float = 12.0,
    k_rep_peer: float = 5.0,
    k_rep_obs: float = 6.0,
    dt_lookahead: float = 0.5,
) -> Tuple[Tuple[float, float, float], List[str]]:
    """
    Computes safety-filtered velocity command using Artificial Potential Field.

    Returns:
        (safe_vel: (vx, vy, vz), violations: list of descriptions)
    """
    vx, vy, vz = proposed_vel
    px, py, pz = current_pos
    violations = []

    rep_vx, rep_vy, rep_vz = 0.0, 0.0, 0.0

    # 1. Peer UAV repulsion
    for peer_id, peer_pos in peer_positions.items():
        if peer_id == uav_id:
            continue
        dx = px - peer_pos[0]
        dy = py - peer_pos[1]
        dz = pz - peer_pos[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < min_separation_m:
            violations.append(f"PEER_PROXIMITY:{peer_id}:{dist:.2f}")

        # Repulsion active up to 2x min_separation
        rep_radius = min_separation_m * 2.0
        if 0.01 < dist < rep_radius:
            # Force proportional to 1/d - 1/d0
            force = k_rep_peer * (1.0 / dist - 1.0 / rep_radius) / (dist * dist)
            rep_vx += (dx / dist) * force
            rep_vy += (dy / dist) * force
            rep_vz += (dz / dist) * force

    # 2. Obstacle repulsion
    for obs_center, obs_radius in obstacles:
        dx = px - obs_center[0]
        dy = py - obs_center[1]
        dz = pz - obs_center[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        boundary_dist = dist - obs_radius

        if boundary_dist < obstacle_safety_margin_m:
            violations.append(f"OBSTACLE_PROXIMITY:{boundary_dist:.2f}")

        rep_radius = obstacle_safety_margin_m * 2.5
        if 0.01 < boundary_dist < rep_radius:
            force = k_rep_obs * (1.0 / max(0.1, boundary_dist) - 1.0 / rep_radius) / max(0.1, boundary_dist**2)
            rep_vx += (dx / dist) * force
            rep_vy += (dy / dist) * force
            rep_vz += (dz / dist) * force

    # 3. Geofence boundary clamping (if polygon defined)
    if len(geofence_polygon) >= 3:
        # Check projected position
        pred_x = px + (vx + rep_vx) * dt_lookahead
        pred_y = py + (vy + rep_vy) * dt_lookahead

        # Simple bounding box / centroid restoration if projecting outside
        poly_xs = [pt[0] for pt in geofence_polygon]
        poly_ys = [pt[1] for pt in geofence_polygon]
        min_x, max_x = min(poly_xs), max(poly_xs)
        min_y, max_y = min(poly_ys), max(poly_ys)

        buf = 10.0
        if pred_x < min_x + buf:
            rep_vx += (min_x + buf - pred_x) * 2.0
            violations.append("GEOFENCE_X_MIN")
        elif pred_x > max_x - buf:
            rep_vx -= (pred_x - (max_x - buf)) * 2.0
            violations.append("GEOFENCE_X_MAX")

        if pred_y < min_y + buf:
            rep_vy += (min_y + buf - pred_y) * 2.0
            violations.append("GEOFENCE_Y_MIN")
        elif pred_y > max_y - buf:
            rep_vy -= (pred_y - (max_y - buf)) * 2.0
            violations.append("GEOFENCE_Y_MAX")

    total_vx = vx + rep_vx
    total_vy = vy + rep_vy
    total_vz = vz + rep_vz

    safe_vel = clamp_magnitude((total_vx, total_vy, total_vz), max_speed_mps)
    return safe_vel, violations
