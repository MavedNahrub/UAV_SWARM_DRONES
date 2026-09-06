"""
rth_logic.py — Battery monitoring, reachability estimation, and RTH logic.

Decoupled from ROS for direct unit testing.
"""

import math
from typing import Tuple


def compute_distance_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def estimate_battery_required(
    current_pos: Tuple[float, float, float],
    target_pos: Tuple[float, float, float],
    gcs_pos: Tuple[float, float, float],
    speed_mps: float = 10.0,
    survey_duration_s: float = 20.0,
    drain_rate_per_sec: float = 0.001
) -> float:
    """
    Estimates the battery fraction consumed to fly from current_pos to target_pos,
    conduct a survey of survey_duration_s, and then return to gcs_pos.
    """
    dist_to_target = compute_distance_3d(current_pos, target_pos)
    dist_to_gcs = compute_distance_3d(target_pos, gcs_pos)
    total_distance = dist_to_target + dist_to_gcs
    flight_time = total_distance / max(speed_mps, 0.1)
    total_time = flight_time + survey_duration_s
    return total_time * drain_rate_per_sec


def is_reachable(
    current_pos: Tuple[float, float, float],
    target_pos: Tuple[float, float, float],
    gcs_pos: Tuple[float, float, float],
    current_battery_frac: float,
    critical_frac: float = 0.20,
    speed_mps: float = 10.0,
    survey_duration_s: float = 20.0,
    drain_rate_per_sec: float = 0.001
) -> bool:
    """
    Returns True if target can be surveyed and UAV can still return to GCS
    with at least critical_frac remaining.
    """
    available_battery = current_battery_frac - critical_frac
    if available_battery <= 0.0:
        return False
    required = estimate_battery_required(
        current_pos, target_pos, gcs_pos, speed_mps, survey_duration_s, drain_rate_per_sec)
    return available_battery >= required


def should_trigger_rth(
    current_pos: Tuple[float, float, float],
    gcs_pos: Tuple[float, float, float],
    current_battery_frac: float,
    critical_frac: float = 0.20,
    speed_mps: float = 10.0,
    drain_rate_per_sec: float = 0.001
) -> bool:
    """
    Returns True if UAV must immediately Return-To-Home (RTH) to safely reach GCS
    before battery drops below critical_frac.
    """
    if current_battery_frac <= critical_frac:
        return True
    dist_to_gcs = compute_distance_3d(current_pos, gcs_pos)
    flight_time = dist_to_gcs / max(speed_mps, 0.1)
    battery_needed = flight_time * drain_rate_per_sec
    return (current_battery_frac - battery_needed) <= critical_frac
