"""
path_loss.py — Log-distance path-loss model for RF link simulation.

Pure-math module, no ROS dependencies — unit-testable independently.

Per ARCHITECTURE.md Section 5.1:
  rssi = tx_power_dbm - (PL0 + 10 * n * log10(d / d0)) + shadowing_sample
  connected = rssi >= threshold
  packet_loss_prob derived from RSSI margin
"""

import math
import random
from dataclasses import dataclass
from typing import Optional


@dataclass
class LinkParams:
    """RF link model parameters."""
    tx_power_dbm: float = 20.0
    path_loss_exponent: float = 3.0
    pl0_dbm: float = 40.0        # path loss at reference distance d0
    d0_m: float = 1.0             # reference distance
    shadowing_std_db: float = 2.0 # log-normal shadowing std dev
    rssi_threshold_dbm: float = -85.0
    max_range_m: float = 500.0
    base_latency_s: float = 0.01  # minimum per-hop latency


def compute_rssi(distance_m: float, params: LinkParams,
                 shadowing_sample: Optional[float] = None) -> float:
    """
    Compute received signal strength (RSSI) in dBm using log-distance
    path-loss model.

    Args:
        distance_m: Distance between two nodes in meters.
        params: RF link model parameters.
        shadowing_sample: Optional pre-computed shadowing value.
                          If None, drawn from N(0, shadowing_std_db).

    Returns:
        RSSI in dBm.
    """
    if distance_m <= 0:
        return params.tx_power_dbm  # co-located, max signal

    # Clamp to d0 minimum for the log
    d = max(distance_m, params.d0_m)

    # Log-distance path loss
    path_loss = params.pl0_dbm + 10.0 * params.path_loss_exponent * math.log10(d / params.d0_m)

    # Shadowing (log-normal fading)
    if shadowing_sample is None:
        shadowing_sample = random.gauss(0, params.shadowing_std_db)

    rssi = params.tx_power_dbm - path_loss + shadowing_sample
    return rssi


def is_connected(rssi_dbm: float, params: LinkParams) -> bool:
    """Check if link is connected based on RSSI threshold."""
    return rssi_dbm >= params.rssi_threshold_dbm


def compute_packet_loss_prob(rssi_dbm: float, params: LinkParams) -> float:
    """
    Derive packet loss probability from RSSI margin.

    Uses a sigmoid function centered at the threshold:
    - Well above threshold → ~0% loss
    - At threshold → 50% loss
    - Well below threshold → ~100% loss

    The margin_scale controls how quickly loss transitions.
    """
    margin = rssi_dbm - params.rssi_threshold_dbm
    margin_scale = 5.0  # dB per unit — steepness of the sigmoid

    try:
        prob = 1.0 / (1.0 + math.exp(margin / margin_scale))
    except OverflowError:
        prob = 0.0 if margin > 0 else 1.0

    return max(0.0, min(1.0, prob))


def compute_latency(distance_m: float, params: LinkParams) -> float:
    """
    Compute per-hop latency in seconds.

    Base latency + distance-proportional component (simulating processing
    and propagation delay).
    """
    return params.base_latency_s + (distance_m / 300000000.0)  # speed of light


def compute_link_quality(distance_m: float, params: LinkParams,
                         shadowing_sample: Optional[float] = None) -> dict:
    """
    Compute all link quality metrics for a given distance.

    Returns:
        Dict with keys: distance_m, rssi_dbm, connected, packet_loss_prob, latency_s
    """
    rssi = compute_rssi(distance_m, params, shadowing_sample)
    connected = is_connected(rssi, params)
    pkt_loss = compute_packet_loss_prob(rssi, params)
    latency = compute_latency(distance_m, params)

    # If distance exceeds max range, force disconnect
    if distance_m > params.max_range_m:
        connected = False
        pkt_loss = 1.0

    return {
        'distance_m': distance_m,
        'rssi_dbm': rssi,
        'connected': connected,
        'packet_loss_prob': pkt_loss,
        'latency_s': latency,
    }
