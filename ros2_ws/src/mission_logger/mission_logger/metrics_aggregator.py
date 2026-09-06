"""
metrics_aggregator.py — Pure logic for aggregating swarm metrics and formatting JSONL records.

Per ARCHITECTURE.md Section 7.
"""

import math
from typing import Dict, List, Optional, Any


class MetricsAggregator:
    """Aggregates simulation metrics and computes summary evaluation rubric."""

    def __init__(self, scenario_name: str = "default", seed: int = 42):
        self.scenario_name = scenario_name
        self.seed = seed
        self.tick = 0
        self.sim_time_s = 0.0

        # Communication counters
        self.sent_messages: Dict[str, float] = {}       # msg_id -> origin_stamp
        self.delivered_messages: Dict[str, float] = {}  # msg_id -> latency_s
        self.dropped_count = 0

        # Battery tracking
        self.uav_batteries: Dict[str, float] = {}

        # Roles and connections
        self.active_uavs: set = set()
        self.connected_uavs: set = set()
        self.connectivity_fraction = 1.0
        self.active_relay_count = 0

        # PoI tracking
        self.poi_states: Dict[str, Dict[str, Any]] = {}  # poi_id -> {status, priority, weight}
        self.poi_events: List[Dict[str, Any]] = []

        # Event records
        self.fault_events: List[Dict[str, Any]] = []
        self.relay_events: List[Dict[str, Any]] = []
        self.collision_events: List[Dict[str, Any]] = []
        self.geofence_violations: List[Dict[str, Any]] = []

        self.relay_reallocations_count = 0

    def record_tx(self, msg_id: str, stamp_s: float):
        if msg_id and msg_id not in self.sent_messages:
            self.sent_messages[msg_id] = stamp_s

    def record_rx(self, msg_id: str, stamp_s: float):
        if msg_id:
            send_time = self.sent_messages.get(msg_id, stamp_s)
            latency = max(0.0, stamp_s - send_time)
            self.delivered_messages[msg_id] = latency

    def record_battery(self, uav_id: str, battery_frac: float):
        self.uav_batteries[uav_id] = battery_frac

    def record_fault(self, uav_id: str, fault_type: str, detected_s: float, recovered_s: Optional[float] = None) -> Dict[str, Any]:
        rec_time = (recovered_s - detected_s) if recovered_s is not None else 0.0
        rec = {
            "type": "fault_event",
            "tick": self.tick,
            "uav_id": uav_id,
            "fault_type": fault_type,
            "detected_tick": int(detected_s * 2.0),
            "recovered_tick": int(recovered_s * 2.0) if recovered_s is not None else 0,
            "recovery_time_s": rec_time,
        }
        self.fault_events.append(rec)
        return rec

    def record_relay(self, relay_uavs: List[str], reallocation_triggered: bool, trigger_reason: str) -> Dict[str, Any]:
        if reallocation_triggered:
            self.relay_reallocations_count += 1
        self.active_relay_count = len(relay_uavs)
        rec = {
            "type": "relay_event",
            "tick": self.tick,
            "relay_uavs": relay_uavs,
            "reallocation_triggered": reallocation_triggered,
            "trigger_reason": trigger_reason,
        }
        self.relay_events.append(rec)
        return rec

    def record_collision(self, uav_a: str, uav_b: str, distance_m: float, min_sep_m: float) -> Dict[str, Any]:
        rec = {
            "type": "collision_event",
            "tick": self.tick,
            "uav_a": uav_a,
            "uav_b": uav_b,
            "distance_m": distance_m,
            "min_separation_m": min_sep_m,
        }
        self.collision_events.append(rec)
        return rec

    def record_geofence(self, uav_id: str, pos: List[float], corrected_pos: List[float]) -> Dict[str, Any]:
        rec = {
            "type": "geofence_violation",
            "tick": self.tick,
            "uav_id": uav_id,
            "position": pos,
            "corrected_position": corrected_pos,
        }
        self.geofence_violations.append(rec)
        return rec

    def record_poi_event(self, poi_id: str, event: str, priority: int, weight: float) -> Dict[str, Any]:
        rec = {
            "type": "poi_event",
            "tick": self.tick,
            "poi_id": poi_id,
            "event": event,
            "priority": priority,
            "priority_weight": weight,
        }
        self.poi_events.append(rec)
        return rec

    def generate_tick_summary(self, tick: int, sim_time_s: float) -> Dict[str, Any]:
        self.tick = tick
        self.sim_time_s = sim_time_s

        sent = len(self.sent_messages)
        delivered = len(self.delivered_messages)
        dropped = max(0, sent - delivered)

        latencies = list(self.delivered_messages.values())
        avg_lat = sum(latencies) / len(latencies) if latencies else 0.0

        b_vals = list(self.uav_batteries.values())
        b_min = min(b_vals) if b_vals else 1.0
        b_mean = sum(b_vals) / len(b_vals) if b_vals else 1.0

        # PoI stats
        p_pending = sum(1 for p in self.poi_states.values() if p.get('status') == 0)
        p_prog = sum(1 for p in self.poi_states.values() if p.get('status') in (1, 2))
        p_surv = sum(1 for p in self.poi_states.values() if p.get('status') == 3)
        p_exp = sum(1 for p in self.poi_states.values() if p.get('status') == 4)

        return {
            "type": "tick_summary",
            "tick": self.tick,
            "sim_time_s": round(self.sim_time_s, 2),
            "num_active_uavs": len(self.active_uavs),
            "num_connected_uavs": len(self.connected_uavs),
            "connectivity_fraction": round(self.connectivity_fraction, 3),
            "messages_sent": sent,
            "messages_delivered": delivered,
            "messages_dropped": dropped,
            "avg_latency_s": round(avg_lat, 4),
            "active_relay_count": self.active_relay_count,
            "poi_pending": p_pending,
            "poi_in_progress": p_prog,
            "poi_surveyed": p_surv,
            "poi_expired": p_exp,
            "battery_min_frac": round(b_min, 3),
            "battery_mean_frac": round(b_mean, 3),
        }

    def generate_mission_summary(self, mission_duration_s: float) -> Dict[str, Any]:
        sent = len(self.sent_messages)
        delivered = len(self.delivered_messages)
        pdr = (delivered / sent) if sent > 0 else 1.0

        latencies = list(self.delivered_messages.values())
        avg_lat = sum(latencies) / len(latencies) if latencies else 0.0

        total_pois = len(self.poi_states)
        surveyed_pois = sum(1 for p in self.poi_states.values() if p.get('status') == 3)
        completion_rate = (surveyed_pois / total_pois) if total_pois > 0 else 1.0

        # Weighted score
        score = sum(p.get('weight', 1.0) for p in self.poi_states.values() if p.get('status') == 3)

        # Recovery times
        recovery_times = [f['recovery_time_s'] for f in self.fault_events if f.get('recovery_time_s', 0.0) > 0]
        avg_rec = sum(recovery_times) / len(recovery_times) if recovery_times else 0.0
        max_rec = max(recovery_times) if recovery_times else 0.0

        return {
            "type": "mission_summary",
            "completion_rate": round(completion_rate, 3),
            "priority_weighted_score": round(score, 1),
            "pdr": round(pdr, 3),
            "avg_latency_s": round(avg_lat, 4),
            "connectivity_availability": round(self.connectivity_fraction, 3),
            "total_downtime_s": round(sum(recovery_times), 2),
            "relay_reallocations_count": self.relay_reallocations_count,
            "avg_recovery_time_s": round(avg_rec, 2),
            "max_recovery_time_s": round(max_rec, 2),
            "collision_count": len(self.collision_events),
            "mission_duration_s": round(mission_duration_s, 2),
            "seed": self.seed,
            "scenario_name": self.scenario_name,
        }
