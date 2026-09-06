"""
test_metrics_aggregator.py — Unit tests for metrics aggregator and JSONL summary generator.
"""

import pytest
from mission_logger.metrics_aggregator import MetricsAggregator


def test_tx_rx_pdr_and_latency():
    agg = MetricsAggregator(scenario_name="test_pdr")
    agg.record_tx("msg_1", stamp_s=10.0)
    agg.record_tx("msg_2", stamp_s=10.5)

    # Deliver msg_1 with 0.2s latency
    agg.record_rx("msg_1", stamp_s=10.2)

    tick_sum = agg.generate_tick_summary(tick=1, sim_time_s=11.0)
    assert tick_sum["messages_sent"] == 2
    assert tick_sum["messages_delivered"] == 1
    assert tick_sum["messages_dropped"] == 1
    assert abs(tick_sum["avg_latency_s"] - 0.2) < 1e-4

    # Deliver msg_2 with 0.1s latency
    agg.record_rx("msg_2", stamp_s=10.6)

    miss_sum = agg.generate_mission_summary(mission_duration_s=12.0)
    assert miss_sum["pdr"] == 1.0
    assert abs(miss_sum["avg_latency_s"] - 0.15) < 1e-4


def test_fault_recording():
    agg = MetricsAggregator(scenario_name="test_fault")
    rec = agg.record_fault(
        uav_id="uav_01",
        fault_type="LINK_LOSS",
        detected_s=5.0,
        recovered_s=7.5
    )
    assert rec["recovery_time_s"] == 2.5

    miss_sum = agg.generate_mission_summary(mission_duration_s=10.0)
    assert miss_sum["avg_recovery_time_s"] == 2.5
    assert miss_sum["max_recovery_time_s"] == 2.5


def test_collision_recording():
    agg = MetricsAggregator(scenario_name="test_collision")
    rec = agg.record_collision("uav_00", "uav_01", distance_m=3.5, min_sep_m=8.0)
    assert rec["type"] == "collision_event"

    miss_sum = agg.generate_mission_summary(mission_duration_s=10.0)
    assert miss_sum["collision_count"] == 1
