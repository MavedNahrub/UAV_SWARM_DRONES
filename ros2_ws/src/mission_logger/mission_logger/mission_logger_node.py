"""
mission_logger_node.py — JSONL mission logger for PUSHPAK UAV-X.

Per ARCHITECTURE.md Section 7:
- Subscribes to all mission topics
- Emits line-by-line JSONL records (tick_summary, poi_event, fault_event, relay_event,
  collision_event, geofence_violation, mission_summary)
"""

import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from swarm_msgs.msg import (
    SwarmMessage, UAVState, LinkStateArray, FaultEvent,
    RelayDecision, CollisionEvent, GeofenceViolation
)
from .metrics_aggregator import MetricsAggregator


class MissionLoggerNode(Node):
    """Central logging node writing evaluation-ready JSONL logs."""

    def __init__(self):
        super().__init__('mission_logger_node')

        self.declare_parameter('uav_ids', 'uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
        self.declare_parameter('out_path', 'logs/run.jsonl')
        self.declare_parameter('scenario_name', 'scenario_basic')
        self.declare_parameter('scenario', '')
        self.declare_parameter('log_rate_hz', 2.0)

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]
        self.out_path = self.get_parameter('out_path').get_parameter_value().string_value
        scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value
        log_rate = self.get_parameter('log_rate_hz').value

        self.aggregator = MetricsAggregator(scenario_name=scenario_name)

        if scenario_path and os.path.exists(scenario_path):
            try:
                import yaml
                with open(scenario_path, 'r') as f:
                    sdata = yaml.safe_load(f) or {}
                for poi_data in sdata.get('pois', []):
                    p_id = poi_data.get('poi_id')
                    pri = poi_data.get('priority', 1)
                    weights = {1: 1.0, 2: 2.0, 3: 3.0}
                    self.aggregator.poi_states[p_id] = {
                        'status': 0,
                        'priority': pri,
                        'weight': weights.get(pri, 1.0)
                    }
                m_name = sdata.get('mission', {}).get('name')
                if m_name:
                    self.aggregator.scenario_name = m_name
            except Exception as e:
                self.get_logger().warn(f'Could not load initial PoIs from scenario: {e}')

        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        self.log_file = open(self.out_path, 'w', encoding='utf-8')

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=50)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # UAV-specific topics
        for uid in self.uav_ids:
            self._setup_uav_subs(uid, reliable, best_effort)

        # Global swarm topics
        self.create_subscription(LinkStateArray, '/swarm/link_states', self._on_link_states, reliable)
        self.create_subscription(FaultEvent, '/swarm/fault_events', self._on_fault_event, reliable)
        self.create_subscription(RelayDecision, '/swarm/relay_decisions', self._on_relay_decision, reliable)
        self.create_subscription(CollisionEvent, '/swarm/collision_events', self._on_collision_event, reliable)
        self.create_subscription(GeofenceViolation, '/swarm/geofence_violations', self._on_geofence_violation, reliable)

        self.start_time = self._now_sec()
        self.current_tick = 0
        self.mission_summary_written = False
        self.timer = self.create_timer(1.0 / max(log_rate, 0.1), self._log_tick)

        self.get_logger().info(f'MissionLogger started, writing to {self.out_path}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _write_record(self, record: dict):
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()

    def _setup_uav_subs(self, uid: str, reliable: QoSProfile, best_effort: QoSProfile):
        def tx_cb(msg: SwarmMessage, u=uid):
            stamp = msg.origin_stamp.sec + msg.origin_stamp.nanosec * 1e-9
            self.aggregator.record_tx(msg.msg_id, stamp)
            if msg.msg_type == 'POI_REPORT':
                self._handle_poi_report(msg)
            elif msg.msg_type == 'POI_ALERT':
                self._handle_poi_alert(msg)

        def rx_cb(msg: SwarmMessage, u=uid):
            now = self._now_sec()
            self.aggregator.record_rx(msg.msg_id, now)
            if msg.msg_type == 'POI_ALERT':
                self._handle_poi_alert(msg)

        def battery_cb(msg: UAVState, u=uid):
            self.aggregator.record_battery(u, msg.battery_frac)

        def task_cb(msg: UAVState, u=uid):
            if msg.role != UAVState.GROUNDED:
                self.aggregator.active_uavs.add(u)
            if msg.assigned_poi_id:
                p_id = msg.assigned_poi_id
                if p_id not in self.aggregator.poi_states:
                    self.aggregator.poi_states[p_id] = {'status': 1, 'priority': 1, 'weight': 1.0}
                    rec = self.aggregator.record_poi_event(p_id, 'IN_PROGRESS', 1, 1.0)
                    self._write_record(rec)
                elif self.aggregator.poi_states[p_id]['status'] == 0:
                    self.aggregator.poi_states[p_id]['status'] = 1
                    pri = self.aggregator.poi_states[p_id].get('priority', 1)
                    w = self.aggregator.poi_states[p_id].get('weight', 1.0)
                    rec = self.aggregator.record_poi_event(p_id, 'IN_PROGRESS', pri, w)
                    self._write_record(rec)

        self.create_subscription(SwarmMessage, f'/{uid}/comm_tx', tx_cb, reliable)
        self.create_subscription(SwarmMessage, f'/{uid}/comm_rx', rx_cb, reliable)
        self.create_subscription(UAVState, f'/{uid}/battery_state', battery_cb, best_effort)
        self.create_subscription(UAVState, f'/{uid}/task_state', task_cb, reliable)

    def _handle_poi_report(self, msg: SwarmMessage):
        try:
            payload = json.loads(msg.payload_json)
            p_id = payload.get('poi_id')
            pri = int(payload.get('priority', 1))
            weights = {1: 1.0, 2: 2.0, 3: 3.0}
            weight = weights.get(pri, 1.0)
            if p_id:
                if p_id in self.aggregator.poi_states and self.aggregator.poi_states[p_id].get('status') == 3:
                    return
                self.aggregator.poi_states[p_id] = {'status': 3, 'priority': pri, 'weight': weight}
                rec = self.aggregator.record_poi_event(p_id, 'SURVEYED', pri, weight)
                self._write_record(rec)
                self.get_logger().info(f'Logged POI SURVEYED: {p_id}')
        except Exception as e:
            self.get_logger().warn(f'Error logging POI_REPORT: {e}')

    def _handle_poi_alert(self, msg: SwarmMessage):
        try:
            payload = json.loads(msg.payload_json)
            p_id = payload.get('poi_id')
            pri = int(payload.get('priority', 1))
            weights = {1: 1.0, 2: 2.0, 3: 3.0}
            weight = weights.get(pri, 1.0)
            if p_id and p_id not in self.aggregator.poi_states:
                self.aggregator.poi_states[p_id] = {'status': 0, 'priority': pri, 'weight': weight}
                rec = self.aggregator.record_poi_event(p_id, 'ALERTED', pri, weight)
                self._write_record(rec)
                self.get_logger().info(f'Logged POI ALERTED: {p_id} (priority={pri})')
        except Exception as e:
            pass

    def _on_link_states(self, msg: LinkStateArray):
        self.aggregator.connectivity_fraction = msg.connectivity_fraction
        conn = set()
        for l in msg.links:
            if l.connected:
                conn.add(l.uav_a)
                conn.add(l.uav_b)
        self.aggregator.connected_uavs = conn

    def _on_fault_event(self, msg: FaultEvent):
        det_s = msg.detected_stamp.sec + msg.detected_stamp.nanosec * 1e-9
        rec_s = (msg.recovered_stamp.sec + msg.recovered_stamp.nanosec * 1e-9) if msg.recovered_stamp.sec > 0 else None
        rec = self.aggregator.record_fault(msg.uav_id, msg.fault_type, det_s, rec_s)
        self._write_record(rec)

    def _on_relay_decision(self, msg: RelayDecision):
        rec = self.aggregator.record_relay(
            list(msg.relay_uav_ids),
            msg.reallocation_triggered,
            msg.trigger_reason
        )
        self._write_record(rec)

    def _on_collision_event(self, msg: CollisionEvent):
        rec = self.aggregator.record_collision(
            msg.uav_a, msg.uav_b, msg.distance_m, msg.min_separation_m)
        self._write_record(rec)

    def _on_geofence_violation(self, msg: GeofenceViolation):
        rec = self.aggregator.record_geofence(
            msg.uav_id,
            [msg.position.x, msg.position.y, msg.position.z],
            [msg.corrected_position.x, msg.corrected_position.y, msg.corrected_position.z]
        )
        self._write_record(rec)

    def _log_tick(self):
        sim_time = self._now_sec() - self.start_time
        rec = self.aggregator.generate_tick_summary(self.current_tick, sim_time)
        self._write_record(rec)
        self.current_tick += 1

        total_pois = len(self.aggregator.poi_states)
        p_surv = sum(1 for p in self.aggregator.poi_states.values() if p.get('status') == 3)
        if total_pois > 0 and p_surv >= total_pois and not self.mission_summary_written:
            summary = self.aggregator.generate_mission_summary(sim_time)
            self._write_record(summary)
            self.mission_summary_written = True
            self.get_logger().info(f'All {total_pois} PoIs surveyed! Mission summary written.')

    def destroy_node(self):
        if not self.mission_summary_written:
            mission_dur = self._now_sec() - self.start_time
            summary = self.aggregator.generate_mission_summary(mission_dur)
            self._write_record(summary)
            self.mission_summary_written = True
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
