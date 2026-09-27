"""
scenario_orchestrator_node.py — Reads scenario YAML and dispatches timed events.

Responsibilities:
- Distributes initial PoI list to all CBBA nodes at startup
- Injects fault events at scheduled times
- Injects priority interrupts (new PoIs) at scheduled times
- Shuts down the mission when duration elapses
"""

import json
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as MsgTime
from geometry_msgs.msg import Point
from swarm_msgs.msg import SwarmMessage, FaultEvent


def float_to_ros_time(t: float) -> MsgTime:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return MsgTime(sec=sec, nanosec=nanosec)


class ScenarioOrchestratorNode(Node):
    """Central scenario orchestrator — dispatches timed events."""

    def __init__(self):
        super().__init__('scenario_orchestrator_node')

        self.declare_parameter('scenario', '')
        self.declare_parameter('uav_ids', '')

        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value
        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]

        self.scenario_data = {}
        self.pois = []
        self.fault_injections = []
        self.priority_interrupts = []
        self.mission_duration_s = 300.0
        self.roles = {}

        self._load_scenario(scenario_path)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=50)

        # Publishers: one comm_rx per UAV for distributing PoI alerts
        self.comm_rx_pubs = {}
        for uid in self.uav_ids:
            self.comm_rx_pubs[uid] = self.create_publisher(
                SwarmMessage, f'/{uid}/comm_rx', qos)

        # Pose subscribers for readiness tracking — wait for all active UAVs
        # to report position before distributing PoIs
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pose_received = {uid: False for uid in self.uav_ids}
        self.active_uav_ids = [
            uid for uid in self.uav_ids
            if self.roles.get(uid, 'SURVEYOR') != 'RESERVE'
        ]
        for uid in self.uav_ids:
            self.create_subscription(
                Point, f'/{uid}/pose',
                lambda msg, u=uid: self._on_pose(u, msg),
                best_effort_qos)

        # Fault event publisher
        self.fault_event_pub = self.create_publisher(
            FaultEvent, '/swarm/fault_events', qos)

        # State tracking
        self.start_time = None
        self.initial_pois_sent = False
        self.fired_faults = set()
        self.fired_interrupts = set()

        # Timer at 1 Hz
        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'ScenarioOrchestrator started: {len(self.pois)} PoIs, '
            f'{len(self.fault_injections)} fault injections, '
            f'{len(self.priority_interrupts)} priority interrupts, '
            f'duration={self.mission_duration_s}s')

    def _load_scenario(self, path: str):
        if not path or not os.path.exists(path):
            self.get_logger().warn(f'Scenario file not found: {path}')
            return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.scenario_data = data

            mission = data.get('mission', {})
            self.mission_duration_s = float(mission.get('duration_s', 300.0))

            self.roles = data.get('uavs', {}).get('roles', {})

            for poi_data in data.get('pois', []):
                self.pois.append({
                    'poi_id': poi_data['poi_id'],
                    'position': poi_data['position'],
                    'priority': poi_data.get('priority', 1),
                    'survey_duration_s': poi_data.get('survey_duration_s', 20.0),
                })

            for fi in data.get('fault_injection', []):
                self.fault_injections.append({
                    'time_s': fi['time_s'],
                    'target_uav': fi['target_uav'],
                    'fault_type': fi['fault_type'],
                })

            for pi in data.get('priority_interrupts', []):
                self.priority_interrupts.append({
                    'time_s': pi['time_s'],
                    'poi_id': pi['poi_id'],
                    'position': pi['position'],
                    'priority': pi.get('priority', 3),
                    'survey_duration_s': pi.get('survey_duration_s', 15.0),
                })

        except Exception as e:
            self.get_logger().error(f'Error loading scenario: {e}')

    def _on_pose(self, uav_id: str, msg):
        """Track UAV position readiness."""
        if not self.pose_received.get(uav_id, False):
            self.pose_received[uav_id] = True
            self.get_logger().info(f'UAV {uav_id} reported position — marking as ready')

    def _all_active_uavs_ready(self) -> bool:
        """Check if all non-RESERVE UAVs have reported valid position."""
        return all(self.pose_received.get(uid, False) for uid in self.active_uav_ids)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        if self.start_time is None:
            self.start_time = self._now_sec()

        elapsed = self._now_sec() - self.start_time

        # 1. Send initial PoIs when all active UAVs are airborne (replaces fixed 10s delay)
        if not self.initial_pois_sent:
            if self._all_active_uavs_ready() or elapsed >= 60.0:
                if elapsed >= 60.0:
                    self.get_logger().warn(
                        'Timeout waiting for all UAVs — distributing PoIs anyway')
                self._distribute_initial_pois()
                self.initial_pois_sent = True
            elif elapsed >= 15.0 and int(elapsed) % 10 == 0:
                ready = sum(1 for uid in self.active_uav_ids
                            if self.pose_received.get(uid, False))
                self.get_logger().warn(
                    f'Waiting for UAVs: {ready}/{len(self.active_uav_ids)} ready')

        # 2. Check fault injections
        for i, fi in enumerate(self.fault_injections):
            if i not in self.fired_faults and elapsed >= fi['time_s']:
                self._inject_fault(fi)
                self.fired_faults.add(i)

        # 3. Check priority interrupts
        for i, pi in enumerate(self.priority_interrupts):
            if i not in self.fired_interrupts and elapsed >= pi['time_s']:
                self._inject_priority_interrupt(pi)
                self.fired_interrupts.add(i)

        # 4. Check mission duration (log warning only, don't shut down)
        if elapsed >= self.mission_duration_s:
            self.get_logger().info(
                f'Mission duration {self.mission_duration_s}s elapsed. '
                f'Scenario complete.')
            self.timer.cancel()

    def _distribute_initial_pois(self):
        """Send all initial PoIs to all CBBA nodes via POI_ALERT messages."""
        self.get_logger().info(f'Distributing {len(self.pois)} initial PoIs to {len(self.uav_ids)} UAVs')

        for poi in self.pois:
            payload = {
                'poi_id': poi['poi_id'],
                'position': poi['position'],
                'priority': poi['priority'],
                'survey_duration_s': poi['survey_duration_s'],
            }

            for uid in self.uav_ids:
                msg = SwarmMessage()
                msg.msg_id = f'poi_init_{poi["poi_id"]}_{uid}'
                msg.src_uav_id = 'orchestrator'
                msg.dst_uav_id = uid
                msg.msg_type = 'POI_ALERT'
                msg.payload_json = json.dumps(payload)
                msg.origin_stamp = self.get_clock().now().to_msg()
                msg.hop_count = 0
                msg.ttl = 10

                if uid in self.comm_rx_pubs:
                    self.comm_rx_pubs[uid].publish(msg)

        self.get_logger().info('Initial PoIs distributed successfully')

    def _inject_fault(self, fi: dict):
        """Inject a fault event for a target UAV."""
        self.get_logger().warn(
            f'FAULT INJECTION: {fi["fault_type"]} on {fi["target_uav"]} at t={fi["time_s"]}s')

        event = FaultEvent()
        event.detected_stamp = self.get_clock().now().to_msg()
        event.uav_id = fi['target_uav']
        event.fault_type = fi['fault_type']
        # recovered_stamp left at zero (not recovered)
        self.fault_event_pub.publish(event)

    def _inject_priority_interrupt(self, pi: dict):
        """Inject a new high-priority PoI mid-mission."""
        self.get_logger().warn(
            f'PRIORITY INTERRUPT: {pi["poi_id"]} (priority={pi["priority"]}) at t={pi["time_s"]}s')

        payload = {
            'poi_id': pi['poi_id'],
            'position': pi['position'],
            'priority': pi['priority'],
            'survey_duration_s': pi['survey_duration_s'],
        }

        for uid in self.uav_ids:
            msg = SwarmMessage()
            msg.msg_id = f'poi_alert_{pi["poi_id"]}_{uid}'
            msg.src_uav_id = 'orchestrator'
            msg.dst_uav_id = uid
            msg.msg_type = 'POI_ALERT'
            msg.payload_json = json.dumps(payload)
            msg.origin_stamp = self.get_clock().now().to_msg()
            msg.hop_count = 0
            msg.ttl = 10

            if uid in self.comm_rx_pubs:
                self.comm_rx_pubs[uid].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
