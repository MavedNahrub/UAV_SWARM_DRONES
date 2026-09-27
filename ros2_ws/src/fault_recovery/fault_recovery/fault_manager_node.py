"""
fault_manager_node.py — Per-UAV fault detection and recovery ROS 2 node.

Per ARCHITECTURE.md Section 6.4:
- Subscribes to /uavN/comm_rx for HEARTBEAT messages
- Publishes HEARTBEAT on /uavN/comm_tx on a fixed period
- Tracks peer heartbeat timestamps; flags LINK_LOSS / UAV_FAILURE upon timeout
- Publishes /swarm/fault_events (FaultEvent)
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from swarm_msgs.msg import SwarmMessage, FaultEvent, UAVState
from builtin_interfaces.msg import Time as MsgTime

from .heartbeat_tracker import HeartbeatTracker


def float_to_ros_time(t: float) -> MsgTime:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return MsgTime(sec=sec, nanosec=nanosec)


class FaultManagerNode(Node):
    """Per-UAV fault manager node."""

    def __init__(self):
        super().__init__('fault_manager_node')

        self.declare_parameter('uav_id', '')
        self.declare_parameter('heartbeat_period_s', 1.0)
        self.declare_parameter('timeout_s', 15.0)
        self.declare_parameter('known_peers', '')

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        self.heartbeat_period_s = self.get_parameter('heartbeat_period_s').value
        self.timeout_s = self.get_parameter('timeout_s').value
        known_peers_str = self.get_parameter('known_peers').get_parameter_value().string_value

        self.tracker = HeartbeatTracker(my_id=self.uav_id, timeout_s=self.timeout_s)

        now_sec = self._now_sec()
        if known_peers_str:
            for pid in known_peers_str.split(','):
                pid = pid.strip()
                if pid:
                    self.tracker.register_peer(pid, current_time=now_sec)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=50)

        # Subscribers
        self.create_subscription(SwarmMessage, 'comm_rx', self._on_comm_rx, qos)
        self.create_subscription(UAVState, 'task_state', self._on_task_state, qos)

        # Publishers
        self.comm_tx_pub = self.create_publisher(SwarmMessage, 'comm_tx', qos)
        self.fault_event_pub = self.create_publisher(FaultEvent, '/swarm/fault_events', qos)

        # Timers
        self.hb_timer = self.create_timer(self.heartbeat_period_s, self._send_heartbeat)
        # Watchdog runs every 2s (not 0.5s) to avoid log spam during relay routing
        self.watchdog_timer = self.create_timer(2.0, self._check_timeouts)

        self.get_logger().info(f'Fault manager node initialized for {self.uav_id}')

    def _now_sec(self) -> float:
        t = self.get_clock().now()
        return t.nanoseconds / 1e9

    def _on_comm_rx(self, msg: SwarmMessage):
        if msg.msg_type == 'HEARTBEAT':
            now_sec = self._now_sec()
            recovery = self.tracker.record_heartbeat(msg.src_uav_id, now_sec)
            if recovery:
                peer_id, detected_time, recovered_time = recovery
                self.get_logger().warn(f'[{self.uav_id}] Peer {peer_id} RECOVERED from fault!')
                event = FaultEvent()
                event.detected_stamp = float_to_ros_time(detected_time)
                event.recovered_stamp = float_to_ros_time(recovered_time)
                event.uav_id = peer_id
                event.fault_type = 'LINK_LOSS'
                self.fault_event_pub.publish(event)

    def _on_task_state(self, msg: UAVState):
        if msg.uav_id and msg.uav_id != self.uav_id:
            self.tracker.register_peer(msg.uav_id, self._now_sec())

    def _send_heartbeat(self):
        msg = SwarmMessage()
        msg.msg_id = f'hb_{self.uav_id}_{self._now_sec():.2f}'
        msg.src_uav_id = self.uav_id
        msg.dst_uav_id = ''  # broadcast
        msg.msg_type = 'HEARTBEAT'
        msg.payload_json = json.dumps({'uav_id': self.uav_id, 'stamp': self._now_sec()})
        msg.origin_stamp = self.get_clock().now().to_msg()
        msg.ttl = 3
        self.comm_tx_pub.publish(msg)

    def _check_timeouts(self):
        now_sec = self._now_sec()
        new_faults = self.tracker.check_timeouts(now_sec)
        for peer_id, detected_time in new_faults:
            self.get_logger().error(f'[{self.uav_id}] Detected FAULT: Peer {peer_id} timed out!')
            event = FaultEvent()
            event.detected_stamp = float_to_ros_time(detected_time)
            # recovered_stamp is zero/unset initially
            event.uav_id = peer_id
            event.fault_type = 'LINK_LOSS'
            self.fault_event_pub.publish(event)


def main(args=None):
    rclpy.init(args=args)
    node = FaultManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
