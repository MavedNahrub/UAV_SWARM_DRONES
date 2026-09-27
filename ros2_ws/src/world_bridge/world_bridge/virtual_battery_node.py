"""
virtual_battery_node.py — Per-UAV virtual battery simulator.

Per ARCHITECTURE.md Section 6.1:
- Subscribes to /uavN/pose and /uavN/task_state (role)
- Simulates linear-with-role-multiplier battery drain
- Publishes /uavN/battery_state (UAVState with battery_frac)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from swarm_msgs.msg import UAVState
from .battery_model import VirtualBatteryModel


class VirtualBatteryNode(Node):
    """Per-UAV virtual battery node."""

    def __init__(self):
        super().__init__('virtual_battery_node')

        self.declare_parameter('uav_id', '')
        self.declare_parameter('initial_battery_frac', 1.0)
        self.declare_parameter('base_drain_per_sec', 0.003)   # demo: RTH in ~120-150s airborne
        self.declare_parameter('update_hz', 1.0)

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        initial_battery = self.get_parameter('initial_battery_frac').value
        base_drain = self.get_parameter('base_drain_per_sec').value
        self.update_hz = self.get_parameter('update_hz').value

        self.model = VirtualBatteryModel(
            initial_frac=initial_battery,
            base_drain_per_sec=base_drain
        )
        self.current_role = UAVState.IDLE
        self.current_pos = Point()

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriptions
        self.create_subscription(Point, 'pose', self._on_pose, best_effort)
        self.create_subscription(UAVState, 'task_state', self._on_task_state, reliable)

        # Publisher
        self.battery_pub = self.create_publisher(UAVState, 'battery_state', best_effort)

        # Timer
        period = 1.0 / max(self.update_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f'Virtual battery node started for {self.uav_id}')


    def _on_pose(self, msg: Point):
        self.current_pos = msg

    def _on_task_state(self, msg: UAVState):
        self.current_role = msg.role

    def _tick(self):
        # Use fixed dt = timer period. This is robust against ROS clock jumps
        # that were causing instant battery drain (dt=30s on first tick).
        dt = 1.0 / max(self.update_hz, 0.1)

        b_frac = self.model.step(dt, self.current_role)

        msg = UAVState()
        msg.uav_id = self.uav_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.position = self.current_pos
        msg.battery_frac = b_frac
        msg.role = self.current_role
        msg.health = UAVState.NOMINAL if b_frac > 0.10 else UAVState.DEGRADED

        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
