"""
battery_rth_node.py — Per-UAV battery monitoring and predictive RTH node.

Per ARCHITECTURE.md Section 6.5:
- Subscribes to /uavN/battery_state and /uavN/pose
- Serves /uavN/estimate_reachability for cbba_node
- Decides RTH based on predictive return energy vs critical battery threshold
- Publishes /uavN/rth_command and /uavN/cmd_vel_proposed (steering towards GCS)
- Calls /swarm/request_reserve_swap_in when RTH triggered
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from swarm_msgs.msg import UAVState, FaultEvent
from swarm_msgs.srv import EstimateReachability, RequestReserveSwapIn

from .rth_logic import is_reachable, should_trigger_rth, compute_distance_3d


class BatteryRTHNode(Node):
    """Per-UAV battery and RTH manager."""

    def __init__(self):
        super().__init__('battery_rth_node')

        self.declare_parameter('uav_id', '')
        self.declare_parameter('critical_frac', 0.30)
        self.declare_parameter('cruise_speed_mps', 10.0)
        self.declare_parameter('drain_rate_per_sec', 0.003)   # must match virtual_battery_node
        self.declare_parameter('gcs_x', 0.0)
        self.declare_parameter('gcs_y', 0.0)
        self.declare_parameter('gcs_z', 0.0)   # ground level — drone must descend and land

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        self.critical_frac = self.get_parameter('critical_frac').value
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.drain_rate = self.get_parameter('drain_rate_per_sec').value
        self.gcs_pos = (
            self.get_parameter('gcs_x').value,
            self.get_parameter('gcs_y').value,
            self.get_parameter('gcs_z').value,
        )

        self.current_pos = (0.0, 0.0, 0.0)
        self.battery_frac = 1.0
        self.rth_triggered = False
        self.swap_requested = False
        self.landing = False       # True once near GCS horizontally, descending
        self.grounded = False      # True once z < 1m, fully stopped
        self._log_tick = 0  # for periodic battery logging

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.create_subscription(Point, 'pose', self._on_pose, best_effort)
        self.create_subscription(UAVState, 'battery_state', self._on_battery_state, best_effort)

        # Publishers
        self.rth_pub = self.create_publisher(Bool, 'rth_command', qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_proposed', qos)
        self.fault_pub = self.create_publisher(FaultEvent, '/swarm/fault_events', qos)

        # Services & Clients
        self.reachability_srv = self.create_service(
            EstimateReachability, 'estimate_reachability', self._handle_estimate_reachability)
        self.swap_client = self.create_client(
            RequestReserveSwapIn, '/swarm/request_reserve_swap_in')

        # Timer (2 Hz)
        self.timer = self.create_timer(0.5, self._rth_check_tick)
        self.get_logger().info(f'Battery RTH node initialized for {self.uav_id}')

    def _on_pose(self, msg: Point):
        self.current_pos = (msg.x, msg.y, msg.z)

    def _on_battery_state(self, msg: UAVState):
        self.battery_frac = msg.battery_frac

    def _handle_estimate_reachability(self, request, response):
        target = (request.poi_position.x, request.poi_position.y, request.poi_position.z)
        response.reachable = is_reachable(
            current_pos=self.current_pos,
            target_pos=target,
            gcs_pos=self.gcs_pos,
            current_battery_frac=self.battery_frac,
            critical_frac=self.critical_frac,
            speed_mps=self.cruise_speed,
            drain_rate_per_sec=self.drain_rate,
        )
        return response

    def _rth_check_tick(self):
        self._log_tick += 1
        # Log battery level every 5s (10 ticks @ 2Hz) for evidence capture
        if self._log_tick % 10 == 0:
            dist_to_gcs = math.sqrt(
                (self.current_pos[0] - self.gcs_pos[0])**2 +
                (self.current_pos[1] - self.gcs_pos[1])**2 +
                (self.current_pos[2] - self.gcs_pos[2])**2)
            self.get_logger().info(
                f'[{self.uav_id}] BAT={self.battery_frac:.4f} '
                f'pos=({self.current_pos[0]:.1f},{self.current_pos[1]:.1f},{self.current_pos[2]:.1f}) '
                f'dist_to_gcs={dist_to_gcs:.1f}m rth={self.rth_triggered}')

        # Don't check RTH until drone is actually airborne (z > 1m)
        if self.current_pos[2] < 1.0:
            return

        if not self.rth_triggered:
            trigger = should_trigger_rth(
                current_pos=self.current_pos,
                gcs_pos=self.gcs_pos,
                current_battery_frac=self.battery_frac,
                critical_frac=self.critical_frac,
                speed_mps=self.cruise_speed,
                drain_rate_per_sec=self.drain_rate,
            )
            if trigger:
                self.rth_triggered = True
                self.get_logger().warn(
                    f'[{self.uav_id}] RTH TRIGGERED! Battery: {self.battery_frac:.3f}, Critical: {self.critical_frac}')
                b = Bool()
                b.data = True
                self.rth_pub.publish(b)

                # Report fault event
                fe = FaultEvent()
                fe.detected_stamp = self.get_clock().now().to_msg()
                fe.uav_id = self.uav_id
                fe.fault_type = 'BATTERY_CRITICAL'
                self.fault_pub.publish(fe)

                # Request reserve swap-in
                if self.swap_client.wait_for_service(timeout_sec=0.2):
                    req = RequestReserveSwapIn.Request()
                    req.departing_uav_id = self.uav_id
                    req.reason = 'RTH'
                    self.swap_client.call_async(req)
                    self.swap_requested = True

        if self.rth_triggered:
            # Phase 1: Fly toward GCS at cruise altitude until close horizontally
            dx = self.gcs_pos[0] - self.current_pos[0]
            dy = self.gcs_pos[1] - self.current_pos[1]
            horiz_dist = math.sqrt(dx*dx + dy*dy)

            cmd = Twist()

            if self.grounded:
                # Phase 3: On the ground — publish zero velocity, do nothing
                self.cmd_vel_pub.publish(cmd)
                return

            if horiz_dist > 5.0 and not self.landing:
                # Phase 1: cruise toward GCS at current altitude
                dz = 20.0 - self.current_pos[2]  # maintain 20m until close
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                scale = min(self.cruise_speed / max(dist, 0.1), 1.0)
                cmd.linear.x = dx * scale
                cmd.linear.y = dy * scale
                cmd.linear.z = dz * scale
                self.cmd_vel_pub.publish(cmd)
                if self._log_tick % 10 == 0:
                    self.get_logger().info(
                        f'[{self.uav_id}] RTH CRUISE: horiz_dist={horiz_dist:.1f}m '
                        f'pos=({self.current_pos[0]:.1f},{self.current_pos[1]:.1f},{self.current_pos[2]:.1f})')
            else:
                # Phase 2: Above GCS — descend to land
                if not self.landing:
                    self.landing = True
                    self.get_logger().warn(
                        f'[{self.uav_id}] RTH LANDING: Reached GCS area, descending...')

                if self.current_pos[2] < 1.0:
                    # Phase 3: Grounded
                    self.grounded = True
                    self.get_logger().warn(
                        f'[{self.uav_id}] RTH COMPLETE: Landed at '
                        f'({self.current_pos[0]:.1f},{self.current_pos[1]:.1f},{self.current_pos[2]:.1f})')
                    self.cmd_vel_pub.publish(cmd)  # zero velocity
                    return

                # Descend at -2 m/s, hold position horizontally
                cmd.linear.x = dx * 0.5  # gentle horizontal correction
                cmd.linear.y = dy * 0.5
                cmd.linear.z = -2.0       # descend steadily
                self.cmd_vel_pub.publish(cmd)
                if self._log_tick % 10 == 0:
                    self.get_logger().info(
                        f'[{self.uav_id}] RTH DESCENDING: z={self.current_pos[2]:.1f}m')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryRTHNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
