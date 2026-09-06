"""
safety_supervisor_node.py — Central safety supervisor node.

Per ARCHITECTURE.md Section 6.6:
- Subscribes to every /uavN/cmd_vel_proposed, /uavN/pose, /swarm/geofence, /swarm/obstacles
- Operates at 20 Hz
- Sole publisher to /uavN/cmd_vel
- Enforces geofence boundaries, inter-UAV minimum separation, and obstacle avoidance via APF
- Emits events to /swarm/collision_events and /swarm/geofence_violations
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from swarm_msgs.msg import Geofence, ObstacleArray, CollisionEvent, GeofenceViolation

from .apf_safety import compute_apf_safe_velocity


class SafetySupervisorNode(Node):
    """Central safety supervisor and velocity arbiter."""

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.declare_parameter('uav_ids', 'uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
        self.declare_parameter('min_separation_m', 8.0)
        self.declare_parameter('obstacle_safety_margin_m', 5.0)
        self.declare_parameter('max_speed_mps', 12.0)
        self.declare_parameter('supervisor_rate_hz', 20.0)

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]
        self.min_sep = self.get_parameter('min_separation_m').value
        self.obs_margin = self.get_parameter('obstacle_safety_margin_m').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        rate_hz = self.get_parameter('supervisor_rate_hz').value

        self.positions = {}
        self.proposed_vels = {}
        self.geofence_poly = []
        self.obstacles = []

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscribers and Publishers per UAV
        self.cmd_vel_pubs = {}
        for uid in self.uav_ids:
            self._setup_uav_topics(uid, best_effort, reliable)

        # Geofence & obstacle subscribers
        self.create_subscription(Geofence, '/swarm/geofence', self._on_geofence, latched)
        self.create_subscription(ObstacleArray, '/swarm/obstacles', self._on_obstacles, latched)

        # Event publishers
        self.collision_pub = self.create_publisher(CollisionEvent, '/swarm/collision_events', reliable)
        self.geofence_viol_pub = self.create_publisher(GeofenceViolation, '/swarm/geofence_violations', reliable)

        # High-rate safety enforcement loop (20 Hz)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._enforce_safety_loop)
        self.get_logger().info(f'SafetySupervisor initialized at {rate_hz} Hz for UAVs: {self.uav_ids}')

    def _setup_uav_topics(self, uid: str, best_effort: QoSProfile, reliable: QoSProfile):
        self.positions[uid] = (0.0, 0.0, 0.0)
        self.proposed_vels[uid] = (0.0, 0.0, 0.0)

        def pose_cb(msg: Point, u=uid):
            self.positions[u] = (msg.x, msg.y, msg.z)

        def vel_cb(msg: Twist, u=uid):
            self.proposed_vels[u] = (msg.linear.x, msg.linear.y, msg.linear.z)

        self.create_subscription(Point, f'/{uid}/pose', pose_cb, best_effort)
        self.create_subscription(Twist, f'/{uid}/cmd_vel_proposed', vel_cb, reliable)
        self.cmd_vel_pubs[uid] = self.create_publisher(Twist, f'/{uid}/cmd_vel', reliable)

    def _on_geofence(self, msg: Geofence):
        self.geofence_poly = [(p.x, p.y) for p in msg.polygon]

    def _on_obstacles(self, msg: ObstacleArray):
        self.obstacles = [
            ((obs.center.x, obs.center.y, obs.center.z), obs.radius_m)
            for obs in msg.obstacles
        ]

    def _enforce_safety_loop(self):
        now = self.get_clock().now().to_msg()

        for uid in self.uav_ids:
            cur_pos = self.positions.get(uid, (0.0, 0.0, 0.0))
            prop_vel = self.proposed_vels.get(uid, (0.0, 0.0, 0.0))

            safe_vel, violations = compute_apf_safe_velocity(
                uav_id=uid,
                proposed_vel=prop_vel,
                current_pos=cur_pos,
                peer_positions=self.positions,
                obstacles=self.obstacles,
                geofence_polygon=self.geofence_poly,
                min_separation_m=self.min_sep,
                obstacle_safety_margin_m=self.obs_margin,
                max_speed_mps=self.max_speed
            )

            # Publish safe command
            cmd = Twist()
            cmd.linear.x = safe_vel[0]
            cmd.linear.y = safe_vel[1]
            cmd.linear.z = safe_vel[2]
            self.cmd_vel_pubs[uid].publish(cmd)

            # Process violations for logging
            for v in violations:
                if v.startswith("PEER_PROXIMITY"):
                    parts = v.split(':')
                    peer_id = parts[1]
                    dist = float(parts[2])
                    ev = CollisionEvent()
                    ev.stamp = now
                    ev.uav_a = min(uid, peer_id)
                    ev.uav_b = max(uid, peer_id)
                    ev.distance_m = dist
                    ev.min_separation_m = self.min_sep
                    self.collision_pub.publish(ev)
                elif v.startswith("GEOFENCE"):
                    gv = GeofenceViolation()
                    gv.stamp = now
                    gv.uav_id = uid
                    gv.position = Point(x=cur_pos[0], y=cur_pos[1], z=cur_pos[2])
                    gv.corrected_position = Point(
                        x=cur_pos[0] + safe_vel[0] * 0.05,
                        y=cur_pos[1] + safe_vel[1] * 0.05,
                        z=cur_pos[2] + safe_vel[2] * 0.05
                    )
                    self.geofence_viol_pub.publish(gv)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
