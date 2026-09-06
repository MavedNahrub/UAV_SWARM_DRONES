"""
collision_geofence_monitor_node.py — Independent ground-truth safety monitor.

Per ARCHITECTURE.md Section 6.1:
- Subscribes to all /uavN/pose, /swarm/geofence, /swarm/obstacles
- Computes pairwise separation and geofence violations for logging and verification
- Read-only observer (never modifies commands)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from swarm_msgs.msg import Geofence, ObstacleArray, CollisionEvent, GeofenceViolation
from .geometry_utils import point_in_polygon_2d, check_obstacle_collision, compute_pairwise_separation


class CollisionGeofenceMonitorNode(Node):
    """Monitors collisions and geofence breaches for evaluation and logging."""

    def __init__(self):
        super().__init__('collision_geofence_monitor_node')

        self.declare_parameter('uav_ids', 'uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
        self.declare_parameter('min_separation_m', 5.0)
        self.declare_parameter('check_rate_hz', 5.0)

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]
        self.min_sep = self.get_parameter('min_separation_m').value
        check_rate = self.get_parameter('check_rate_hz').value

        self.uav_positions = {}
        self.geofence_polygon = []
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

        # Pose subscribers for all UAVs
        for uid in self.uav_ids:
            self._create_pose_sub(uid, best_effort)

        # Geofence and obstacle subscribers
        self.create_subscription(Geofence, '/swarm/geofence', self._on_geofence, latched)
        self.create_subscription(ObstacleArray, '/swarm/obstacles', self._on_obstacles, latched)

        # Publishers for logger
        self.collision_pub = self.create_publisher(CollisionEvent, '/swarm/collision_events', reliable)
        self.geofence_viol_pub = self.create_publisher(GeofenceViolation, '/swarm/geofence_violations', reliable)

        self.timer = self.create_timer(1.0 / max(check_rate, 0.1), self._check_safety)
        self.get_logger().info('Collision and geofence monitor started')

    def _create_pose_sub(self, uav_id: str, qos: QoSProfile):
        def cb(msg: Point):
            self.uav_positions[uav_id] = (msg.x, msg.y, msg.z)
        self.create_subscription(Point, f'/{uav_id}/pose', cb, qos)

    def _on_geofence(self, msg: Geofence):
        self.geofence_polygon = [(p.x, p.y) for p in msg.polygon]

    def _on_obstacles(self, msg: ObstacleArray):
        self.obstacles = [
            ((obs.center.x, obs.center.y, obs.center.z), obs.radius_m)
            for obs in msg.obstacles
        ]

    def _check_safety(self):
        now = self.get_clock().now().to_msg()
        # 1. Pairwise separation check
        pairs = compute_pairwise_separation(self.uav_positions)
        for uav_a, uav_b, dist in pairs:
            if dist < self.min_sep:
                self.get_logger().warn(
                    f'COLLISION VIOLATION: {uav_a} and {uav_b} distance {dist:.2f}m < {self.min_sep:.2f}m')
                ev = CollisionEvent()
                ev.stamp = now
                ev.uav_a = uav_a
                ev.uav_b = uav_b
                ev.distance_m = float(dist)
                ev.min_separation_m = float(self.min_sep)
                self.collision_pub.publish(ev)

        # 2. Geofence check
        if len(self.geofence_polygon) >= 3:
            for uid, pos in self.uav_positions.items():
                if not point_in_polygon_2d((pos[0], pos[1]), self.geofence_polygon):
                    self.get_logger().warn(
                        f'GEOFENCE VIOLATION: {uid} at ({pos[0]:.1f}, {pos[1]:.1f}) is outside boundary!')
                    gv = GeofenceViolation()
                    gv.stamp = now
                    gv.uav_id = uid
                    gv.position = Point(x=pos[0], y=pos[1], z=pos[2])
                    gv.corrected_position = Point(x=pos[0], y=pos[1], z=pos[2])
                    self.geofence_viol_pub.publish(gv)

        # 3. Obstacle collision check
        for uid, pos in self.uav_positions.items():
            for obs_center, obs_radius in self.obstacles:
                if check_obstacle_collision(pos, obs_center, obs_radius):
                    self.get_logger().warn(
                        f'OBSTACLE COLLISION: {uid} at {pos} entered obstacle at {obs_center} (r={obs_radius}m)!')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGeofenceMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
