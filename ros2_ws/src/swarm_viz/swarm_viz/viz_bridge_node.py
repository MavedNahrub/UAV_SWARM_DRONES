"""
viz_bridge_node.py — RViz2 visualization bridge publishing MarkerArray.

Publishes markers for:
- UAV positions, roles, and status
- Geofence polygon boundary
- Obstacles
- Active RF mesh links (colored by RSSI)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from swarm_msgs.msg import UAVState, LinkStateArray, Geofence, ObstacleArray


ROLE_COLORS = {
    UAVState.SURVEYOR: ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9),    # Green
    UAVState.RELAY: ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9),       # Blue
    UAVState.RTH: ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),         # Yellow
    UAVState.RESERVE: ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5),     # Gray
    UAVState.GROUNDED: ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.5),    # Dark Gray
    UAVState.IDLE: ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8),        # Light Gray
}


class VizBridgeNode(Node):
    """Bridges swarm state into RViz2 MarkerArray."""

    def __init__(self):
        super().__init__('viz_bridge_node')

        self.declare_parameter('uav_ids', 'uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
        self.declare_parameter('viz_rate_hz', 5.0)

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]
        rate_hz = self.get_parameter('viz_rate_hz').value

        self.positions = {}
        self.roles = {}
        self.batteries = {}
        self.link_states = []
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

        # UAV subscriptions
        for uid in self.uav_ids:
            self._setup_uav_subs(uid, best_effort, reliable)

        # Global swarm subscriptions
        self.create_subscription(LinkStateArray, '/swarm/link_states', self._on_link_states, reliable)
        self.create_subscription(Geofence, '/swarm/geofence', self._on_geofence, latched)
        self.create_subscription(ObstacleArray, '/swarm/obstacles', self._on_obstacles, latched)

        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/swarm/viz_markers', reliable)

        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_markers)
        self.get_logger().info('VizBridgeNode started')

    def _setup_uav_subs(self, uid: str, best_effort: QoSProfile, reliable: QoSProfile):
        def pose_cb(msg: Point, u=uid):
            self.positions[u] = (msg.x, msg.y, msg.z)

        def task_cb(msg: UAVState, u=uid):
            self.roles[u] = msg.role

        def battery_cb(msg: UAVState, u=uid):
            self.batteries[u] = msg.battery_frac

        self.create_subscription(Point, f'/{uid}/pose', pose_cb, best_effort)
        self.create_subscription(UAVState, f'/{uid}/task_state', task_cb, reliable)
        self.create_subscription(UAVState, f'/{uid}/battery_state', battery_cb, best_effort)

    def _on_link_states(self, msg: LinkStateArray):
        self.link_states = msg.links

    def _on_geofence(self, msg: Geofence):
        self.geofence_poly = msg.polygon

    def _on_obstacles(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def _publish_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        # 1. UAV markers (Spheres + Text)
        for uid, pos in self.positions.items():
            role = self.roles.get(uid, UAVState.IDLE)
            color = ROLE_COLORS.get(role, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8))

            # Vehicle body sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'uav_body'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.scale.x = 2.0
            m.scale.y = 2.0
            m.scale.z = 1.0
            m.color = color
            markers.markers.append(m)

            # Text label above UAV
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now
            t.ns = 'uav_label'
            t.id = marker_id
            marker_id += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = pos[0]
            t.pose.position.y = pos[1]
            t.pose.position.z = pos[2] + 2.5
            t.scale.z = 1.5
            batt = self.batteries.get(uid, 1.0) * 100.0
            t.text = f"{uid} ({batt:.0f}%)"
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            markers.markers.append(t)

        # 2. Geofence polygon boundary (Line strip)
        if len(self.geofence_poly) >= 3:
            gm = Marker()
            gm.header.frame_id = 'map'
            gm.header.stamp = now
            gm.ns = 'geofence'
            gm.id = marker_id
            marker_id += 1
            gm.type = Marker.LINE_STRIP
            gm.action = Marker.ADD
            gm.scale.x = 1.0
            gm.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.8)
            gm.points = list(self.geofence_poly) + [self.geofence_poly[0]]  # closed loop
            markers.markers.append(gm)

        # 3. Obstacles (Cylinders)
        for obs in self.obstacles:
            om = Marker()
            om.header.frame_id = 'map'
            om.header.stamp = now
            om.ns = 'obstacles'
            om.id = marker_id
            marker_id += 1
            om.type = Marker.CYLINDER
            om.action = Marker.ADD
            om.pose.position = obs.center
            om.scale.x = obs.radius_m * 2.0
            om.scale.y = obs.radius_m * 2.0
            om.scale.z = 40.0
            om.color = ColorRGBA(r=0.8, g=0.3, b=0.1, a=0.4)
            markers.markers.append(om)

        # 4. Mesh RF links (Lines)
        lm = Marker()
        lm.header.frame_id = 'map'
        lm.header.stamp = now
        lm.ns = 'mesh_links'
        lm.id = marker_id
        marker_id += 1
        lm.type = Marker.LINE_LIST
        lm.action = Marker.ADD
        lm.scale.x = 0.4
        lm.color = ColorRGBA(r=0.2, g=0.9, b=0.9, a=0.6)

        line_pts = []
        for l in self.link_states:
            if l.connected and l.uav_a in self.positions and l.uav_b in self.positions:
                pa = self.positions[l.uav_a]
                pb = self.positions[l.uav_b]
                line_pts.append(Point(x=pa[0], y=pa[1], z=pa[2]))
                line_pts.append(Point(x=pb[0], y=pb[1], z=pb[2]))
        lm.points = line_pts
        markers.markers.append(lm)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VizBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
