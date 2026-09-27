"""
geofence_obstacle_publisher_node.py — Publishes geofence and static obstacles.

Per ARCHITECTURE.md Section 6.1:
- Reads scenario YAML
- Publishes Geofence and ObstacleArray on latched (/swarm/geofence, /swarm/obstacles)
- Provides GetGeofence and GetObstacles services
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from swarm_msgs.msg import Geofence, Obstacle, ObstacleArray
from swarm_msgs.srv import GetGeofence, GetObstacles


class GeofenceObstaclePublisherNode(Node):
    """Central node publishing geofence and obstacle ground truth."""

    def __init__(self):
        super().__init__('geofence_obstacle_publisher_node')

        self.declare_parameter('scenario', '')
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value

        self.geofence_msg = Geofence()
        self.obstacle_array_msg = ObstacleArray()

        self._load_scenario(scenario_path)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Publishers
        self.geofence_pub = self.create_publisher(Geofence, '/swarm/geofence', latched_qos)
        self.obstacles_pub = self.create_publisher(ObstacleArray, '/swarm/obstacles', latched_qos)

        # Services
        self.get_geofence_srv = self.create_service(
            GetGeofence, '/swarm/get_geofence', self._handle_get_geofence)
        self.get_obstacles_srv = self.create_service(
            GetObstacles, '/swarm/get_obstacles', self._handle_get_obstacles)

        # Timer to latch/re-publish at 1 Hz
        self.timer = self.create_timer(1.0, self._publish_all)
        self._publish_all()

        self.get_logger().info(
            f'GeofenceObstaclePublisher loaded {len(self.geofence_msg.polygon)} vertices, '
            f'{len(self.obstacle_array_msg.obstacles)} obstacles')

    def _load_scenario(self, path: str):
        # Default fallback values if no file or missing sections
        default_polygon = [
            Point(x=-200.0, y=-200.0, z=0.0),
            Point(x=200.0, y=-200.0, z=0.0),
            Point(x=200.0, y=200.0, z=0.0),
            Point(x=-200.0, y=200.0, z=0.0),
        ]
        self.geofence_msg.polygon = default_polygon

        if path and os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}

                def _parse_pt(p):
                    if isinstance(p, dict):
                        return Point(x=float(p.get('x', 0.0)), y=float(p.get('y', 0.0)), z=float(p.get('z', 0.0)))
                    elif isinstance(p, (list, tuple)):
                        z = float(p[2]) if len(p) > 2 else 0.0
                        return Point(x=float(p[0]), y=float(p[1]), z=z)
                    return Point(x=0.0, y=0.0, z=0.0)

                geo_data = data.get('geofence', {}).get('polygon', [])
                if geo_data:
                    self.geofence_msg.polygon = [_parse_pt(p) for p in geo_data]

                obs_data = data.get('obstacles', [])
                obs_list = []
                for o in obs_data:
                    obs = Obstacle()
                    obs.center = _parse_pt(o.get('center', {}))
                    obs.radius_m = float(o.get('radius_m', o.get('radius', 10.0)))
                    obs_list.append(obs)
                self.obstacle_array_msg.obstacles = obs_list
            except Exception as e:
                self.get_logger().error(f'Error reading scenario YAML {path}: {e}')

    def _publish_all(self):
        self.geofence_pub.publish(self.geofence_msg)
        self.obstacles_pub.publish(self.obstacle_array_msg)

    def _handle_get_geofence(self, request, response):
        response.geofence = self.geofence_msg
        return response

    def _handle_get_obstacles(self, request, response):
        response.obstacles = self.obstacle_array_msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceObstaclePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
