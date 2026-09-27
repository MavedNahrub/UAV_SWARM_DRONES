"""
relay_node.py — Per-UAV relay placement ROS 2 node.

Per ARCHITECTURE.md Section 6.3:
- Subscribes to /uavN/comm_rx for RELAY_UPDATE messages
- Builds belief of current surveyor positions
- Runs greedy relay heuristic
- Publishes RELAY_UPDATE gossip on /uavN/comm_tx
- Publishes /uavN/cmd_vel_proposed if this UAV is a relay
- Publishes /swarm/relay_decisions (from canonical instance)
"""

import json
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from swarm_msgs.msg import UAVState, SwarmMessage, RelayDecision, LinkStateArray, FaultEvent

from .relay_heuristic import compute_relay_positions, check_gcs_connectivity

import networkx as nx


class RelayNode(Node):
    """Per-UAV relay placement node."""

    def __init__(self):
        super().__init__('relay_node')

        self.declare_parameter('uav_id', '')
        self.declare_parameter('relay_tick_hz', 1.0)
        self.declare_parameter('cruise_speed_mps', 10.0)
        self.declare_parameter('relay_altitude_m', 25.0)
        self.declare_parameter('scenario', '')

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        self.relay_tick_hz = self.get_parameter('relay_tick_hz').value
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.relay_altitude = self.get_parameter('relay_altitude_m').value
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value

        # State: believed positions of all UAVs
        self.uav_positions = {}
        self.uav_roles = {}
        self.failed_uavs = set()
        self.connectivity_graph = nx.Graph()
        self.my_relay_target = None  # position to fly to if we're a relay
        self.is_relay = False
        self.position_received = False

        if scenario_path and os.path.exists(scenario_path):
            try:
                with open(scenario_path, 'r') as f:
                    sdata = yaml.safe_load(f) or {}
                roles = sdata.get('uavs', {}).get('roles', {})
                role_map = {
                    'SURVEYOR': UAVState.SURVEYOR,
                    'RELAY': UAVState.RELAY,
                    'RESERVE': UAVState.RESERVE,
                    'GROUNDED': UAVState.GROUNDED,
                    'IDLE': UAVState.IDLE,
                }
                for uid, r in roles.items():
                    self.uav_roles[uid] = role_map.get(r, UAVState.SURVEYOR)
                    idx = int(uid.split('_')[1]) if '_' in uid and uid.split('_')[1].isdigit() else 0
                    self.uav_positions[uid] = (float(idx * 3.0), 0.0, 0.0)
            except Exception as e:
                self.get_logger().warn(f'Could not load scenario in relay_node: {e}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=50)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.create_subscription(SwarmMessage, 'comm_rx', self._on_comm_rx, qos)
        self.create_subscription(Point, 'pose', self._on_pose, best_effort)
        self.create_subscription(
            LinkStateArray, '/swarm/link_states', self._on_link_states, qos)
        self.create_subscription(
            UAVState, 'task_state', self._on_task_state, qos)
        self.create_subscription(
            FaultEvent, '/swarm/fault_events', self._on_fault_event, qos)

        # Publishers
        self.comm_tx_pub = self.create_publisher(SwarmMessage, 'comm_tx', qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_proposed', qos)
        self.relay_decision_pub = self.create_publisher(
            RelayDecision, '/swarm/relay_decisions', qos)

        # Timer
        self.timer = self.create_timer(1.0 / self.relay_tick_hz, self._relay_tick)
        self.get_logger().info(f'Relay node started for {self.uav_id}')

    def _on_pose(self, msg: Point):
        self.uav_positions[self.uav_id] = (msg.x, msg.y, msg.z)
        self.position_received = True

    def _on_task_state(self, msg: UAVState):
        if msg.uav_id in self.failed_uavs:
            return
        self.uav_roles[msg.uav_id] = msg.role

    def _on_fault_event(self, msg: FaultEvent):
        if msg.fault_type in ('UAV_FAILURE', 'BATTERY_CRITICAL'):
            self.failed_uavs.add(msg.uav_id)
            self.uav_roles[msg.uav_id] = UAVState.GROUNDED
            self.get_logger().warn(
                f'Relay node noted failure on {msg.uav_id}, role set to GROUNDED')

    def _on_link_states(self, msg: LinkStateArray):
        """Rebuild connectivity graph from link states."""
        self.connectivity_graph.clear()
        for ls in msg.links:
            self.connectivity_graph.add_node(ls.uav_a)
            self.connectivity_graph.add_node(ls.uav_b)
            if ls.connected:
                self.connectivity_graph.add_edge(
                    ls.uav_a, ls.uav_b, weight=ls.latency_s)

    def _on_comm_rx(self, msg: SwarmMessage):
        if msg.msg_type == 'RELAY_UPDATE':
            try:
                payload = json.loads(msg.payload_json)
                uid = payload.get('uav_id')
                pos = payload.get('position')
                role = payload.get('role')
                if uid and pos:
                    self.uav_positions[uid] = (pos['x'], pos['y'], pos['z'])
                if uid and role is not None and uid not in self.failed_uavs:
                    self.uav_roles[uid] = role
            except (json.JSONDecodeError, KeyError):
                pass

    def _relay_tick(self):
        """Periodic relay computation."""
        # Don't run relay computation until own position is valid
        if not self.position_received:
            return

        # Identify surveyors and available relays (excluding failed/grounded)
        surveyors = [uid for uid, role in self.uav_roles.items()
                     if role == UAVState.SURVEYOR and uid not in self.failed_uavs]
        relays = [uid for uid, role in self.uav_roles.items()
                  if role in (UAVState.RELAY, UAVState.IDLE, UAVState.RESERVE) and uid not in self.failed_uavs]

        # Dynamic fallback: if all dedicated relays are grounded, pick available reserve or idle UAV
        if not relays:
            relays = [uid for uid, role in self.uav_roles.items()
                      if uid not in self.failed_uavs and uid not in surveyors]

        if not surveyors:
            return

        gcs_pos = (0.0, 0.0, 0.0)

        assignments = compute_relay_positions(
            self.uav_positions, gcs_pos, surveyors, relays,
            self.connectivity_graph)

        # Check if this UAV is assigned as relay
        self.is_relay = False
        self.my_relay_target = None
        for a in assignments:
            if a['uav_id'] == self.uav_id:
                self.is_relay = True
                self.my_relay_target = a['target_position']
                break

        # Publish relay decision (canonical: lowest UAV ID does this)
        all_uav_ids = sorted(self.uav_positions.keys())
        if all_uav_ids and self.uav_id == all_uav_ids[0]:
            decision = RelayDecision()
            decision.stamp = self.get_clock().now().to_msg()
            decision.relay_uav_ids = [a['uav_id'] for a in assignments]
            all_conn, _ = check_gcs_connectivity(
                self.connectivity_graph, surveyors)
            decision.connectivity_ok = all_conn
            decision.reallocation_triggered = len(assignments) > 0
            decision.trigger_reason = 'PERIODIC'
            self.relay_decision_pub.publish(decision)

        # Gossip our position/role
        self._gossip_state()

        # Command synthesis — only publish cmd_vel_proposed if our actual role is RELAY
        # to prevent overriding CBBA's velocity commands for SURVEYOR UAVs
        my_actual_role = self.uav_roles.get(self.uav_id, UAVState.IDLE)
        if my_actual_role == UAVState.RELAY and self.is_relay and self.my_relay_target:
            self._fly_to_relay_position()
        elif my_actual_role == UAVState.RELAY and self.is_relay:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

    def _gossip_state(self):
        """Broadcast our position and role via comm_tx."""
        pos = self.uav_positions.get(self.uav_id, (0, 0, 0))
        payload = {
            'uav_id': self.uav_id,
            'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
            'role': self.uav_roles.get(self.uav_id, UAVState.IDLE),
        }
        msg = SwarmMessage()
        msg.src_uav_id = self.uav_id
        msg.dst_uav_id = ''  # broadcast
        msg.msg_type = 'RELAY_UPDATE'
        msg.payload_json = json.dumps(payload)
        msg.origin_stamp = self.get_clock().now().to_msg()
        msg.ttl = 5
        self.comm_tx_pub.publish(msg)

    def _fly_to_relay_position(self):
        """Generate velocity command toward relay waypoint."""
        target = self.my_relay_target
        pos = self.uav_positions.get(self.uav_id, (0, 0, 0))

        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dz = self.relay_altitude - pos[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        cmd = Twist()
        if dist > 2.0:
            scale = min(self.cruise_speed / max(dist, 0.1), 1.0)
            cmd.linear.x = dx * scale
            cmd.linear.y = dy * scale
            cmd.linear.z = dz * scale
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
