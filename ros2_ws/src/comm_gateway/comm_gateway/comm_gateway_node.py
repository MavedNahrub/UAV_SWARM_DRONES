"""
comm_gateway_node.py — Central degraded-mesh network simulator.

Per ARCHITECTURE.md Section 5:
- Subscribes to all /uavN/pose for pairwise distance
- Computes LinkState via log-distance path-loss model
- Gates SwarmMessage delivery with per-hop latency, packet loss, TTL
- Publishes LinkStateArray on /swarm/link_states
- Routes messages from /uavN/comm_tx to /uavN/comm_rx through the mesh

This node is the simulated RF channel — it never makes decisions,
only computes physical link quality and forwards opaque envelopes.
"""

import json
import uuid
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import networkx as nx
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from swarm_msgs.msg import LinkState, LinkStateArray, SwarmMessage

from .path_loss import LinkParams, compute_link_quality


@dataclass
class InFlightMessage:
    """A SwarmMessage being routed hop-by-hop through the mesh."""
    msg: SwarmMessage
    current_node: str       # which UAV/GCS the message is currently at
    next_hop: Optional[str] = None
    remaining_latency_s: float = 0.0
    delivered: bool = False
    dropped: bool = False


class CommGatewayNode(Node):
    """
    Central comm gateway — simulates the degraded RF mesh network.

    Per ARCHITECTURE.md Section 5.1:
    - Only subscriber to every /uavN/comm_tx
    - Only publisher to every /uavN/comm_rx
    - Publishes /swarm/link_states at gateway_tick_hz
    """

    def __init__(self):
        super().__init__('comm_gateway_node')

        # Parameters
        self.declare_parameter('uav_ids', '')
        self.declare_parameter('gateway_tick_hz', 10.0)
        self.declare_parameter('tx_power_dbm', 20.0)
        self.declare_parameter('path_loss_exponent', 3.0)
        self.declare_parameter('pl0_dbm', 40.0)
        self.declare_parameter('d0_m', 1.0)
        self.declare_parameter('shadowing_std_db', 2.0)
        self.declare_parameter('rssi_threshold_dbm', -85.0)
        self.declare_parameter('max_range_m', 500.0)
        self.declare_parameter('base_latency_s', 0.01)
        self.declare_parameter('max_ttl', 10)

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        self.uav_ids = [uid.strip() for uid in uav_ids_str.split(',') if uid.strip()]
        self.gateway_tick_hz = self.get_parameter('gateway_tick_hz').value
        self.max_ttl = self.get_parameter('max_ttl').value

        # Build link params from ROS parameters
        self.link_params = LinkParams(
            tx_power_dbm=self.get_parameter('tx_power_dbm').value,
            path_loss_exponent=self.get_parameter('path_loss_exponent').value,
            pl0_dbm=self.get_parameter('pl0_dbm').value,
            d0_m=self.get_parameter('d0_m').value,
            shadowing_std_db=self.get_parameter('shadowing_std_db').value,
            rssi_threshold_dbm=self.get_parameter('rssi_threshold_dbm').value,
            max_range_m=self.get_parameter('max_range_m').value,
            base_latency_s=self.get_parameter('base_latency_s').value,
        )

        # QoS profiles
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # State
        self.uav_positions: Dict[str, Point] = {}
        self.in_flight_messages: List[InFlightMessage] = []
        self.connectivity_graph = nx.Graph()

        # GCS is at origin (position set from scenario, default 0,0,0)
        self.gcs_position = Point(x=0.0, y=0.0, z=0.0)

        # Per-UAV subscribers and publishers
        self.pose_subs = {}
        self.comm_tx_subs = {}
        self.comm_rx_pubs = {}

        for uav_id in self.uav_ids:
            # Subscribe to /uavN/pose (bridged from PX4 local position)
            self.pose_subs[uav_id] = self.create_subscription(
                Point,
                f'/{uav_id}/pose',
                lambda msg, uid=uav_id: self._on_pose(uid, msg),
                self.best_effort_qos,
            )

            # Subscribe to /uavN/comm_tx (outgoing messages from app nodes)
            self.comm_tx_subs[uav_id] = self.create_subscription(
                SwarmMessage,
                f'/{uav_id}/comm_tx',
                lambda msg, uid=uav_id: self._on_comm_tx(uid, msg),
                self.reliable_qos,
            )

            # Publisher for /uavN/comm_rx (incoming messages to app nodes)
            self.comm_rx_pubs[uav_id] = self.create_publisher(
                SwarmMessage,
                f'/{uav_id}/comm_rx',
                self.reliable_qos,
            )

        # Link state publisher
        self.link_state_pub = self.create_publisher(
            LinkStateArray,
            '/swarm/link_states',
            self.reliable_qos,
        )

        # Gateway tick timer
        period = 1.0 / self.gateway_tick_hz
        self.timer = self.create_timer(period, self._gateway_tick)

        self.get_logger().info(
            f'CommGateway started: {len(self.uav_ids)} UAVs, '
            f'tick={self.gateway_tick_hz}Hz, '
            f'range={self.link_params.max_range_m}m'
        )

    def _on_pose(self, uav_id: str, msg: Point):
        """Update a UAV's known position."""
        self.uav_positions[uav_id] = msg

    def _on_comm_tx(self, src_uav_id: str, msg: SwarmMessage):
        """
        Receive a SwarmMessage from a UAV's app nodes.
        Queue it for hop-by-hop routing through the mesh.
        """
        # Assign a message ID if not set
        if not msg.msg_id:
            msg.msg_id = str(uuid.uuid4())[:8]

        # Set TTL if not set
        if msg.ttl == 0:
            msg.ttl = self.max_ttl

        in_flight = InFlightMessage(
            msg=msg,
            current_node=src_uav_id,
        )
        self.in_flight_messages.append(in_flight)

        self.get_logger().debug(
            f'Queued msg {msg.msg_id}: {src_uav_id} → {msg.dst_uav_id} '
            f'type={msg.msg_type}'
        )

    def _compute_distance(self, pos_a: Point, pos_b: Point) -> float:
        """Euclidean distance between two 3D points."""
        dx = pos_a.x - pos_b.x
        dy = pos_a.y - pos_b.y
        dz = pos_a.z - pos_b.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _gateway_tick(self):
        """
        Main gateway tick — per ARCHITECTURE.md Section 5.1:
        1. Compute pairwise LinkState
        2. Build connectivity graph
        3. Route in-flight messages one hop
        """
        # Need at least some positions to do anything
        if len(self.uav_positions) < 1:
            return

        # === Step 1: Compute pairwise link states ===
        link_states = []
        all_nodes = list(self.uav_ids) + ['GCS']
        node_positions = dict(self.uav_positions)
        node_positions['GCS'] = self.gcs_position

        self.connectivity_graph.clear()
        for node_id in all_nodes:
            if node_id in node_positions:
                self.connectivity_graph.add_node(node_id)

        connected_count = 0
        total_pairs = 0

        for i, id_a in enumerate(all_nodes):
            for j in range(i + 1, len(all_nodes)):
                id_b = all_nodes[j]

                if id_a not in node_positions or id_b not in node_positions:
                    continue

                # Canonical ordering: a < b lexicographically
                if id_a > id_b:
                    id_a, id_b = id_b, id_a

                distance = self._compute_distance(
                    node_positions[id_a], node_positions[id_b]
                )

                quality = compute_link_quality(distance, self.link_params)

                ls = LinkState()
                ls.uav_a = id_a
                ls.uav_b = id_b
                ls.distance_m = quality['distance_m']
                ls.rssi_dbm = quality['rssi_dbm']
                ls.connected = quality['connected']
                ls.packet_loss_prob = quality['packet_loss_prob']
                ls.latency_s = quality['latency_s']
                ls.is_relay_edge = False  # updated by relay_placement

                link_states.append(ls)
                total_pairs += 1

                if quality['connected']:
                    connected_count += 1
                    self.connectivity_graph.add_edge(
                        id_a, id_b,
                        weight=quality['latency_s'],
                        packet_loss=quality['packet_loss_prob'],
                    )

        # === Publish LinkStateArray ===
        lsa = LinkStateArray()
        lsa.stamp = self.get_clock().now().to_msg()
        lsa.links = link_states
        lsa.connectivity_fraction = (
            connected_count / total_pairs if total_pairs > 0 else 0.0
        )
        self.link_state_pub.publish(lsa)

        # === Step 2 & 3: Route in-flight messages ===
        self._route_messages()

    def _route_messages(self):
        """
        Advance all in-flight messages one hop toward their destination.

        Per ARCHITECTURE.md Section 5.1 step 4:
        - Find shortest path (Dijkstra by latency_s)
        - Draw against packet_loss_prob for the hop
        - Decrement TTL
        - On reaching dst, publish to /uavN/comm_rx
        """
        import random

        still_in_flight = []

        for ifm in self.in_flight_messages:
            if ifm.delivered or ifm.dropped:
                continue

            # Decrement remaining latency from previous tick
            if ifm.remaining_latency_s > 0:
                ifm.remaining_latency_s -= (1.0 / self.gateway_tick_hz)
                if ifm.remaining_latency_s > 0:
                    still_in_flight.append(ifm)
                    continue

            dst = ifm.msg.dst_uav_id
            current = ifm.current_node

            # Check if delivered
            if current == dst:
                if dst in self.comm_rx_pubs:
                    self.comm_rx_pubs[dst].publish(ifm.msg)
                    self.get_logger().debug(
                        f'Delivered msg {ifm.msg.msg_id} to {dst}'
                    )
                ifm.delivered = True
                continue

            # Check TTL
            if ifm.msg.ttl <= 0:
                self.get_logger().debug(
                    f'Dropped msg {ifm.msg.msg_id}: TTL expired at {current}'
                )
                ifm.dropped = True
                continue

            # Find next hop via shortest path
            if (current not in self.connectivity_graph or
                    dst not in self.connectivity_graph):
                # Source or destination not in graph — can't route
                still_in_flight.append(ifm)
                continue

            try:
                path = nx.shortest_path(
                    self.connectivity_graph, current, dst, weight='weight'
                )
            except nx.NetworkXNoPath:
                # No path — keep waiting (topology may change)
                still_in_flight.append(ifm)
                continue

            if len(path) < 2:
                ifm.delivered = True
                if dst in self.comm_rx_pubs:
                    self.comm_rx_pubs[dst].publish(ifm.msg)
                continue

            next_hop = path[1]

            # Get edge data for packet loss check
            edge_data = self.connectivity_graph.get_edge_data(current, next_hop)
            if edge_data is None:
                still_in_flight.append(ifm)
                continue

            # Draw against packet loss probability
            pkt_loss = edge_data.get('packet_loss', 0.0)
            if random.random() < pkt_loss:
                self.get_logger().debug(
                    f'Dropped msg {ifm.msg.msg_id}: packet loss on '
                    f'{current} → {next_hop}'
                )
                ifm.dropped = True
                continue

            # Advance one hop
            ifm.current_node = next_hop
            ifm.msg.hop_count += 1
            ifm.msg.ttl -= 1
            ifm.remaining_latency_s = edge_data.get('weight', self.link_params.base_latency_s)

            still_in_flight.append(ifm)

        self.in_flight_messages = still_in_flight


def main(args=None):
    rclpy.init(args=args)
    node = CommGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
