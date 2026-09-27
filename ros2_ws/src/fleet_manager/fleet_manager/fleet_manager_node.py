"""
fleet_manager_node.py — Central fleet management node for PUSHPAK UAV-X.

Per ARCHITECTURE.md Section 6.4:
- Holds fleet-level authority for reserve activation
- Subscribes to /swarm/fault_events
- Provides /swarm/request_reserve_swap_in service
- Activates reserve UAV when active surveyor/relay experiences failure or RTH
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from swarm_msgs.msg import FaultEvent, UAVState
from swarm_msgs.srv import RequestReserveSwapIn
from .fleet_pool import FleetPool


class FleetManagerNode(Node):
    """Central fleet manager node."""

    def __init__(self):
        super().__init__('fleet_manager_node')

        self.declare_parameter('uav_ids', 'uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
        self.declare_parameter('reserve_uav_ids', 'uav_04,uav_05')
        self.declare_parameter('scenario', '')

        uav_ids_str = self.get_parameter('uav_ids').get_parameter_value().string_value
        reserve_str = self.get_parameter('reserve_uav_ids').get_parameter_value().string_value

        all_ids = [x.strip() for x in uav_ids_str.split(',') if x.strip()]
        reserves = [x.strip() for x in reserve_str.split(',') if x.strip()]
        active = [x for x in all_ids if x not in reserves]

        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value
        if scenario_path and os.path.exists(scenario_path):
            try:
                import yaml
                with open(scenario_path, 'r') as f:
                    sdata = yaml.safe_load(f) or {}
                roles = sdata.get('uavs', {}).get('roles', {})
                if roles:
                    reserves = [uid for uid, role in roles.items() if role == 'RESERVE']
                    active = [uid for uid, role in roles.items() if role != 'RESERVE']
                    all_ids = list(roles.keys())
            except Exception as e:
                self.get_logger().warn(f'Could not parse scenario roles: {e}')

        self.pool = FleetPool(active_uav_ids=active, reserve_uav_ids=reserves)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=50)

        # Service
        self.srv = self.create_service(
            RequestReserveSwapIn,
            '/swarm/request_reserve_swap_in',
            self._handle_swap_in)

        # Subscriptions
        self.create_subscription(
            FaultEvent,
            '/swarm/fault_events',
            self._on_fault_event,
            qos)

        # Publishers to notify activated UAVs
        self.uav_state_pubs = {}
        for uid in all_ids:
            self.uav_state_pubs[uid] = self.create_publisher(
                UAVState, f'/{uid}/task_state', qos)

        self.get_logger().info(
            f'FleetManager initialized. Active: {self.pool.get_active()}, '
            f'Reserves: {self.pool.get_reserves()}')

    def _handle_swap_in(self, request, response):
        self.get_logger().warn(
            f'Reserve swap requested for {request.departing_uav_id}, reason: {request.reason}')
        accepted, activated_id = self.pool.request_swap_in(
            request.departing_uav_id, request.reason)

        response.accepted = accepted
        response.activated_uav_id = activated_id

        if accepted:
            self.get_logger().info(
                f'Reserve swap ACCEPTED: {activated_id} activated to replace {request.departing_uav_id}')
            self._notify_activation(activated_id)
        else:
            self.get_logger().warn('Reserve swap REJECTED: No reserve UAVs available.')

        return response

    def _on_fault_event(self, msg: FaultEvent):
        # Auto-swap on hard failure or critical fault
        if msg.fault_type in ('UAV_FAILURE', 'BATTERY_CRITICAL'):
            if msg.uav_id in self.pool.get_active():
                self.get_logger().error(
                    f'Auto-handling fault {msg.fault_type} on active UAV {msg.uav_id}')
                accepted, activated_id = self.pool.request_swap_in(
                    msg.uav_id, msg.fault_type)
                if accepted:
                    self.get_logger().info(
                        f'Auto-activated reserve {activated_id} for failed {msg.uav_id}')
                    self._notify_activation(activated_id)

    def _notify_activation(self, activated_id: str):
        if activated_id in self.uav_state_pubs:
            msg = UAVState()
            msg.uav_id = activated_id
            msg.stamp = self.get_clock().now().to_msg()
            msg.role = UAVState.SURVEYOR
            msg.health = UAVState.NOMINAL
            self.uav_state_pubs[activated_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
