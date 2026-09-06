"""
cbba_node.py — Per-UAV CBBA task allocation ROS 2 node.

Per ARCHITECTURE.md Section 6.2:
- Subscribes: /uavN/comm_rx (TASK_BID, TASK_AWARD, TASK_RELEASE, POI_ALERT)
- Subscribes: /uavN/pose, /uavN/battery_state
- Publishes: /uavN/comm_tx (outgoing bids/awards)
- Publishes: /uavN/task_state (UAVState with assignment info)
- Publishes: /uavN/cmd_vel_proposed (velocity command when surveying)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from swarm_msgs.msg import UAVState, SwarmMessage

from .cbba import CBBAAgent, TaskInfo, BidInfo, PRIORITY_WEIGHTS


class CBBANode(Node):
    """Per-UAV CBBA task allocation node."""

    def __init__(self):
        super().__init__('cbba_node')

        # Parameters
        self.declare_parameter('uav_id', '')
        self.declare_parameter('bundle_capacity', 3)
        self.declare_parameter('cbba_tick_hz', 2.0)
        self.declare_parameter('cruise_speed_mps', 10.0)
        self.declare_parameter('arrival_threshold_m', 5.0)
        self.declare_parameter('survey_altitude_m', 20.0)

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        bundle_cap = self.get_parameter('bundle_capacity').value
        self.cbba_tick_hz = self.get_parameter('cbba_tick_hz').value
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.arrival_threshold = self.get_parameter('arrival_threshold_m').value
        self.survey_altitude = self.get_parameter('survey_altitude_m').value

        # CBBA agent logic
        self.agent = CBBAAgent(
            agent_id=self.uav_id,
            position=(0.0, 0.0, 0.0),
            bundle_capacity=bundle_cap,
            battery_frac=1.0,
            speed_mps=self.cruise_speed,
        )

        # State
        self.current_position = Point()
        self.battery_frac = 1.0
        self.role = UAVState.SURVEYOR
        self.cbba_round = 0
        self.surveying_task_id = None
        self.survey_timer_remaining = 0.0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            SwarmMessage, 'comm_rx', self._on_comm_rx, qos)
        self.create_subscription(
            Point, 'pose', self._on_pose, best_effort_qos)
        self.create_subscription(
            UAVState, 'battery_state', self._on_battery, best_effort_qos)

        # Publishers
        self.comm_tx_pub = self.create_publisher(SwarmMessage, 'comm_tx', qos)
        self.task_state_pub = self.create_publisher(UAVState, 'task_state', qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_proposed', qos)

        # CBBA tick timer
        period = 1.0 / self.cbba_tick_hz
        self.timer = self.create_timer(period, self._cbba_tick)

        self.get_logger().info(f'CBBA node started for {self.uav_id}')

    def _on_pose(self, msg: Point):
        self.current_position = msg
        self.agent.update_position((msg.x, msg.y, msg.z))

    def _on_battery(self, msg: UAVState):
        self.battery_frac = msg.battery_frac
        self.agent.update_battery(msg.battery_frac)

    def _on_comm_rx(self, msg: SwarmMessage):
        """Process incoming messages via comm_gateway."""
        if msg.msg_type == 'TASK_BID':
            self._handle_bid(msg)
        elif msg.msg_type == 'TASK_AWARD':
            self._handle_award(msg)
        elif msg.msg_type == 'TASK_RELEASE':
            self._handle_release(msg)
        elif msg.msg_type == 'POI_ALERT':
            self._handle_poi_alert(msg)

    def _handle_bid(self, msg: SwarmMessage):
        """Process an incoming TASK_BID from another agent."""
        try:
            payload = json.loads(msg.payload_json)
        except json.JSONDecodeError:
            return

        bids = []
        for bid_data in payload.get('bids', []):
            bids.append(BidInfo(
                agent_id=bid_data['agent_id'],
                task_id=bid_data['task_id'],
                bid_value=bid_data['bid_value'],
                timestamp=bid_data['timestamp'],
            ))

        changed = self.agent.consensus_phase(bids)
        if changed:
            # Re-run bundle phase and broadcast updated bids
            self._run_bundle_and_broadcast()

    def _handle_award(self, msg: SwarmMessage):
        """Handle a task award message."""
        # Awards are handled implicitly through the bid consensus
        pass

    def _handle_release(self, msg: SwarmMessage):
        """Handle a task release (agent leaving, fault, RTH)."""
        try:
            payload = json.loads(msg.payload_json)
        except json.JSONDecodeError:
            return

        task_id = payload.get('task_id', '')
        if task_id and task_id in self.agent.winning_bids:
            # Clear the winning bid so we can re-bid
            del self.agent.winning_bids[task_id]

    def _handle_poi_alert(self, msg: SwarmMessage):
        """Handle a new high-priority PoI injected mid-mission."""
        try:
            payload = json.loads(msg.payload_json)
        except json.JSONDecodeError:
            return

        task = TaskInfo(
            task_id=payload['poi_id'],
            position=(
                payload['position']['x'],
                payload['position']['y'],
                payload['position']['z'],
            ),
            priority=payload.get('priority', 3),
            survey_duration_s=payload.get('survey_duration_s', 15.0),
        )

        current_time = self.get_clock().now().nanoseconds / 1e9
        new_bids = self.agent.handle_priority_interrupt(task, current_time)

        if new_bids:
            self._broadcast_bids(new_bids)
            self.get_logger().info(
                f'Priority interrupt: bidding on {task.task_id} '
                f'(priority={task.priority})'
            )

    def _cbba_tick(self):
        """Periodic CBBA round + command synthesis."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Run CBBA bundle phase
        new_bids = self.agent.bundle_phase(current_time)
        if new_bids:
            self._broadcast_bids(new_bids)

        self.cbba_round += 1

        # Publish task state
        self._publish_task_state()

        # Command synthesis: generate velocity toward current target
        if self.role == UAVState.SURVEYOR:
            self._command_synthesis()

    def _broadcast_bids(self, bids: list):
        """Broadcast bids to all other agents via comm_tx."""
        payload = {
            'bids': [
                {
                    'agent_id': b.agent_id,
                    'task_id': b.task_id,
                    'bid_value': b.bid_value,
                    'timestamp': b.timestamp,
                }
                for b in bids
            ]
        }

        # Broadcast to all (dst_uav_id empty = broadcast behavior
        # handled by app convention — each agent processes all bids)
        msg = SwarmMessage()
        msg.src_uav_id = self.uav_id
        msg.dst_uav_id = ''  # broadcast
        msg.msg_type = 'TASK_BID'
        msg.payload_json = json.dumps(payload)
        msg.origin_stamp = self.get_clock().now().to_msg()
        msg.hop_count = 0
        msg.ttl = 10

        self.comm_tx_pub.publish(msg)

    def _run_bundle_and_broadcast(self):
        """Re-run bundle phase and broadcast any new bids."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        new_bids = self.agent.bundle_phase(current_time)
        if new_bids:
            self._broadcast_bids(new_bids)

    def _publish_task_state(self):
        """Publish current task assignment state."""
        state = UAVState()
        state.uav_id = self.uav_id
        state.stamp = self.get_clock().now().to_msg()
        state.position = self.current_position
        state.battery_frac = self.battery_frac
        state.role = self.role
        state.health = UAVState.NOMINAL

        target = self.agent.get_current_target()
        state.assigned_poi_id = target if target else ''
        state.bundle = self.agent.get_assigned_tasks()

        self.task_state_pub.publish(state)

    def _command_synthesis(self):
        """
        Generate velocity command toward current target PoI.

        Per ARCHITECTURE.md Section 6.2.1: publishes cmd_vel_proposed
        for the Safety Supervisor to consume.
        """
        target_id = self.agent.get_current_target()
        if not target_id or target_id not in self.agent.tasks:
            # No target — hover
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return

        task = self.agent.tasks[target_id]
        target_pos = task.position

        # Compute direction vector
        dx = target_pos[0] - self.current_position.x
        dy = target_pos[1] - self.current_position.y
        dz = self.survey_altitude - self.current_position.z

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        cmd = Twist()
        if dist > self.arrival_threshold:
            # Fly toward target at cruise speed
            scale = min(self.cruise_speed / max(dist, 0.1), 1.0)
            cmd.linear.x = dx * scale
            cmd.linear.y = dy * scale
            cmd.linear.z = dz * scale
        else:
            # At target — hover (surveying)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

            # Track survey progress
            if self.surveying_task_id != target_id:
                self.surveying_task_id = target_id
                self.survey_timer_remaining = task.survey_duration_s
                self.get_logger().info(f'Surveying {target_id}...')

            self.survey_timer_remaining -= (1.0 / self.cbba_tick_hz)
            if self.survey_timer_remaining <= 0:
                # Survey complete
                self.get_logger().info(f'Survey complete: {target_id}')
                task.status = 3  # SURVEYED
                self.agent.release_task(
                    target_id,
                    self.get_clock().now().nanoseconds / 1e9
                )
                self.surveying_task_id = None

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CBBANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
