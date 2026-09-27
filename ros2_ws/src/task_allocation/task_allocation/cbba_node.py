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
import os
import yaml

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
        self.declare_parameter('scenario', '')
        self.declare_parameter('initial_role', 'SURVEYOR')

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        bundle_cap = self.get_parameter('bundle_capacity').value
        self.cbba_tick_hz = self.get_parameter('cbba_tick_hz').value
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.arrival_threshold = self.get_parameter('arrival_threshold_m').value
        self.survey_altitude = self.get_parameter('survey_altitude_m').value
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value
        role_str = self.get_parameter('initial_role').get_parameter_value().string_value

        role_map = {
            'SURVEYOR': UAVState.SURVEYOR,
            'RELAY': UAVState.RELAY,
            'RESERVE': UAVState.RESERVE,
            'GROUNDED': UAVState.GROUNDED,
            'IDLE': UAVState.IDLE,
        }

        idx = int(self.uav_id.split('_')[1]) if '_' in self.uav_id and self.uav_id.split('_')[1].isdigit() else 0
        spawn_pos = (float(idx * 3.0), 0.0, 0.0)

        # CBBA agent logic
        self.agent = CBBAAgent(
            agent_id=self.uav_id,
            position=spawn_pos,
            bundle_capacity=bundle_cap,
            battery_frac=1.0,
            speed_mps=self.cruise_speed,
        )

        self.scenario_path = scenario_path
        self.scenario_pois_data = []  # stored for mission replay

        if scenario_path and os.path.exists(scenario_path):
            try:
                with open(scenario_path, 'r') as f:
                    sdata = yaml.safe_load(f) or {}
                roles = sdata.get('uavs', {}).get('roles', {})
                if self.uav_id in roles:
                    role_str = roles[self.uav_id]

                for pdata in sdata.get('pois', []):
                    pos = pdata.get('position', {})
                    task = TaskInfo(
                        task_id=pdata['poi_id'],
                        position=(pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0)),
                        priority=pdata.get('priority', 1),
                        survey_duration_s=pdata.get('survey_duration_s', 20.0),
                    )
                    self.agent.tasks[task.task_id] = task
                    self.scenario_pois_data.append(pdata)
            except Exception as e:
                self.get_logger().warn(f'Could not load scenario in cbba_node: {e}')

        self.role = role_map.get(role_str, UAVState.SURVEYOR)

        # State
        self.current_position = Point(x=spawn_pos[0], y=spawn_pos[1], z=spawn_pos[2])
        self.battery_frac = 1.0
        self.cbba_round = 0
        self.surveying_task_id = None
        self.survey_timer_remaining = 0.0
        self.position_received = False

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
        self.create_subscription(
            UAVState, 'task_state', self._on_task_state, qos)

        # RTH command — when True, stop all task execution
        from std_msgs.msg import Bool
        self.create_subscription(
            Bool, 'rth_command', self._on_rth_command, qos)
        self.rth_active = False

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
        self.position_received = True

    def _on_rth_command(self, msg):
        """When RTH fires, stop all task execution and release held tasks."""
        if msg.data and not self.rth_active:
            self.rth_active = True
            self.get_logger().warn(
                f'[{self.uav_id}] CBBA: RTH active — releasing all tasks, yielding velocity channel')
            # Release current tasks so other UAVs can rebid
            self.agent.release_all_tasks()

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
        elif msg.msg_type == 'POI_REPORT':
            self._handle_poi_report(msg)

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
        if changed and self.role == UAVState.SURVEYOR and self.position_received:
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
            priority=payload.get('priority', 1),
            survey_duration_s=payload.get('survey_duration_s', 20.0),
        )

        # Skip if this exact task already exists (prevents double-loading from
        # scenario YAML + orchestrator distribution from disrupting convergence)
        existing = self.agent.tasks.get(task.task_id)
        if existing and existing.position == task.position and existing.priority == task.priority:
            return

        self.agent.tasks[task.task_id] = task

        # Don't bid until PX4 is ready — tasks are stored, bidding happens
        # in the next _cbba_tick after position_received becomes True
        if not self.position_received:
            return

        if self.role == UAVState.SURVEYOR:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if task.priority >= 3:
                new_bids = self.agent.handle_priority_interrupt(task, current_time)
                if new_bids:
                    self._broadcast_bids(new_bids)
                    self.get_logger().info(
                        f'Priority interrupt: bidding on {task.task_id} (priority={task.priority})'
                    )
            else:
                new_bids = self.agent.bundle_phase(current_time)
                if new_bids:
                    self._broadcast_bids(new_bids)

    def _handle_poi_report(self, msg: SwarmMessage):
        """Handle a POI_REPORT indicating a PoI has been surveyed."""
        try:
            payload = json.loads(msg.payload_json)
            p_id = payload.get('poi_id')
            if p_id and p_id in self.agent.tasks:
                self.agent.tasks[p_id].status = 3  # SURVEYED
                if p_id in self.agent.bundle:
                    self.agent.bundle.remove(p_id)
                if p_id in self.agent.path:
                    self.agent.path.remove(p_id)
        except Exception:
            pass

    def _on_task_state(self, msg: UAVState):
        if msg.uav_id == self.uav_id and msg.role != self.role:
            self.get_logger().info(
                f'[{self.uav_id}] Role changed from {self.role} to {msg.role}')
            self.role = msg.role

    def _cbba_tick(self):
        """Periodic CBBA round + command synthesis."""
        # Don't run CBBA until PX4 is up and bridge reports valid position
        if not self.position_received:
            return

        if self.role != UAVState.SURVEYOR:
            self._publish_task_state()
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Run CBBA bundle phase
        new_bids = self.agent.bundle_phase(current_time)
        if new_bids:
            self._broadcast_bids(new_bids)
        elif self.agent.bundle and (self.cbba_round % 4 == 0):
            # Re-announce winning bids so newly-booted/reconnected peers synchronize
            current_bids = [
                self.agent.winning_bids[tid]
                for tid in self.agent.bundle
                if tid in self.agent.winning_bids
            ]
            if current_bids:
                self._broadcast_bids(current_bids)

        self.cbba_round += 1

        # Mission replay: if all tasks are done and this is a SURVEYOR, reset
        # them to PENDING so the swarm keeps flying during the demo
        if self.role == UAVState.SURVEYOR and self.scenario_pois_data:
            all_done = all(
                t.status == 3  # SURVEYED
                for t in self.agent.tasks.values()
                if t.task_id in [p['poi_id'] for p in self.scenario_pois_data]
            )
            if all_done and not self.agent.bundle:
                self.get_logger().info(
                    f'[{self.uav_id}] All POIs surveyed — replaying mission (patrol loop)')
                for pdata in self.scenario_pois_data:
                    tid = pdata['poi_id']
                    if tid in self.agent.tasks:
                        self.agent.tasks[tid].status = 0  # reset to PENDING
                        # Clear winning bid so CBBA re-bids
                        self.agent.winning_bids.pop(tid, None)
                self.surveying_task_id = None

        # Publish task state
        self._publish_task_state()

        # Command synthesis: generate velocity toward current target
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
        # If RTH is active, do NOT publish — battery_rth_node owns the velocity channel
        if self.rth_active:
            return
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
            # Fly toward target: normalize direction, scale by cruise speed
            # Decelerate smoothly when within 2x arrival threshold to prevent oscillation
            speed = self.cruise_speed
            if dist < self.arrival_threshold * 2.0:
                speed *= (dist / (self.arrival_threshold * 2.0))
            inv_dist = 1.0 / max(dist, 0.1)
            cmd.linear.x = dx * inv_dist * speed
            cmd.linear.y = dy * inv_dist * speed
            cmd.linear.z = dz * inv_dist * speed
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

                # Broadcast POI_REPORT
                rep_msg = SwarmMessage()
                rep_msg.msg_id = f'rep_{target_id}_{self.uav_id}_{self.get_clock().now().nanoseconds}'
                rep_msg.src_uav_id = self.uav_id
                rep_msg.dst_uav_id = ''  # broadcast
                rep_msg.msg_type = 'POI_REPORT'
                rep_msg.payload_json = json.dumps({
                    'poi_id': target_id,
                    'status': 3,
                    'surveyed_by': self.uav_id,
                    'priority': task.priority,
                })
                rep_msg.origin_stamp = self.get_clock().now().to_msg()
                rep_msg.ttl = 10
                self.comm_tx_pub.publish(rep_msg)

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
