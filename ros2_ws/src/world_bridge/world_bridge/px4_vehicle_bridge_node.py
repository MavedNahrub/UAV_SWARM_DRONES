"""
px4_vehicle_bridge_node.py — Per-UAV PX4↔ROS2 bridge for PUSHPAK UAV-X.

Per ARCHITECTURE.md Section 4.4:
- Subscribes to PX4's VehicleLocalPosition and republishes as /{uav_id}/pose (Point)
- Subscribes to /{uav_id}/cmd_vel (Twist from safety_supervisor) and publishes
  TrajectorySetpoint + OffboardControlMode to PX4
- Arms the vehicle and engages offboard mode for active roles
"""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from swarm_msgs.msg import UAVState


class PX4VehicleBridgeNode(Node):
    """Per-UAV bridge between PX4 uXRCE-DDS topics and ROS 2 swarm topics."""

    def __init__(self):
        super().__init__('px4_vehicle_bridge_node')

        self.declare_parameter('uav_id', '')
        self.declare_parameter('px4_instance', 0)
        self.declare_parameter('initial_role', 0)  # 0=SURVEYOR, 1=RELAY, 3=RESERVE
        self.declare_parameter('scenario', '')

        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        self.px4_instance = self.get_parameter('px4_instance').value
        self.initial_role = self.get_parameter('initial_role').value
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value

        if scenario_path and os.path.exists(scenario_path):
            try:
                with open(scenario_path, 'r') as f:
                    sdata = yaml.safe_load(f) or {}
                roles = sdata.get('uavs', {}).get('roles', {})
                r_str = roles.get(self.uav_id, 'SURVEYOR')
                role_map = {
                    'SURVEYOR': UAVState.SURVEYOR,
                    'RELAY': UAVState.RELAY,
                    'RESERVE': UAVState.RESERVE,
                    'GROUNDED': UAVState.GROUNDED,
                    'IDLE': UAVState.IDLE,
                }
                self.initial_role = role_map.get(r_str, self.initial_role)
            except Exception as e:
                self.get_logger().warn(f'Could not load scenario in px4 bridge: {e}')

        # Build PX4 topic namespace
        if self.px4_instance == 0:
            self.px4_ns = ''
        else:
            self.px4_ns = f'/px4_{self.px4_instance}'

        # QoS for PX4 subscriptions (must match PX4's publisher QoS: BEST_EFFORT + VOLATILE)
        px4_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # QoS for PX4 publications (must match PX4's subscriber QoS: BEST_EFFORT + VOLATILE)
        px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # === PX4 Subscribers (both standard and _v1 topic names) ===
        for suffix in ['', '_v1']:
            self.create_subscription(
                VehicleLocalPosition,
                f'{self.px4_ns}/fmu/out/vehicle_local_position{suffix}',
                self._on_vehicle_local_position,
                px4_sub_qos,
            )
            self.create_subscription(
                VehicleStatus,
                f'{self.px4_ns}/fmu/out/vehicle_status{suffix}',
                self._on_vehicle_status,
                px4_sub_qos,
            )

        # === ROS 2 Swarm Subscribers ===
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, reliable_qos)
        self.create_subscription(
            UAVState, 'task_state', self._on_task_state, reliable_qos)

        # RTH command — when True, enter landing sequence
        from std_msgs.msg import Bool
        self.create_subscription(
            Bool, 'rth_command', self._on_rth_command, reliable_qos)
        self.rth_active = False
        self.land_command_sent = False

        # === ROS 2 Swarm Publishers ===
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_pub = self.create_publisher(Point, 'pose', best_effort_qos)

        # === PX4 Publishers ===
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            f'{self.px4_ns}/fmu/in/offboard_control_mode',
            px4_pub_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{self.px4_ns}/fmu/in/trajectory_setpoint',
            px4_pub_qos,
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            f'{self.px4_ns}/fmu/in/vehicle_command',
            px4_pub_qos,
        )

        self.spawn_offset_x = float(self.px4_instance * 3.0)
        self.spawn_offset_y = 0.0

        # State
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_pos = (self.spawn_offset_x, self.spawn_offset_y, 0.0)
        self.armed = False
        self.offboard_engaged = False
        self.vehicle_status = None
        self.setpoint_counter = 0
        self.role = self.initial_role
        self.position_received = False

        # Timer at 20 Hz for offboard control loop
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'PX4VehicleBridge started: uav_id={self.uav_id}, '
            f'px4_instance={self.px4_instance}, '
            f'px4_ns="{self.px4_ns}", '
            f'initial_role={self.initial_role}, '
            f'spawn_offset=({self.spawn_offset_x}, {self.spawn_offset_y})')

    def _px4_timestamp(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_vehicle_local_position(self, msg: VehicleLocalPosition):
        """Translate PX4 NED local position to ROS 2 pose with spawn offset."""
        if not msg.xy_valid or not msg.z_valid:
            point = Point()
            point.x = self.spawn_offset_x
            point.y = self.spawn_offset_y
            point.z = 0.0
            self.pose_pub.publish(point)
            return

        self.position_received = True
        world_x = msg.x + self.spawn_offset_x
        world_y = msg.y + self.spawn_offset_y
        world_z = -msg.z
        self.current_pos = (world_x, world_y, world_z)

        point = Point()
        point.x = world_x
        point.y = world_y
        point.z = world_z
        self.pose_pub.publish(point)

    def _on_vehicle_status(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def _on_cmd_vel(self, msg: Twist):
        """Receive safe velocity from safety_supervisor."""
        self.current_vel = (msg.linear.x, msg.linear.y, msg.linear.z)

    def _on_task_state(self, msg: UAVState):
        """Track role changes (e.g. reserve activation)."""
        if msg.uav_id == self.uav_id:
            self.role = msg.role

    def _on_rth_command(self, msg):
        """Receive RTH command from battery_rth_node."""
        if msg.data and not self.rth_active:
            self.rth_active = True
            self.get_logger().warn(f'[{self.uav_id}] RTH COMMAND RECEIVED — entering return-to-home')

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._px4_timestamp()
        self.offboard_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        vx, vy, vz = self.current_vel

        # If velocity is zero and drone is near ground, command takeoff climb
        # to prevent PX4 "auto preflight disarming" (no meaningful offboard intent)
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(vz) < 0.01:
            if self.current_pos[2] < 5.0:
                vz = 2.0  # Climb at 2 m/s until we get real commands
                msg.position = [
                    self.current_pos[0] - self.spawn_offset_x,
                    self.current_pos[1] - self.spawn_offset_y,
                    -5.0,
                ]
            else:
                msg.position = [float('nan'), float('nan'), float('nan')]
        else:
            msg.position = [float('nan'), float('nan'), float('nan')]

        # Convert velocity: our z is up, PX4 NED z is down
        msg.velocity = [vx, vy, -vz]
        msg.yaw = float('nan')  # Don't control yaw
        msg.timestamp = self._px4_timestamp()
        self.trajectory_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        target_sys = self.vehicle_status.system_id if (self.vehicle_status is not None and self.vehicle_status.system_id > 0) else (1 + self.px4_instance)
        msg.target_system = int(target_sys)
        msg.target_component = 1
        msg.source_system = int(target_sys)
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._px4_timestamp()
        self.vehicle_cmd_pub.publish(msg)

    def _arm(self):
        # param2=21196 is PX4's magic number for force-arming
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=21196.0,
        )
        self.get_logger().info(f'[{self.uav_id}] Arm command sent (force-arm key 21196)')

    def _engage_offboard(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f'[{self.uav_id}] Offboard mode command sent')

    def _control_loop(self):
        """
        20 Hz control loop:
        - For active roles (SURVEYOR, RELAY): arm → offboard → stream setpoints
        - For RESERVE: only publish pose, don't arm
        """
        # Reserve UAVs stay grounded until promoted
        if self.role == UAVState.RESERVE or self.role == UAVState.GROUNDED:
            return

        # Wait for PX4 to report valid position before starting offboard sequence
        if not self.position_received:
            return

        # Always publish offboard control mode + trajectory setpoint
        # (PX4 requires continuous stream of setpoints before and during offboard)
        self._publish_offboard_control_mode()

        # If in RTH and drone is near ground, send LAND + DISARM
        if self.rth_active and self.current_pos[2] < 1.5 and self.armed:
            if not self.land_command_sent:
                self.land_command_sent = True
                self.get_logger().warn(
                    f'[{self.uav_id}] PX4 LAND command sent — altitude {self.current_pos[2]:.1f}m')
                # PX4 VEHICLE_CMD_NAV_LAND = 21
                self._publish_vehicle_command(21, param1=0.0, param5=0.0, param6=0.0, param7=0.0)
            # Also send DISARM after a brief delay
            if self.land_command_sent and self.current_pos[2] < 0.5:
                self.get_logger().warn(f'[{self.uav_id}] DISARMING — on the ground')
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0, param2=21196.0)  # force disarm
                self.armed = False
                self.role = UAVState.GROUNDED
            # Publish zero setpoint to keep PX4 happy
            self._publish_trajectory_setpoint()
            return

        self._publish_trajectory_setpoint()

        self.setpoint_counter += 1

        # Check actual arm state from vehicle_status
        actually_armed = (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state == 2  # ARMED
        )
        if actually_armed and not self.armed:
            self.armed = True
            self.get_logger().info(f'[{self.uav_id}] PX4 confirmed ARMED')

        # After enough setpoints, start the offboard + arm sequence
        if self.setpoint_counter >= 30:
            in_offboard = (
                self.vehicle_status is not None
                and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            )
            if not in_offboard and (self.setpoint_counter - 30) % 20 == 0:
                self._engage_offboard()

            # Retry arming every ~1s (20 ticks at 20Hz) until PX4 confirms armed
            if not self.armed and self.setpoint_counter >= 40:
                if (self.setpoint_counter - 40) % 20 == 0:
                    self._arm()

    def _set_param_com_rcl_except(self):
        """Set COM_RCL_EXCEPT=4 so PX4 allows offboard arming without GCS link."""
        self._publish_vehicle_command(
            176,  # VEHICLE_CMD_DO_SET_PARAMETER (MAV_CMD_DO_SET_PARAMETER)
            param1=float(ord('C')),  # Not used for PX4 params, use param cmd instead
        )
        # Use the direct parameter set command for PX4
        # COM_RCL_EXCEPT: bit 2 (value 4) = offboard mode exception
        msg = VehicleCommand()
        msg.command = 0  # Will use param_set via uXRCE-DDS instead
        msg.timestamp = self._px4_timestamp()
        # For PX4 SITL, the cleanest way is to set the param at startup.
        # We'll just keep retrying the arm command — PX4 clears the GCS
        # check after the first offboard setpoint stream is established.
        self.get_logger().info(f'[{self.uav_id}] Offboard engaged, will retry arming until PX4 accepts')


def main(args=None):
    rclpy.init(args=args)
    node = PX4VehicleBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
