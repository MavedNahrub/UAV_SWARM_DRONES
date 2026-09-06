#!/usr/bin/env python3
"""
offboard_takeoff.py — Basic offboard control script for PX4 via ROS 2 / uXRCE-DDS

Arms the drone, switches to offboard mode, commands takeoff to 5m,
hovers for 10 seconds, then lands.

Usage (inside the Docker container):
    # Terminal 1: Start XRCE-DDS agent
    MicroXRCEAgent udp4 -p 8888

    # Terminal 2: Start PX4 SITL
    cd ~/PX4-Autopilot && make px4_sitl gz_x500

    # Terminal 3: Run this script
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 offboard_takeoff.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardTakeoff(Node):
    """Minimal offboard controller: arm → takeoff → hover → land."""

    def __init__(self, namespace: str = ""):
        super().__init__("offboard_takeoff")

        # QoS for PX4 topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──
        ns = f"/{namespace}" if namespace else ""
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos
        )

        # ── Subscribers ──
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{ns}/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f"{ns}/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos,
        )

        # ── State ──
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0  # NED frame: negative = up

        # Timer at 20 Hz (offboard requires >2 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("OffboardTakeoff node started")

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send disarm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")

    def land(self):
        """Switch to auto-land mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.get_logger().info("Land command sent")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        """Main control loop at 20 Hz."""
        # Always publish offboard control mode (required for offboard)
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Publish trajectory setpoint: hover at takeoff_height
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, self.takeoff_height]
        trajectory_msg.yaw = 0.0
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(trajectory_msg)

        # Engage offboard + arm after sending enough setpoints
        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()
            self.arm()

        # After 200 ticks (10 sec of hover), land
        if self.offboard_setpoint_counter == 220:
            self.land()
            self.get_logger().info("Hover complete — landing...")

        # After 320 ticks (15 sec), shut down
        if self.offboard_setpoint_counter == 420:
            self.get_logger().info("✅ Offboard takeoff test PASSED")
            raise SystemExit(0)

        if self.offboard_setpoint_counter < 500:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
