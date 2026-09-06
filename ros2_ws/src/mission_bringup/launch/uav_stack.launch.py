"""
uav_stack.launch.py — Launch ONE UAV's application nodes (namespaced).

Per ARCHITECTURE.md Section 4.5. Each per-UAV node is launched here:
task_allocation, relay_placement, fault_recovery, battery_rth, world_bridge.

This launch file is included N times (once per UAV) with a PushRosNamespace
in multi_uav_mission.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav_id_arg = DeclareLaunchArgument('uav_id', description='UAV identifier')
    px4_instance_arg = DeclareLaunchArgument('px4_instance', description='PX4 instance number')

    uav_id = LaunchConfiguration('uav_id')

    return LaunchDescription([
        uav_id_arg,
        px4_instance_arg,

        # Virtual battery (world_bridge)
        Node(
            package='world_bridge',
            executable='virtual_battery_node',
            name='virtual_battery_node',
            parameters=[{'uav_id': uav_id}],
            output='screen',
        ),

        # CBBA task allocation
        Node(
            package='task_allocation',
            executable='cbba_node',
            name='cbba_node',
            parameters=[{'uav_id': uav_id, 'bundle_capacity': 3}],
            output='screen',
        ),

        # Relay placement
        Node(
            package='relay_placement',
            executable='relay_node',
            name='relay_node',
            parameters=[{'uav_id': uav_id}],
            output='screen',
        ),

        # Fault manager
        Node(
            package='fault_recovery',
            executable='fault_manager_node',
            name='fault_manager_node',
            parameters=[{'uav_id': uav_id, 'timeout_ticks': 10}],
            output='screen',
        ),

        # Battery / RTH
        Node(
            package='battery_rth',
            executable='battery_rth_node',
            name='battery_rth_node',
            parameters=[{'uav_id': uav_id, 'critical_frac': 0.2}],
            output='screen',
        ),
    ])
