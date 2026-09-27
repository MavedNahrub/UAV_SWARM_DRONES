"""
uav_stack.launch.py — Launch ONE UAV's application nodes (namespaced).

Per ARCHITECTURE.md Section 4.5. Each per-UAV node is launched here:
task_allocation, relay_placement, fault_recovery, battery_rth, world_bridge.

This launch file is included N times (once per UAV) with a PushRosNamespace
in multi_uav_mission.launch.py. It can also be launched standalone by passing
the 'namespace' argument (used by launch_sim_gui.sh).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    uav_id = LaunchConfiguration('uav_id').perform(context)
    px4_instance = int(LaunchConfiguration('px4_instance').perform(context))
    scenario = LaunchConfiguration('scenario').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    nodes = [
        # PX4 vehicle bridge (world_bridge)
        Node(
            package='world_bridge',
            executable='px4_vehicle_bridge_node',
            name='px4_vehicle_bridge_node',
            parameters=[{
                'uav_id': uav_id,
                'px4_instance': px4_instance,
                'scenario': scenario,
            }],
            output='screen',
        ),

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
            parameters=[{
                'uav_id': uav_id,
                'bundle_capacity': 3,
                'scenario': scenario,
            }],
            output='screen',
        ),

        # Relay placement
        Node(
            package='relay_placement',
            executable='relay_node',
            name='relay_node',
            parameters=[{
                'uav_id': uav_id,
                'scenario': scenario,
            }],
            output='screen',
        ),

        # Fault manager
        Node(
            package='fault_recovery',
            executable='fault_manager_node',
            name='fault_manager_node',
            parameters=[{'uav_id': uav_id, 'timeout_s': 20.0, 'heartbeat_period_s': 2.0}],
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
    ]

    # Apply namespace if provided (for standalone launch from shell scripts)
    if namespace:
        return [GroupAction([PushRosNamespace(namespace)] + nodes)]
    else:
        return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uav_id', description='UAV identifier'),
        DeclareLaunchArgument('px4_instance', description='PX4 instance number'),
        DeclareLaunchArgument('scenario', default_value='', description='Scenario YAML path'),
        DeclareLaunchArgument('namespace', default_value='', description='ROS namespace (for standalone launch)'),
        OpaqueFunction(function=launch_setup),
    ])

