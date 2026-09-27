"""
central_nodes.launch.py — Launch shared/central nodes (once).

Per ARCHITECTURE.md Section 4.6:
  - comm_gateway
  - safety_supervisor
  - fleet_manager
  - mission_logger
  - swarm_viz
  - world_bridge central nodes (geofence/obstacle publisher, collision monitor)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav_ids_arg = DeclareLaunchArgument('uav_ids',
                                         description='Comma-separated UAV IDs')
    scenario_arg = DeclareLaunchArgument('scenario',
                                          default_value='scenarios/scenario_basic.yaml',
                                          description='Scenario YAML path')
    log_path_arg = DeclareLaunchArgument('log_path',
                                         default_value='logs/run.jsonl',
                                         description='Path to output JSONL log file')

    uav_ids = LaunchConfiguration('uav_ids')
    scenario = LaunchConfiguration('scenario')
    log_path = LaunchConfiguration('log_path')

    return LaunchDescription([
        uav_ids_arg,
        scenario_arg,
        log_path_arg,

        # Scenario orchestrator — timed event dispatcher
        Node(
            package='mission_bringup',
            executable='scenario_orchestrator_node',
            name='scenario_orchestrator_node',
            parameters=[{'scenario': scenario, 'uav_ids': uav_ids}],
            output='screen',
        ),

        # Comm gateway — degraded mesh network simulator
        Node(
            package='comm_gateway',
            executable='comm_gateway_node',
            name='comm_gateway_node',
            parameters=[{'uav_ids': uav_ids}],
            output='screen',
        ),

        # Safety supervisor — sole owner of PX4 setpoint topics
        Node(
            package='safety_supervisor',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            parameters=[{'uav_ids': uav_ids}],
            output='screen',
        ),

        # Fleet manager — reserve UAV management
        Node(
            package='fleet_manager',
            executable='fleet_manager_node',
            name='fleet_manager_node',
            parameters=[{'uav_ids': uav_ids, 'scenario': scenario}],
            output='screen',
        ),

        # Mission logger — JSONL output
        Node(
            package='mission_logger',
            executable='mission_logger_node',
            name='mission_logger_node',
            parameters=[{'out_path': log_path, 'uav_ids': uav_ids, 'scenario': scenario}],
            output='screen',
        ),

        # Swarm RViz visualization bridge
        Node(
            package='swarm_viz',
            executable='viz_bridge_node',
            name='viz_bridge_node',
            parameters=[{'uav_ids': uav_ids}],
            output='screen',
        ),

        # Geofence & obstacle publisher (world_bridge)
        Node(
            package='world_bridge',
            executable='geofence_obstacle_publisher_node',
            name='geofence_obstacle_publisher_node',
            parameters=[{'scenario': scenario}],
            output='screen',
        ),

        # Collision & geofence monitor (world_bridge)
        Node(
            package='world_bridge',
            executable='collision_geofence_monitor_node',
            name='collision_geofence_monitor_node',
            parameters=[{'uav_ids': uav_ids}],
            output='screen',
        ),
    ])
