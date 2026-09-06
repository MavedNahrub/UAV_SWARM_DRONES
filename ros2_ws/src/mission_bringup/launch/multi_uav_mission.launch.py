"""
multi_uav_mission.launch.py — Top-level entry point for the PUSHPAK UAV-X swarm.

Launches:
  1. Gazebo Harmonic world (once)
  2. Micro XRCE-DDS Agent (once)
  3. N PX4 SITL instances (per-vehicle)
  4. N UAV app-node stacks (per-vehicle, namespaced)
  5. Central/shared nodes (once)

Per ARCHITECTURE.md Section 4.3.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))
    scenario = LaunchConfiguration('scenario').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    pkg_dir = get_package_share_directory('mission_bringup')
    launch_dir = os.path.join(pkg_dir, 'launch')

    actions = []

    actions.append(LogInfo(msg=f'Launching PUSHPAK UAV-X swarm: {num_uavs} UAVs, scenario={scenario}'))

    # 1. Gazebo Harmonic, once, with the mission world
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo_world.launch.py')),
        launch_arguments={'headless': str(headless)}.items(),
    ))

    # 2. Micro XRCE-DDS Agent, once, shared by all PX4 instances
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'xrce_agent.launch.py')),
    ))

    # 3. N PX4 SITL instances into the same Gazebo world
    for i in range(num_uavs):
        uav_id = f'uav_{i:02d}'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'px4_vehicle.launch.py')),
            launch_arguments={
                'instance': str(i),
                'uav_id': uav_id,
            }.items(),
        ))

    # 4. Per-UAV app node stacks, namespaced
    for i in range(num_uavs):
        uav_id = f'uav_{i:02d}'
        actions.append(GroupAction([
            PushRosNamespace(uav_id),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'uav_stack.launch.py')),
                launch_arguments={
                    'uav_id': uav_id,
                    'px4_instance': str(i),
                }.items(),
            ),
        ]))

    # 5. Central/shared nodes
    uav_ids = ','.join(f'uav_{i:02d}' for i in range(num_uavs))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'central_nodes.launch.py')),
        launch_arguments={
            'uav_ids': uav_ids,
            'scenario': scenario,
        }.items(),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='6',
                              description='Number of UAVs to spawn'),
        DeclareLaunchArgument('scenario', default_value='scenarios/scenario_basic.yaml',
                              description='Path to scenario YAML file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo in headless mode'),
        OpaqueFunction(function=launch_setup),
    ])
