"""
gazebo_world.launch.py — Launch Gazebo Harmonic with the disaster_area world.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mission_bringup')
    world_file = os.path.join(pkg_dir, 'worlds', 'disaster_area.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.path.expanduser(
                    '~/PX4-Autopilot/Tools/simulation/gz/models:'
                    '~/PX4-Autopilot/Tools/simulation/gz/worlds'
                ),
            },
        ),
    ])
