"""
gazebo_world.launch.py — Launch Gazebo Harmonic with the disaster_area world.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    pkg_dir = get_package_share_directory('mission_bringup')
    world_file = os.path.join(pkg_dir, 'worlds', 'disaster_area.sdf')

    cmd = ['gz', 'sim', '-r']
    if headless:
        cmd.append('-s')
    cmd.append(world_file)

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.path.expanduser(
                    '~/PX4-Autopilot/Tools/simulation/gz/models:'
                    '~/PX4-Autopilot/Tools/simulation/gz/worlds'
                ),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': os.path.expanduser(
                    '~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins'
                ),
                'GZ_SIM_SERVER_CONFIG_PATH': os.path.expanduser(
                    '~/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config'
                ),
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'QT_X11_NO_MITSHM': '1',
            },
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
