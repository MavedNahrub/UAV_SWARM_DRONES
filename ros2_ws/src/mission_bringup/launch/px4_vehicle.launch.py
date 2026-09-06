"""
px4_vehicle.launch.py — Launch ONE PX4 SITL instance + topic remaps.

Per ARCHITECTURE.md Section 4.4. Spawns a PX4 process at a given pose into
the already-running Gazebo world.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    instance = int(LaunchConfiguration('instance').perform(context))
    uav_id = LaunchConfiguration('uav_id').perform(context)

    pkg_dir = get_package_share_directory('mission_bringup')
    poses_file = os.path.join(pkg_dir, 'config', 'vehicle_spawn_poses.yaml')

    # Load spawn pose from config
    with open(poses_file, 'r') as f:
        poses_config = yaml.safe_load(f)

    pose = poses_config.get('spawn_poses', {}).get(uav_id, {})
    x = pose.get('x', instance * 3.0)
    y = pose.get('y', 0.0)
    z = pose.get('z', 0.0)
    yaw = pose.get('yaw', 0.0)

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')

    instance_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default', f'instance_{instance}')
    os.makedirs(instance_dir, exist_ok=True)

    # Clean up any stale lock/socket for this instance
    for f in [f'/tmp/px4_lock-{instance}', f'/tmp/px4-sock-{instance}']:
        if os.path.exists(f):
            try:
                os.remove(f)
            except OSError:
                pass

    spawn_pose_str = f'{x},{y},{z},0,0,{yaw}'

    actions = []

    # PX4 SITL instance
    actions.append(ExecuteProcess(
        cmd=[
            px4_bin,
            '-i', str(instance),
            '-d',
            os.path.join(px4_dir, 'ROMFS', 'px4fmu_common'),
        ],
        cwd=instance_dir,
        additional_env={
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_GZ_MODEL_POSE': spawn_pose_str,
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_GZ_WORLD': 'default',
        },
        output='screen',
        name=f'px4_sitl_{instance}',
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('instance', description='PX4 instance number (0-based)'),
        DeclareLaunchArgument('uav_id', description='UAV identifier string (e.g. uav_00)'),
        OpaqueFunction(function=launch_setup),
    ])
