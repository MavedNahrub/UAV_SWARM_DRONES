"""
xrce_agent.launch.py — Launch a single Micro XRCE-DDS Agent shared by all PX4 instances.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
        ),
    ])
