from setuptools import setup

package_name = 'world_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devamburhan',
    maintainer_email='devamburhan@example.com',
    description='World bridge and virtual environment package for PUSHPAK UAV-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_battery_node = world_bridge.virtual_battery_node:main',
            'geofence_obstacle_publisher_node = world_bridge.geofence_obstacle_publisher_node:main',
            'collision_geofence_monitor_node = world_bridge.collision_geofence_monitor_node:main',
            'px4_vehicle_bridge_node = world_bridge.px4_vehicle_bridge_node:main',
        ],
    },
)
