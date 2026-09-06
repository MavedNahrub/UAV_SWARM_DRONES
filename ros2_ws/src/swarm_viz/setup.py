from setuptools import setup

package_name = 'swarm_viz'

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
    description='Swarm RViz visualization package for PUSHPAK UAV-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viz_bridge_node = swarm_viz.viz_bridge_node:main',
        ],
    },
)
