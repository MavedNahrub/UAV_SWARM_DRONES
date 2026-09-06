from setuptools import setup

package_name = 'comm_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='devamburhan',
    maintainer_email='devamburhan@example.com',
    description='Degraded-mesh network simulator for the PUSHPAK UAV-X swarm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm_gateway_node = comm_gateway.comm_gateway_node:main',
        ],
    },
)
