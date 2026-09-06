from setuptools import setup
package_name = 'relay_placement'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools', 'networkx'], zip_safe=True,
    maintainer='devamburhan', maintainer_email='devamburhan@example.com',
    description='Relay placement for PUSHPAK UAV-X swarm', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['relay_node = relay_placement.relay_node:main']},
)
