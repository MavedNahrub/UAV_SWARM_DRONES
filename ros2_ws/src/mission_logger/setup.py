from setuptools import setup

package_name = 'mission_logger'

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
    description='Mission logger package for PUSHPAK UAV-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_logger_node = mission_logger.mission_logger_node:main',
        ],
    },
)
