from setuptools import setup

package_name = 'battery_rth'

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
    description='Battery RTH package for PUSHPAK UAV-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_rth_node = battery_rth.battery_rth_node:main',
        ],
    },
)
