from setuptools import find_packages, setup

package_name = 'lidar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='robot@landerpi.local',
    description='ROS2 driver for LD19/MS200 lidar on LanderPi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_driver_node = lidar_driver.ld19_driver_node:main',
        ],
    },
)
