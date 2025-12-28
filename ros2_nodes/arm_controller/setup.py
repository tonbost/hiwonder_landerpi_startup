from setuptools import find_packages, setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='robot@landerpi.local',
    description='ROS2 arm controller for LanderPi 5-DOF arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller_node = arm_controller.arm_controller_node:main',
        ],
    },
)
