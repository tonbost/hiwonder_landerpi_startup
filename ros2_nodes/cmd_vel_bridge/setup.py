import os
from glob import glob
from setuptools import setup

package_name = 'cmd_vel_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='user@example.com',
    description='cmd_vel to motor bridge for LanderPi mecanum robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = cmd_vel_bridge.cmd_vel_bridge_node:main',
        ],
    },
)
