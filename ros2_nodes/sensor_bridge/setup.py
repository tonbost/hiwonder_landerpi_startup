from setuptools import setup

package_name = 'sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='landerpi@example.com',
    description='Bridge ROS2 topics to JSON files for low-latency reading from host Python',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_bridge_node = sensor_bridge.sensor_bridge_node:main',
        ],
    },
)
