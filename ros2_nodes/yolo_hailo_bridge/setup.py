from setuptools import setup

package_name = 'yolo_hailo_bridge'

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
    description='ZeroMQ bridge to host Hailo inference server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = yolo_hailo_bridge.bridge_node:main',
        ],
    },
)
