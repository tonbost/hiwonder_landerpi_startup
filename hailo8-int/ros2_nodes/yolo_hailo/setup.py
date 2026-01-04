from setuptools import find_packages, setup

package_name = 'yolo_hailo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='user@example.com',
    description='Hailo-accelerated YOLO object detection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_hailo_node = yolo_hailo.yolo_hailo_node:main',
        ],
    },
)
