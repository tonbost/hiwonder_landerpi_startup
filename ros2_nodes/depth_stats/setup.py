from setuptools import setup

package_name = 'depth_stats'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='landerpi@example.com',
    description='Depth camera stats publisher for Aurora 930',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_stats_node = depth_stats.depth_stats_node:main',
        ],
    },
)
