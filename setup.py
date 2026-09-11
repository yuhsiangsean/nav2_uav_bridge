import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav2_uav_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yuhsiangsean.en15@nycu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_bridge = nav2_uav_bridge.odom_bridge:main',
            'cmd_vel_bridge = nav2_uav_bridge.cmd_vel_bridge:main',
            'fake_scan_publisher = nav2_uav_bridge.fake_scan_publisher:main',
            'apriltag_obstacle_bridge = nav2_uav_bridge.apriltag_obstacle_bridge:main',
        ],
    },
)
