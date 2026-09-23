from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_uav_bridge_dir = get_package_share_directory('nav2_uav_bridge')

    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='MAV4',
        description='Namespace prefix PX4 publishes /fmu/... topics under (e.g. MAV4 for /MAV4/fmu/...). Empty string for no prefix.',
    )
    px4_namespace = LaunchConfiguration('px4_namespace')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.12', '--y', '0.03', '--z', '0.242',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
        ],
    )

    odom_bridge = Node(
        package='nav2_uav_bridge',
        executable='odom_bridge',
        parameters=[{'px4_namespace': px4_namespace}],
    )

    fake_scan_publisher = Node(
        package='nav2_uav_bridge',
        executable='fake_scan_publisher',
    )

    cmd_vel_bridge = Node(
        package='nav2_uav_bridge',
        executable='cmd_vel_bridge',
        parameters=[{'px4_namespace': px4_namespace}],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(nav2_uav_bridge_dir, 'maps', 'blank_map.yaml'),
            'params_file': os.path.join(nav2_uav_bridge_dir, 'params', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        px4_namespace_arg,
        static_tf,
        base_footprint_tf,
        camera_tf,
        odom_bridge,
        cmd_vel_bridge,
        nav2_bringup,
        fake_scan_publisher,
    ])
