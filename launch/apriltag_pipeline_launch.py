from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {'use_sim_time': True}

    spawn_marker = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-file', '/root/PX4-Autopilot/Tools/simulation/gz/models/apriltag_marker_0/model.sdf',
            '-name', 'apriltag_marker_0',
            '-x', '2', '-y', '0', '-z', '0', '-Y', '1.5708',
        ],
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
                '/camera/image_raw'),
            ('/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info',
                '/camera/camera_info'),
        ],
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=['/root/ros2_ws/src/nav2_uav_bridge/params/apriltag.yaml'],
    )

    apriltag_obstacle_bridge = Node(
        package='nav2_uav_bridge',
        executable='apriltag_obstacle_bridge',
        parameters=[use_sim_time],
    )

    return LaunchDescription([
        camera_bridge,
        apriltag_node,
        apriltag_obstacle_bridge,
        # 立牌要等 Gazebo 世界完全載入才能生成，晚一點點下手比較保險
        TimerAction(period=5.0, actions=[spawn_marker]),
    ])
