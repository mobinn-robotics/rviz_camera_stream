from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # launch rviz with config/rviz_camera_stream.rviz
        Node(package='rviz2', executable='rviz2', name='rviz',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('rviz_camera_stream'), 'config', 'rviz_camera_stream.rviz'])],
             output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             # ROS1 disable_pub_plugins (compressed, compressedDepth, theora) of rviz1/camera1/image and rviz1/camera2/image
             parameters=[{
                 'rviz1.camera1.image.enable_pub_plugins': ['image_transport/raw'],
                 'rviz1.camera2.image.enable_pub_plugins': ['image_transport/raw'],
             }]),

        GroupAction([
            PushRosNamespace('camera1'),
            Node(package='tf2_ros', executable='static_transform_publisher', name='camera_broadcaster',
                 arguments=['--x', '0.1', '--y', '0.1', '--z', '-0.5', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                            '--frame-id', 'map', '--child-frame-id', 'camera1']),
            ExecuteProcess(name='camera_info',
                           cmd=['ros2', 'topic', 'pub', '-r', '2', '/camera1/camera_info', 'sensor_msgs/msg/CameraInfo',
                                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera1'},"
                                " height: 480, width: 640, distortion_model: 'plumb_bob',"
                                " d: [0.0],"
                                " k: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],"
                                " r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],"
                                " p: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],"
                                " binning_x: 0, binning_y: 0,"
                                " roi: {x_offset: 0, y_offset: 0, height: 480, width: 640, do_rectify: false}}"],
                           output='screen'),
        ]),

        GroupAction([
            PushRosNamespace('camera2'),
            Node(package='tf2_ros', executable='static_transform_publisher', name='camera_broadcaster',
                 arguments=['--x', '0.1', '--y', '-0.1', '--z', '-2.5', '--qx', '0.01', '--qy', '0', '--qz', '0.02', '--qw', '1',
                            '--frame-id', 'map', '--child-frame-id', 'camera2']),
            ExecuteProcess(name='camera_info',
                           cmd=['ros2', 'topic', 'pub', '-r', '2', '/camera2/camera_info', 'sensor_msgs/msg/CameraInfo',
                                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera2'},"
                                " height: 720, width: 1280, distortion_model: 'plumb_bob',"
                                " d: [0.0],"
                                " k: [300.0, 0.0, 640.0, 0.0, 300.0, 360.0, 0.0, 0.0, 1.0],"
                                " r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],"
                                " p: [300.0, 0.0, 640.0, 0.0, 0.0, 300.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0],"
                                " binning_x: 0, binning_y: 0,"
                                " roi: {x_offset: 0, y_offset: 0, height: 720, width: 1280, do_rectify: false}}"],
                           output='screen'),
        ]),
    ])
