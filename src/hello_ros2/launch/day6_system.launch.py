import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration




def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic') #创建，运行时才决定具体值
    lidar_config = os.path.join(
        get_package_share_directory('hello_ros2'), #找到hello_ros2包的路径
        'config',
        'day6_lidar.yaml'
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
    'scan_topic',
    default_value='/front_scan'),

        Node(
            package='hello_ros2',
            executable='fake_lidar_node',
            name='fake_lidar',
            parameters=[lidar_config],
            remappings=[
                ('/scan', scan_topic)
            ],
            output='screen'
        ),

        Node(
            package='hello_ros2',
            executable='scan_subscriber',
            name='scan_subscriber',
            remappings=[
                ('/scan', scan_topic)
            ],
            output='screen'
        )

    ])