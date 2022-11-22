from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ip_launch_arg = DeclareLaunchArgument(
        'ip', default_value='172.26.1.0'
    )
    
    return LaunchDescription([
        ip_launch_arg,
        Node(
            package='robotino2',
            executable='robotino2_node',
            parameters=[{
                'ip': LaunchConfiguration('ip')
            }]
        )
    ])