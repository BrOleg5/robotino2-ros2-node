from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'ip', default_value='172.26.1.0'
        ),
        Node(
            package='robotino2',
            executable='robotino2_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'ip': LaunchConfiguration('ip')
            }]
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            remappings=[
                ("/cmd_vel", "/robotino2/cmd_vel")
            ]
        )
    ])
