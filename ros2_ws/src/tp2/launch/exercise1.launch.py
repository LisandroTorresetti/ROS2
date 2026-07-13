from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tp2',
            executable='talker',
            output='screen',
            parameters=[
                {"max_counter_value": 256},
                {"publish_rate": 200}
            ]
        ),
        Node(
            package='tp2',
            executable='listener',
            output='screen',
            parameters=[
                {"max_counter_value": 50}
            ]
        ),
    ])