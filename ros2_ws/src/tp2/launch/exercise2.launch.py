from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tp2',
            executable='actionserver',
            output='screen'
        ),
        Node(
            package='tp2',
            executable='textpublisher',
            output='screen',
            parameters=[
                {
                    "text": "hola que tal tu como estas? dime si eres feliz"
                }
            ]
        ),
    ])