# Lanza el ejemplo C++ `hello_moveit` con los parámetros de MoveIt cargados.
#
# El nodo MoveGroupInterface necesita robot_description, robot_description_semantic,
# robot_description_kinematics y joint_limits en su parameter server para poder
# construir el RobotModel. Con `ros2 run` no los tiene y falla al arrancar.
#
# Uso:
#   Terminal 1:  ros2 launch clase5 mycobot_launch.py
#   Terminal 2:  ros2 launch clase5 hello_moveit.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('clase5')

    moveit_config = (
        MoveItConfigsBuilder('myCobot320', package_name='clase5')
        .robot_description(
            file_path=os.path.join(pkg_share, 'robot_description', 'mycobot_320_m5_2022', 'mycobot_320_m5_2022.xacro')
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'mycobot_320_m5_2022.srdf')
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'kinematics.yaml')
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'joint_limits.yaml')
        )
        .pilz_cartesian_limits(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'pilz_cartesian_limits.yaml')
        )
        .to_moveit_configs()
    )

    hello_moveit_node = Node(
        package='clase5',
        executable='hello_moveit',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([hello_moveit_node])
