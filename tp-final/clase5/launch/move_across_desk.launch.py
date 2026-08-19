# Lanza el recorrido que cruza el escritorio esquivando las torres.
#
# Por que un launch y no `ros2 run`:
#   - `use_sim_time` tiene que estar en true. Todo el resto del sistema corre
#     con el reloj de Gazebo, y si este nodo usa el reloj de pared los
#     timestamps de la trayectoria no coinciden con los del controlador.
#   - Los parametros de MoveIt (robot_description, semantic, kinematics,
#     joint_limits) quedan cargados igual que en hello_moveit.launch.py. Este
#     script en particular habla con move_group por action/service y no
#     construye un RobotModel propio, asi que no los necesita, pero dejarlos
#     puestos permite cambiarlo a MoveGroupInterface o MoveItPy sin tocar el
#     launch.
#
# Uso:
#   Terminal 1:  ros2 launch clase5 mycobot_launch.py
#   Terminal 2:  ros2 launch clase5 move_across_desk.launch.py

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

    move_across_desk_node = Node(
        package='clase5',
        executable='move_across_desk.py',
        name='move_across_desk',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([move_across_desk_node])
