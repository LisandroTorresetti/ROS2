# Copyright 2026 pgonzal@fi.uba.ar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # ==========================================================================
    # VARIABLES DE ENTORNO
    # Se setean con os.environ (no con SetEnvironmentVariable) porque deben estar
    # disponibles ANTES de que cualquier proceso hijo arranque. SetEnvironmentVariable
    # es una acción del grafo de launch y se ejecuta demasiado tarde para esto.
    # ==========================================================================

    # Le dice a Gazebo dónde buscar plugins de sistema (ej: gz-sim-imu-system).
    # Se extiende en lugar de pisarse por si ya había algo seteado.
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
        '/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins:'
        + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )

    # ==========================================================================
    # PATHS DEL PAQUETE
    # Se calculan una sola vez y se reusan abajo.
    # ==========================================================================

    pkg_share = FindPackageShare('clase5').find('clase5')

    # ==========================================================================
    # ACCIONES DE ENTORNO
    # SetEnvironmentVariable SÍ es adecuado para GZ_SIM_RESOURCE_PATH porque
    # Gazebo la lee en tiempo de ejecución cuando carga modelos, no al arrancar.
    # ==========================================================================

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, '..') + ':' +  # share/ → Gazebo encuentra clase5/robot_description/...
        os.path.join(pkg_share, 'models') + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # ==========================================================================
    # ARGUMENTOS DE LAUNCH
    # Permiten parametrizar el launcher desde línea de comandos.
    # ==========================================================================

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='True para usar un clock simulado'
    )

    arg_xacro_file = DeclareLaunchArgument(
        'xacro_file',
        default_value='dp/dp.xacro',
        description='Archivo de definición del robot'
    )

    arg_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='mundo_vacio.world',
        description='Nombre del archivo del mundo para Gazebo'
    )

    # ==========================================================================
    # DESCRIPCIÓN DEL ROBOT
    # xacro genera el URDF en tiempo de launch. El resultado se comparte como
    # parámetro entre robot_state_publisher y gz_spawn_entity.
    # ==========================================================================

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('clase5'),
                'robot_description',
                LaunchConfiguration('xacro_file')
            ]),
        ])
    }


    # ==========================================================================
    # NODOS DE INFRAESTRUCTURA
    # Son los nodos necesarios para que el sistema funcione internamente.
    # Se definen todos acá arriba porque algunos se referencian en event handlers.
    # ==========================================================================

    # Publica la descripción del robot y el árbol de transformaciones TF.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Crea la entidad del robot en el mundo de Gazebo a partir del URDF.
    node_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mi_robot',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.553',
        ],
    )

    # Traduce topics entre Gazebo y ROS2.
    # Sintaxis: /topic@tipo_ros[gz_tipo  significa Gz → ROS2.
    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states: Gazebo → ROS2
            '/world/mundo_simulacion/model/mi_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # IMU: Gazebo → ROS2 (el plugin de Gazebo se encargó de publicar este topic)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Torque en ejes: ROS2 → Gazebo (el plugin de Gazebo se encargará de suscribirse a este topic)
            '/model/mi_robot/joint/joint1/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/mi_robot/joint/joint2/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            # Remapeo al topic estándar que espera rviz/plotjuggler
            ('/world/mundo_simulacion/model/mi_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # ==========================================================================
    # LAUNCH DE GAZEBO
    # Se incluye el launcher de ros_gz_sim pasándole el mundo y la config de GUI.
    # El flag -r arranca la simulación automáticamente; -v 1 reduce el verbosity.
    # ==========================================================================

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[(
            'gz_args', [
                #'-r', # Descomentar para correr la simu inmediatamente despues de cargar
                '-v 1 ',
                PathJoinSubstitution([
                    FindPackageShare('clase5'),
                    'worlds',
                    LaunchConfiguration('world_name')
                ]),
                ' --gui-config ',
                PathJoinSubstitution([
                    FindPackageShare('clase5'),
                    'config',
                    'gazebo.config'
                ])
            ]
        )]
    )

    # ==========================================================================
    # NODOS DE USUARIO
    # Interfaces para operar o visualizar el sistema.
    # ==========================================================================

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')]
    )

    node_gui_torque = Node(
        package='clase5',
        executable='gui_apply_torque.py',
        name='gui_apply_torque',
        parameters=[{'robot_name': 'mi_robot'}],
        output='screen',
    )


    # ==========================================================================
    # LAUNCH DESCRIPTION
    # Orden de declaración: args → entorno → Gazebo → infraestructura → usuario.
    # Los nodos manejados por event handlers (joint_state_broadcaster,
    # effort_controller) NO se incluyen acá; los disparan los handlers.
    # ==========================================================================

    return LaunchDescription([
        # Argumentos
        arg_use_sim_time,
        arg_xacro_file,
        arg_world_name,

        # Entorno
        set_resource_path,

        # Simulador
        launch_gazebo,

        # Infraestructura
        node_robot_state_publisher,
        node_gz_spawn_entity,
        node_bridge,

        # Usuario
        node_rviz,
        node_gui_torque,
    ])