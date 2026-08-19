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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    ExecuteProcess, 
    TimerAction,
)
from launch.event_handlers import OnProcessExit
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
from moveit_configs_utils import MoveItConfigsBuilder


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
        os.path.join(pkg_share, '..') + ':' + 
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

    arg_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='mundo_escritorio.world',
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
            os.path.join(pkg_share, 'robot_description', 'mycobot_320_m5_2022', 'mycobot_320_m5_2022.xacro'),
            ' controllers_file:=',
            os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'ros2_controllers.yaml'),
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

    # El controller_manager no es un nodo propio: lo levanta el plugin
    # gz_ros2_control dentro del proceso de Gazebo, y su loop de update corre
    # en el hilo de física. Si la física va lenta, los servicios del
    # controller_manager tardan en responder y el spawner muere con
    # "Failed to acquire lock in 20 seconds". Por eso el timeout largo.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '120',
        ],
    )
    # El joint_state_broadcaster se lanza después de que el robot esté creado en Gazebo, porque necesita leer las posiciones de las articulaciones para publicar el estado de las mismas.
    event_launch_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Configura y lanza el controlador de esfuerzos PID. 
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'ros2_controllers.yaml'),
            '--controller-manager-timeout', '120',
        ],
    )

    # El controlador de esfuerzos se lanza después de que el joint_state_broadcaster esté corriendo, porque el PID necesita leer las posiciones de las articulaciones para calcular los esfuerzos.    
    event_launch_pid_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        )

    # ==========================================================================
    # MOVE GROUP
    # El nodo central de MoveIt. Recibe pedidos de planificación y publica
    # trayectorias al joint_trajectory_controller.
    # ==========================================================================
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
        .trajectory_execution(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'moveit_controllers.yaml')
        )  
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'joint_limits.yaml')
        )
        .pilz_cartesian_limits(
            file_path=os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'pilz_cartesian_limits.yaml')
        )
        .to_moveit_configs()
    )

    ompl_yaml_path = os.path.join(pkg_share, 'config', 'mycobot_320_m5_2022', 'moveit', 'ompl_planning.yaml')
    with open(ompl_yaml_path, 'r') as f:
        ompl_config = yaml.safe_load(f)

    node_move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'ompl': ompl_config},
            {'use_sim_time': True},
        ],
    )
    

    # ==========================================================================
    # BRIDGE
    # Traduce topics entre Gazebo y ROS2.
    # Sintaxis: /topic@tipo_ros[gz_tipo  significa Gz → ROS2.    
    # ==========================================================================

    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU: Gazebo → ROS2 (el plugin de Gazebo se encargó de publicar este topic)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
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
                #'-r', # Correr la simu inmediatamente despues de cargar
                '-v 1 ',
                PathJoinSubstitution([
                    FindPackageShare('clase5'),
                    'worlds',
                    LaunchConfiguration('world_name')
                ]),
                ' --gui-config ', 
                os.path.join(pkg_share, 'config', 'gazebo.config')
            ]
        )]
    )

    # ==========================================================================
    # ESCENA DE PLANIFICACIÓN
    # Antes se publicaba acá un cilindro "bloque_caible" en (0.3, 0, 0.25), que
    # no se corresponde con ningún objeto de mundo_escritorio.world: era un
    # obstáculo fantasma detrás del robot. La escena real (escritorio, torres,
    # cono y pelota, con las alturas ya asentadas) la publica
    # scripts/move_across_desk.py, que es quien conoce el recorrido.
    # ==========================================================================


    # ==========================================================================
    # NODOS DE USUARIO
    # Interfaces para operar o visualizar el sistema.
    # ==========================================================================
    
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    node_plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='log',
        arguments=['-l', os.path.join(pkg_share, 'config', 'plotjuggler_layout.xml'),'--ros-args', '--log-level', 'WARN'],
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
        arg_world_name,

        # Entorno
        set_resource_path,

        # Simulador
        launch_gazebo,

        # Infraestructura
        event_launch_joint_state_broadcaster_spawner,
        event_launch_pid_controller_spawner,

        node_robot_state_publisher,
        node_gz_spawn_entity,
        node_bridge,

        # MoveIt
        node_move_group,

        # Usuario
        node_rviz,
        node_plotjuggler,
    ])