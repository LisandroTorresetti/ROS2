#!/usr/bin/env python3
# Mueve el myCobot 320 con pinza desde su pose de reposo hasta un punto del
# otro lado del escritorio, esquivando la pared de torres de mundo_escritorio.
#
# El recorrido tiene dos tramos:
#   1) "listo"   -> se levanta sobre el lado cercano del escritorio
#   2) "cruzado" -> pasa al otro lado de la pared de torres
#
# La recta entre los dos puntos atraviesa tower2box2 y el cono, asi que
# RRTConnect esta obligado a rodear la pared (por los huecos en y ~ +-0.1) o a
# pasarle por encima. Si el planner devuelve una trayectoria casi recta,
# es senal de que la escena no se publico.
#
# Uso:
#   Terminal 1:  ros2 launch clase5 mycobot_launch.py
#   Terminal 2:  ros2 run clase5 move_across_desk.py

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, WorkspaceParameters,
                             Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume,
                             PlanningScene, CollisionObject)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose, Vector3
from shape_msgs.msg import SolidPrimitive

# =====================================================================
# CONFIGURACION GENERAL
# =====================================================================
PLANNING_GROUP = 'arm'
EE_LINK = 'tool0'
REF_FRAME = 'base_link'

# mycobot_launch.py spawnea el robot en z=0.553 y la tapa del escritorio esta
# en z=0.552, o sea que el z=0 de base_link es (a 1mm) la superficie de la
# mesa. Todo lo que sigue usa coordenadas de base_link.
ALTURA_ROBOT = 0.553

# Orientacion de la pinza: (1,0,0,0) es una rotacion de 180 grados en x, que
# deja el +z de tool0 (el eje de aproximacion) apuntando hacia abajo.
PINZA_ABAJO = (1.0, 0.0, 0.0, 0.0)

# =====================================================================
# TRAYECTO
# Cada waypoint es (nombre, x, y, z). Verificados con IK: ambos alcanzables.
# =====================================================================
WAYPOINTS = [
      # Approach tower3 from above (it's only 1 box tall, easy)
      ('sobre_torre2',     -0.20, 0.00,  0.36),
      ('volver',     -0.05, 0.00,  0.25),
  ]


TOLERANCIA_POS = 0.01    # radio de la esfera de tolerancia, en metros
TOLERANCIA_ORI = 0.10    # tolerancia angular por eje, en radianes
TIEMPO_PLANIFICACION = 10.0
INTENTOS = 10

# =====================================================================
# OBSTACULOS
# Los objetos del mundo caen y se acomodan al darle play, asi que lo que se
# publica es la posicion DE REPOSO, no la del spawn. Ejemplo: tower1box2
# aparece en z=0.8 pero termina apoyada sobre tower1box1, en z=0.702.
#
# 'z_mundo' es la altura final en el frame del mundo; se le resta ALTURA_ROBOT
# para pasarla a base_link.
# =====================================================================
OBSTACULOS = [
    # La tapa del escritorio. Se modela 1.5cm por debajo de base_link para no
    # solaparse con la malla de colision de base_link, que apoya justo encima.
    {'id': 'escritorio', 'type': 'box', 'xy': (-0.214, 0.0),
     'z_mundo': 0.518, 'dims': (0.490, 0.844, 0.040)},

    # Torre 1 (3 cajas) en y=+0.2
    {'id': 'tower1box1', 'type': 'box', 'xy': (-0.2, 0.2), 'z_mundo': 0.602, 'dims': (0.1, 0.1, 0.1)},
    {'id': 'tower1box2', 'type': 'box', 'xy': (-0.2, 0.2), 'z_mundo': 0.702, 'dims': (0.1, 0.1, 0.1)},
    {'id': 'tower1box3', 'type': 'box', 'xy': (-0.2, 0.2), 'z_mundo': 0.802, 'dims': (0.1, 0.1, 0.1)},

    # Torre 2 (2 cajas + cono encima) en y=0. Es la que bloquea el camino.
    {'id': 'tower2box1', 'type': 'box', 'xy': (-0.2, 0.0), 'z_mundo': 0.602, 'dims': (0.1, 0.1, 0.1)},
    {'id': 'tower2box2', 'type': 'box', 'xy': (-0.2, 0.0), 'z_mundo': 0.702, 'dims': (0.1, 0.1, 0.1)},
    # El cono va como cilindro: es su envolvente, o sea que sobrestima el
    # obstaculo. Conservador a proposito.
    {'id': 'cone2', 'type': 'cylinder', 'xy': (-0.2, 0.0), 'z_mundo': 0.802, 'dims': (0.1, 0.05)},

    # Torre 3 (1 caja) en y=-0.2
    {'id': 'tower3', 'type': 'box', 'xy': (-0.2, -0.2), 'z_mundo': 0.602, 'dims': (0.1, 0.1, 0.1)},

    # Pelota de tenis apoyada en el borde
    {'id': 'TennisBall', 'type': 'sphere', 'xy': (0.0, -0.3), 'z_mundo': 0.572, 'dims': (0.02,)},

    # OJO: bloque_caible NO esta aca a proposito. Spawnea en z=10 y cae 9.5m
    # sobre el escritorio en x=-0.35: llega a ~14 m/s, rebota y se vuelca, asi
    # que no hay una pose de reposo que se pueda anticipar. Modelarlo parado
    # seria inventar. Cae justo al lado del objetivo (a 2.5cm), asi que para
    # una corrida limpia conviene sacarlo de mundo_escritorio.world.
]


def _primitiva(spec):
    prim = SolidPrimitive()
    if spec['type'] == 'sphere':
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(spec['dims'][0])]
    elif spec['type'] == 'cylinder':
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [float(spec['dims'][0]), float(spec['dims'][1])]
    else:
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(d) for d in spec['dims']]
    return prim


def _objeto_colision(spec):
    co = CollisionObject()
    co.header.frame_id = REF_FRAME
    co.id = spec['id']

    pose = Pose()
    pose.position.x = float(spec['xy'][0])
    pose.position.y = float(spec['xy'][1])
    pose.position.z = float(spec['z_mundo']) - ALTURA_ROBOT
    pose.orientation.w = 1.0

    co.primitives.append(_primitiva(spec))
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
CONTROLADORES = ['joint_state_broadcaster', 'joint_trajectory_controller']


class MoveAcrossDesk(Node):
    def __init__(self):
        super().__init__('move_across_desk')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._ctrl_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self._ultimo_estado = None
        self.create_subscription(JointState, '/joint_states', self._cb_joint_states, 10)

    def _cb_joint_states(self, msg):
        self._ultimo_estado = msg

    # -----------------------------------------------------------------
    def esperar_controladores(self, timeout=120.0):
        """Sin controladores activos el brazo se desploma sobre el escritorio y
        MoveIt rechaza el plan con START_STATE_IN_COLLISION. Mejor detectarlo
        aca y decirlo claro que mandar un goal condenado."""
        if not self._ctrl_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                '/controller_manager/list_controllers no aparece. El plugin '
                'gz_ros2_control no cargo: revisar la terminal de Gazebo.')
            return False

        fin = time.time() + timeout
        faltan = list(CONTROLADORES)
        while time.time() < fin:
            future = self._ctrl_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            res = future.result()
            if res is not None:
                activos = {c.name for c in res.controller if c.state == 'active'}
                faltan = [c for c in CONTROLADORES if c not in activos]
                if not faltan:
                    self.get_logger().info('Controladores activos: ' + ', '.join(CONTROLADORES))
                    return True
            self.get_logger().info(f'Esperando controladores: {faltan}')
            time.sleep(2.0)

        self.get_logger().error(
            f'Los controladores {faltan} nunca se activaron. Sin ellos nada sostiene '
            'el brazo: se cae sobre la mesa y cualquier plan arranca en colision.')
        return False

    # -----------------------------------------------------------------
    def esperar_estado(self, timeout=30.0):
        """Espera a que /joint_states traiga las seis juntas del brazo."""
        fin = time.time() + timeout
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.5)
            msg = self._ultimo_estado
            if msg is not None and all(j in msg.name for j in ARM_JOINTS):
                pos = {n: p for n, p in zip(msg.name, msg.position)}
                self.get_logger().info(
                    'Estado inicial: ' +
                    ', '.join(f'{j}={pos[j]:+.2f}' for j in ARM_JOINTS))
                return True
        self.get_logger().error(
            '/joint_states no publica las juntas del brazo. move_group va a avisar '
            '"Failed to fetch current robot state" y planificar desde un estado invalido.')
        return False

    # -----------------------------------------------------------------
    def publicar_escena(self):
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/apply_planning_scene no disponible: el planner correria a ciegas '
                'y se llevaria las torres puestas. Abortando.')
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [_objeto_colision(o) for o in OBSTACULOS]

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self._scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not (result and result.success):
            self.get_logger().error('No se pudo aplicar la escena de planificacion')
            return False

        self.get_logger().info(
            f'Escena publicada: {len(scene.world.collision_objects)} obstaculos')
        return True

    # -----------------------------------------------------------------
    def _armar_goal(self, x, y, z):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = INTENTOS
        req.allowed_planning_time = TIEMPO_PLANIFICACION
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.planner_id = 'RRTConnect'

        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = REF_FRAME
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = REF_FRAME
        pos.link_name = EE_LINK
        pos.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [TOLERANCIA_POS]
        bv.primitives.append(sp)

        destino = Pose()
        destino.position.x = float(x)
        destino.position.y = float(y)
        destino.position.z = float(z)
        destino.orientation.w = 1.0
        bv.primitive_poses.append(destino)

        pos.constraint_region = bv
        pos.weight = 1.0
        constraints.position_constraints.append(pos)

        qx, qy, qz, qw = PINZA_ABAJO
        ori = OrientationConstraint()
        ori.header.frame_id = REF_FRAME
        ori.link_name = EE_LINK
        ori.orientation.x = qx
        ori.orientation.y = qy
        ori.orientation.z = qz
        ori.orientation.w = qw
        ori.absolute_x_axis_tolerance = TOLERANCIA_ORI
        ori.absolute_y_axis_tolerance = TOLERANCIA_ORI
        ori.absolute_z_axis_tolerance = TOLERANCIA_ORI
        ori.weight = 1.0
        constraints.orientation_constraints.append(ori)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False   # planifica y ejecuta
        return goal

    # -----------------------------------------------------------------
    def ir_a(self, nombre, x, y, z):
        self.get_logger().info(f'--> "{nombre}" en ({x}, {y}, {z})')

        future = self._client.send_goal_async(self._armar_goal(x, y, z))
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'"{nombre}": goal rechazado por move_group')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        code = result.error_code.val
        if code == 1:
            self.get_logger().info(f'"{nombre}": alcanzado')
            return True

        # Los dos errores que mas aparecen cuando algo esta mal configurado.
        if code == -10:
            self.get_logger().error(
                f'"{nombre}": START_STATE_IN_COLLISION. Suele ser el SRDF sin las '
                'exclusiones de la pinza, o un obstaculo publicado encima del robot.')
        elif code == -1:
            self.get_logger().error(
                f'"{nombre}": FAILURE / sin solucion. Probar subir '
                'TIEMPO_PLANIFICACION o alejar el punto de las torres.')
        else:
            self.get_logger().error(f'"{nombre}": error_code={code}')
        return False

    # -----------------------------------------------------------------
    def recorrer(self):
        self.get_logger().info('Esperando 3s para que se estabilice')
        time.sleep(3.0)
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('/move_action no disponible: no arranco move_group')
            return False

        for nombre, x, y, z in WAYPOINTS:
            if not self.ir_a(nombre, x, y, z):
                self.get_logger().error(f'Recorrido interrumpido en "{nombre}"')
                return False

        self.get_logger().info('Recorrido completo: el brazo cruzo el escritorio')
        return True


def main():
    rclpy.init()
    node = MoveAcrossDesk()
    try:
        # El orden importa: primero que el brazo este sostenido y su estado
        # publicado, recien despues la escena y el movimiento.
        if not node.esperar_controladores():
            return
        if not node.esperar_estado():
            return
        if node.publicar_escena():
            node.recorrer()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
