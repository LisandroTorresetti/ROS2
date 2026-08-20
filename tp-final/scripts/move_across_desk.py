#!/usr/bin/env python3
# Mueve el myCobot 320 con pinza en shalom entre los tres pilares que estan
# apoyados sobre el escritorio de mundo_escritorio.world, sin tocarlos.
#
# Los pilares estan en fila sobre x=-0.22, separados 0.22 en y, con alturas
# distintas (0.20 / 0.28 / 0.12). Entre uno y otro queda un hueco de 0.16.
#
# El recorrido:
#   1) se para arriba del hueco derecho
#   2) baja por el hueco, entre el pilar alto y el bajo
#   3) vuelve a subir
#   4) cruza por encima del pilar alto (su tope esta en z=+0.279)
#   5) queda sobre el hueco izquierdo
#
#
# Uso:
#   Terminal 1:  ros2 launch clase5 mycobot_launch.py
#   Terminal 2:  ros2 launch clase5 move_across_desk.launch.py

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
# Cada waypoint es (nombre, x, y, z), en base_link.
#
# =====================================================================
WAYPOINTS = [
    ('sobre_hueco_der',  -0.22, -0.17, 0.32),
    ('hueco_derecho',    -0.22, -0.17, 0.12),
    ('salir_der',        -0.22, -0.17, 0.32),   
    ('sobre_la_alta',    -0.22, -0.06, 0.40),
    ('sobre_hueco_izq',  -0.22,  0.05, 0.32),
]


TOLERANCIA_POS = 0.01
TOLERANCIA_ORI_XY = 0.25
TOLERANCIA_ORI_Z = 3.15
TIEMPO_PLANIFICACION = 15.0
INTENTOS = 10

# =====================================================================
# OBSTACULOS
# Los tres pilares de mundo_escritorio.world son <static>true</static>, o sea
# que no caen ni se acomodan: la pose del SDF ya es la definitiva y esta lista
# coincide exactamente con lo que ve Gazebo.
#
# 'z_mundo' es la altura del CENTRO en el frame del mundo; se le resta
# ALTURA_ROBOT para pasarla a base_link. La tapa del escritorio esta en
# z_mundo=0.552, asi que un pilar de alto h tiene su centro en 0.552 + h/2.
# =====================================================================
OBSTACULOS = [
    # La tapa del escritorio. Se modela 1.5cm por debajo de base_link para no
    # solaparse con la malla de colision de base_link, que apoya justo encima.
    {'id': 'escritorio', 'type': 'box', 'xy': (-0.214, 0.0),
     'z_mundo': 0.518, 'dims': (0.490, 0.844, 0.040)},

    # Pilar alto: al frente del robot. Tope en base_link z=+0.279.
    {'id': 'torre_alta', 'type': 'box', 'xy': (-0.22, -0.06),
     'z_mundo': 0.692, 'dims': (0.06, 0.06, 0.28)},

    # Pilar medio: lado +y. Tope en base_link z=+0.199.
    {'id': 'torre_media', 'type': 'box', 'xy': (-0.22, 0.16),
     'z_mundo': 0.652, 'dims': (0.06, 0.06, 0.20)},

    # Pilar bajo: lado -y. Tope en base_link z=+0.119.
    {'id': 'torre_baja', 'type': 'box', 'xy': (-0.22, -0.28),
     'z_mundo': 0.612, 'dims': (0.06, 0.06, 0.12)},
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
        """Valida que las precondiciones funcione, en caso de no hacerlo falla"""
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
        # El nombre tiene que coincidir con una entrada de planner_configs de
        # ompl_planning.yaml. Con 'RRTConnect' pelado no matchea nada y OMPL
        # descarta el pedido y usa su default.
        req.planner_id = 'RRTConnectkConfigDefault'

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
        ori.absolute_x_axis_tolerance = TOLERANCIA_ORI_XY
        ori.absolute_y_axis_tolerance = TOLERANCIA_ORI_XY
        ori.absolute_z_axis_tolerance = TOLERANCIA_ORI_Z
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

        # Error handler
        if code == -10:
            self.get_logger().error(
                f'"{nombre}": START_STATE_IN_COLLISION')
        elif code == -1:
            self.get_logger().error(
                f'"{nombre}": PLANNING_FAILED')
        elif code == 99999:
            self.get_logger().error(
                f'"{nombre}": FAILURE')
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
