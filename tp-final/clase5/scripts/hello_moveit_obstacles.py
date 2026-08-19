#!/usr/bin/env python3
# Variante de hello_moveit.py que además de mover el myCobot 320 a una pose,
# publica un PlanningScene con obstáculos (cajas y esferas) al servicio
# /apply_planning_scene. El planner de OMPL/RRTConnect debe rodearlos para
# llegar al target.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, WorkspaceParameters,
                              Constraints, PositionConstraint,
                              OrientationConstraint, BoundingVolume,
                              PlanningScene, CollisionObject)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose, Vector3
from shape_msgs.msg import SolidPrimitive

# =====================================================================
# GOAL - modificar estos valores (deben coincidir con hello_moveit_obstacles.cpp)
# =====================================================================
PLANNING_GROUP = 'arm'
EE_LINK = 'tool0'
REF_FRAME = 'base_link'

GOAL_X = 0.061
GOAL_Y = -0.176
GOAL_Z = 0.168
GOAL_QX = 1.0
GOAL_QY = 0.0
GOAL_QZ = 0.0
GOAL_QW = 0.0
# =====================================================================

# =====================================================================
# OBSTACLES - modificar / agregar / borrar entradas libremente.
#   type: 'box'    -> dims = (sx, sy, sz)
#         'sphere' -> dims = (radius,)
#   pose está en el frame REF_FRAME, orientación identidad.
# =====================================================================
OBSTACLES = [
    {'id': 'mesa',   'type': 'box',    'pose': (0.20,  0.00, 0.02), 'dims': (0.30, 0.30, 0.02)},
    {'id': 'pilar',  'type': 'box',    'pose': (0.15, -0.10, 0.15), 'dims': (0.03, 0.03, 0.20)},
    {'id': 'pelota', 'type': 'sphere', 'pose': (0.05, -0.15, 0.20), 'dims': (0.04,)},
]
# =====================================================================


def _make_collision_object(spec):
    co = CollisionObject()
    co.header.frame_id = REF_FRAME
    co.id = spec['id']

    prim = SolidPrimitive()
    if spec['type'] == 'sphere':
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(spec['dims'][0])]
    else:
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(d) for d in spec['dims']]

    pose = Pose()
    pose.position.x = float(spec['pose'][0])
    pose.position.y = float(spec['pose'][1])
    pose.position.z = float(spec['pose'][2])
    pose.orientation.w = 1.0

    co.primitives.append(prim)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


class HelloMoveitObstacles(Node):
    def __init__(self):
        super().__init__('hello_moveit_obstacles')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def publish_obstacles(self):
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/apply_planning_scene no disponible, el planner correrá sin escena')
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [_make_collision_object(o) for o in OBSTACLES]

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self._scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        ok = bool(result and result.success)
        self.get_logger().info(
            f'Obstáculos publicados: {len(scene.world.collision_objects)} (ok={ok})')

    def send_goal(self):
        self._client.wait_for_server()
        self.get_logger().info('move_group disponible, enviando goal...')

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.planner_id = 'RRTConnect'

        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = REF_FRAME
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = REF_FRAME
        pos_constraint.link_name = EE_LINK
        pos_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]  # tolerancia 1cm
        bv.primitives.append(sp)

        goal_pose = Pose()
        goal_pose.position.x = GOAL_X
        goal_pose.position.y = GOAL_Y
        goal_pose.position.z = GOAL_Z
        bv.primitive_poses.append(goal_pose)

        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = REF_FRAME
        ori_constraint.link_name = EE_LINK
        ori_constraint.orientation.x = GOAL_QX
        ori_constraint.orientation.y = GOAL_QY
        ori_constraint.orientation.z = GOAL_QZ
        ori_constraint.orientation.w = GOAL_QW
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False  # planifica y ejecuta

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return

        self.get_logger().info('Goal aceptado, ejecutando...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Movimiento completado exitosamente')
        else:
            self.get_logger().error(f'Error: error_code={result.error_code.val}')


def main():
    rclpy.init()
    node = HelloMoveitObstacles()
    node.publish_obstacles()
    node.send_goal()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
