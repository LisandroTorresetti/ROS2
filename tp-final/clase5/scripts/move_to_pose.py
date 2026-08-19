#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, WorkspaceParameters,
                              Constraints, PositionConstraint,
                              OrientationConstraint, BoundingVolume)
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

# =====================================================================
# GOAL - modificar estos valores
# =====================================================================
GOAL_X = 0.2
GOAL_Y = 0.3
GOAL_Z = 0.03
GOAL_QX = 0.0
GOAL_QY = 0.0
GOAL_QZ = 0.841
GOAL_QW = 0.540
# =====================================================================

class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self):
        self._client.wait_for_server()
        self.get_logger().info('move_group disponible, enviando goal...')

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = 'pendulum'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.planner_id = 'RRTConnect'

        # Workspace
        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        # Goal constraints
        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = 'tool0'
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

        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'
        ori_constraint.link_name = 'tool0'
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
        goal.planning_options.plan_only = True  # planifica y ejecuta

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Resultado: error_code={result.error_code.val}')


def main():
    rclpy.init()
    node = MoveToPose()
    node.send_goal()
    rclpy.shutdown()


if __name__ == '__main__':
    main()