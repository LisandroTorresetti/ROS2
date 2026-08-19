#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, WorkspaceParameters,
                              Constraints, JointConstraint)

# =====================================================================
# GOAL en espacio de joints - modificar estos valores (en radianes)
# =====================================================================
JOINT_GOALS = {
    'joint1': 0.5,
    'joint2': -0.5,
}
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

        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        # Joint constraints
        constraints = Constraints()
        for joint_name, value in JOINT_GOALS.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False

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
            self.get_logger().info('✅ Movimiento completado exitosamente')
        else:
            self.get_logger().error(f'❌ Error: error_code={result.error_code.val}')


def main():
    rclpy.init()
    node = MoveToPose()
    node.send_goal()
    rclpy.shutdown()


if __name__ == '__main__':
    main()