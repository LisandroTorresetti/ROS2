// Ejemplo mínimo de uso de MoveIt 2 en C++ con el myCobot 320 M5.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

// =====================================================================
// GOAL - modificar estos valores
// =====================================================================
static constexpr char PLANNING_GROUP[] = "arm";
static constexpr char EE_LINK[] = "tool0";
static constexpr char REF_FRAME[] = "base_link";

static constexpr double GOAL_X  = 0.261;
static constexpr double GOAL_Y  = -0.176;
static constexpr double GOAL_Z  = 0.168;
static constexpr double GOAL_QX = 1.0;
static constexpr double GOAL_QY = 0.0;
static constexpr double GOAL_QZ = 0.0;
static constexpr double GOAL_QW = 0.0;
// =====================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);
  arm_group_interface.setPoseReferenceFrame(REF_FRAME);
  arm_group_interface.setEndEffectorLink(EE_LINK);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  auto const target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = REF_FRAME;
    msg.header.stamp = node->now();
    msg.pose.position.x = GOAL_X;
    msg.pose.position.y = GOAL_Y;
    msg.pose.position.z = GOAL_Z;
    msg.pose.orientation.x = GOAL_QX;
    msg.pose.orientation.y = GOAL_QY;
    msg.pose.orientation.z = GOAL_QZ;
    msg.pose.orientation.w = GOAL_QW;
    return msg;
  }();
  arm_group_interface.setPoseTarget(target_pose);

  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    RCLCPP_INFO(logger, "Plan encontrado, ejecutando...");
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
