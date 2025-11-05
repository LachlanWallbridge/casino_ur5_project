#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_move_demo");

  // Initialize MoveGroupInterface AFTER the node exists
  MoveGroupInterface move_group(node, "ur_manipulator");

  RCLCPP_INFO(node->get_logger(), "Starting Cartesian move demo...");

//   geometry_msgs::msg::PoseStamped start_pose = move_group.getCurrentPose();
   std::vector<geometry_msgs::msg::Pose> waypoints;

//   geometry_msgs::msg::Pose target_pose = start_pose.pose;
//   target_pose.position.x += 0.10;  // Move 10 cm along X
// 0.491, 0.130, 0.531
// 0.708, -0.706, 0.000, 0.000

	geometry_msgs::msg::Pose target_pose {};
	target_pose.position.x = 0.491 + 0.05;
	target_pose.position.y = 0.130;
	target_pose.position.z = 0.531 - 0.4;

	target_pose.orientation.x = 0.708;
	target_pose.orientation.y = -0.706;
	target_pose.orientation.z = 0.000;
	target_pose.orientation.w = 0.000;

  waypoints.push_back(target_pose);

  RCLCPP_INFO(
    node->get_logger(),
    "Target Pose:\n  Position -> x: %.3f, y: %.3f, z: %.3f\n  Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w
);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node->get_logger(), "Path fraction: %.2f", fraction);

  if (fraction > 0)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Cartesian move executed successfully!");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Cartesian path planning incomplete!");
  }

  rclcpp::shutdown();
  return 0;
}
