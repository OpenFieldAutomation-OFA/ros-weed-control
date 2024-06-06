#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "arm_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("arm_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = -0.4;
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose, "eef_link");

  auto pos = move_group_interface.getPoseTarget("eef_link");
  RCLCPP_INFO(logger, "position: %f, %f, %f",
  pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);

  auto test = move_group_interface.getPoseReferenceFrame();
  RCLCPP_INFO(logger, "reference frame: %s", test.c_str());

  auto test2 = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(logger, "end effector: %s", test2.c_str());

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
