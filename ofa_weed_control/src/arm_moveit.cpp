#include <memory>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("arm_moveit");

  auto t1 = std::chrono::high_resolution_clock::now();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);

  // set position goal (left position)
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  q.setRPY(45 * M_PI / 180, 0, 0);
  // RCLCPP_INFO(logger, "quaternion: %f %f %f %f", q.x(), q.y(), q.z(), q.w());
  target_pose.orientation.w = q.w();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0;
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setGoalOrientationTolerance(45 * M_PI / 180);

  // plan and execute
  move_group_interface.move();

  // set position goal (right position)
  q.setRPY(-45 * M_PI / 180, 0, 0);
  // RCLCPP_INFO(logger, "quaternion: %f %f %f %f", q.x(), q.y(), q.z(), q.w());
  target_pose.orientation.w = q.w();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0;
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setGoalOrientationTolerance(45 * M_PI / 180);

  // plan and execute
  move_group_interface.move();
  
  auto t2 = std::chrono::high_resolution_clock::now();
  auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  RCLCPP_INFO(logger, "%ld ms", ms_int.count());

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
