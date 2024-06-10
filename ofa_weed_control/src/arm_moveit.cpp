#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("arm_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // set position goal
  move_group_interface.setPositionTarget(0.2, 0.1, 0, "eef_link");

  // create service client to set parameters
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr client =
    node->create_client<rcl_interfaces::srv::SetParametersAtomically>("/move_group/set_parameters_atomically");
  
  // set joint limits for treating left plant row
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = "robot_description_planning.joint_limits.joint2.max_position";
  parameter.value.type = 3;
  parameter.value.double_value = 0;
  request->parameters.push_back(parameter);
  parameter.name = "robot_description_planning.joint_limits.joint2.min_position";
  parameter.value.type = 3;
  parameter.value.double_value = -2.1;
  request->parameters.push_back(parameter);
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "Changed joint limits.");
  } else {
    RCLCPP_ERROR(logger, "Failed to change joint limits.");
  }

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

  // // set joint limits for treating right plant row
  request->parameters.clear();
  parameter.name = "robot_description_planning.joint_limits.joint2.max_position";
  parameter.value.type = 3;
  parameter.value.double_value = 2.1;
  request->parameters.push_back(parameter);
  parameter.name = "robot_description_planning.joint_limits.joint2.min_position";
  parameter.value.type = 3;
  parameter.value.double_value = 0;
  request->parameters.push_back(parameter);
  auto result2 = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result2) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "Changed joint limits.");
  } else {
    RCLCPP_ERROR(logger, "Failed to change joint limits.");
  }

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success2)
  {
    move_group_interface.execute(plan2);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
