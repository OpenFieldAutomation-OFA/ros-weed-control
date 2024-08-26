#include <memory>
#include <cmath>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/msg/collision_object.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_moveit");

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("arm_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    this->get_parameters();
  }

  std::vector<double> get_position(int i)
  {
    return {motor1_[i], motor2_[i], motor3_[i]};
  }

  std::size_t get_length()
  {
    return motor1_.size();
  }

private:
  void get_parameters()
  {
    this->get_parameter("motor1", motor1_);
    this->get_parameter("motor2", motor2_);
    this->get_parameter("motor3", motor3_);
  }

private:
  std::vector<double> motor1_;
  std::vector<double> motor2_;
  std::vector<double> motor3_;
};


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<MinimalParam>();

  // Create a ROS LOGGER
  // auto const LOGGER = rclcpp::get_logger("arm_moveit");

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  // Create PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create ground collision box
  // moveit_msgs::msg::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();
  // collision_object.id = "ground";
  // shape_msgs::msg::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 2.0;
  // primitive.dimensions[primitive.BOX_Y] = 2.0;
  // primitive.dimensions[primitive.BOX_Z] = 1.0;
  // geometry_msgs::msg::Pose ground_pose;
  // ground_pose.orientation.w = 1.0;
  // ground_pose.position.x = 0.0;
  // ground_pose.position.y = 0.0;
  // ground_pose.position.z = -0.5;
  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(ground_pose);
  // collision_object.operation = collision_object.ADD;
  // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  // RCLCPP_INFO(LOGGER, "Add ground into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);


  // auto t1 = std::chrono::high_resolution_clock::now();
  // move_group.getCurrentState(10);
  
  // auto t2 = std::chrono::high_resolution_clock::now();
  // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // RCLCPP_INFO(LOGGER, "%ld ms", ms_int.count());

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  // move_group.setPlanningTime(0.2);  // planning failed if it takes longer

  // for (std::size_t i = 0; i < node->get_length(); i++)
  // {
  //   std::vector<double> joint_values = node->get_position(i);
  //   move_group.setJointValueTarget(joint_values);
  //   move_group.move();
  //   rclcpp::sleep_for(std::chrono::milliseconds(3000));
  // }

  // set position goal (left position)
  // move_group.setPoseReferenceFrame("camera_color_frame");

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.22;
  target_pose.position.y = -0.05;
  target_pose.position.z = 0.05;
  move_group.setGoalOrientationTolerance(360 * M_PI / 180);
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // move_group.setPositionTarget(0.224756, -0.3, 0.05);
  // move_group.move();

  // move_group.setPositionTarget(0.224756, 0.3, 0.05);
  // move_group.move();

  move_group.setNamedTarget("home");
  move_group.move();

  // move_group.setPositionTarget(-0.384220, 0.095513, 0.118338);
  
  // move_group.setPoseTarget(target_pose);
  // move_group.setGoalOrientationTolerance(45 * M_PI / 180);

  // plan and execute

  // set position goal (right position)
  // q.setRPY(-90 * M_PI / 180, 0, 0);
  // // RCLCPP_INFO(LOGGER, "quaternion: %f %f %f %f", q.x(), q.y(), q.z(), q.w());
  // target_pose.orientation.w = q.w();
  // target_pose.orientation.x = q.x();
  // target_pose.orientation.y = q.y();
  // target_pose.orientation.z = q.z();
  // target_pose.position.x = 0.235;
  // target_pose.position.y = -0.02;
  // target_pose.position.z = 0.064 + 0.02;
  // // move_group.setPositionTarget(0.1, 0.0, 0.6);
  // move_group.setPoseTarget(target_pose);
  // move_group.setGoalOrientationTolerance(50 * M_PI / 180);

  // // plan and execute
  // move_group.move();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
