#include <cstdio>
#include <k4a/k4a.hpp>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <thread>

// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/common/centroid.h>
// #include <pcl/ml/kmeans.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ofa_interfaces/action/weed_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using WeedControl = ofa_interfaces::action::WeedControl;
using GoalHandleWeedControl = rclcpp_action::ServerGoalHandle<WeedControl>;


class WeedControlActionServer : public rclcpp::Node
{
public:
  WeedControlActionServer()
  : Node("weed_control_action_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // this->declare_parameter("cluster_distance", 10.0);
    // this->declare_parameter("scale_z", 2.0);
    // this->declare_parameter("split_size", 5000);
    // this->declare_parameter("exg_threshold", -25.0);
    // this->declare_parameter("nir_threshold", 80.0);
    // this->declare_parameter("save_images", false);
    // this->declare_parameter("dilation_size", 20);

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<WeedControl>(
      this,
      "weed_control",
      std::bind(&WeedControlActionServer::handle_goal, this, _1, _2),
      std::bind(&WeedControlActionServer::handle_cancel, this, _1),
      std::bind(&WeedControlActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<WeedControl>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WeedControl::Goal> /* goal */)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWeedControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWeedControl> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WeedControlActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWeedControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<WeedControl::Result>();

    // get parameters
    double cluster_distance = this->get_parameter("cluster_distance").as_double();  // tolerance in mm for point cloud clustering
    double scale_z = this->get_parameter("scale_z").as_double();;  // scales depth value in point cloud
    int split_size = this->get_parameter("split_size").as_int();  // big plant point cloud split
    double exg_threshold = this->get_parameter("exg_threshold").as_double();;  // excess green threshold
    double nir_threshold = this->get_parameter("nir_threshold").as_double();  // nir threshold
    bool save_images = this->get_parameter("save_images").as_bool();  // dilation kernel size
    int dilation_size = this->get_parameter("dilation_size").as_int();;  // dilation kernel size

    RCLCPP_INFO(this->get_logger(), "c: %f, e: %f, n: %f", cluster_distance, exg_threshold, nir_threshold);

    moveit::planning_interface::MoveGroupInterface move_group(std::make_shared<rclcpp::Node>(this->get_name()), "arm");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);


    std::vector<double> joint_values = {-0.028, -0.073, 2.040};
    move_group.setJointValueTarget(joint_values);
    move_group.move();

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeedControlActionServer>());
  rclcpp::shutdown();
  return 0;
}