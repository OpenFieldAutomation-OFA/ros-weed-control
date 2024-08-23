#include <cstdio>
#include <cstdint>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ofa_interfaces/action/weed_control.hpp>
#include <ofa_interfaces/action/cluster_classify.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "k4a/k4a.hpp"

// Function to send command to the Arduino
void send_command(int arduino_port_, const std::string& command)
{
  write(arduino_port_, command.c_str(), command.length());
  write(arduino_port_, "\n", 1); // Send newline character
}

class WeedControlActionServer : public rclcpp::Node
{
public:
  using WeedControl = ofa_interfaces::action::WeedControl;
  using GoalHandleWeedControl = rclcpp_action::ServerGoalHandle<WeedControl>;

  using ClusterClassify = ofa_interfaces::action::ClusterClassify;
  using GoalHandleClusterClassify = rclcpp_action::ClientGoalHandle<ClusterClassify>;
  
  WeedControlActionServer()
  : Node("weed_control_action_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // tf2 listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    using namespace std::placeholders;

    save_runs_ = this->get_parameter("save_runs").as_bool();
    if (save_runs_)
    {
      auto now = std::chrono::system_clock::now();
      auto now_time_t = std::chrono::system_clock::to_time_t(now);
      auto now_tm = *std::localtime(&now_time_t);
      std::ostringstream oss;
      oss << std::put_time(&now_tm, "/%Y%m%d_%H%M%S/");
      run_folder_ = this->get_parameter("runs_folder").as_string() + oss.str();
      std::filesystem::create_directory(run_folder_);
    }

    this->init_action_server();
    
    use_mock_hardware_ = this->get_parameter("use_mock_hardware").as_bool();
    if (use_mock_hardware_)
    {
      RCLCPP_INFO(this->get_logger(), "Not using real hardware");

      // manually initialize camera calibration
      k4a_calibration_camera_t color_camera_calibration;
      k4a_calibration_intrinsics_t intrinsics;
      intrinsics.type = K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY;
      intrinsics.parameter_count = 14;
      intrinsics.parameters = {1899.003296, 1080.935913, 2240.591797, 2239.406738, 0.079749, -0.110377, 0.046372, 0, 0, 0, 0, 0, 0.000473, -0.000354, 0};
      color_camera_calibration.intrinsics = intrinsics;
      color_camera_calibration.resolution_width = 3840;
      color_camera_calibration.resolution_height = 2160;
      color_camera_calibration.metric_radius = 1.7;
      calibration_.color_camera_calibration = color_camera_calibration;
      calibration_.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    }
    else
    {
      if (!this->init_arduino())
      {
        rclcpp::shutdown();
      }
      this->init_camera();    
    }

    this->client_ptr_ = rclcpp_action::create_client<ClusterClassify>(
      this,
      "cluster_classify");

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

  ~WeedControlActionServer()
  {
    running_ = false;
    capture_thread_.join();
    device_.stop_cameras();

    close(arduino_port_);
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp_action::Server<WeedControl>::SharedPtr action_server_;
  bool is_busy_ = false;

  rclcpp_action::Client<ClusterClassify>::SharedPtr client_ptr_;

  // capture thread
  std::atomic<bool> running_;
  std::thread capture_thread_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;

  // arduino
  int arduino_port_;

  // camera
  k4a::device device_;
  k4a::calibration calibration_;
  k4a::transformation transformation_;
  k4a::capture capture_;

  // folder for saving images and measurements
  std::string run_folder_;
  int process_count_ = 0;
  bool save_runs_;

  bool use_mock_hardware_;

  bool auto_exposure_;

  void init_action_server()
  {
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const WeedControl::Goal> /* goal */)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      if (is_busy_)
      {
        RCLCPP_INFO(this->get_logger(), "Rejecting goal, server is busy");
        return rclcpp_action::GoalResponse::REJECT;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Accepting goal");
        is_busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    };

    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleWeedControl> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleWeedControl> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<WeedControl>(
      this,
      "weed_control",
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  bool init_arduino()
  {
    const char* arduino = "/dev/ttyACM0";
    // LEDs will flash white for some reason when opening serial communication
    arduino_port_ = open(arduino, O_RDWR);
    if (arduino_port_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error %i from opening Arduino port: %s", errno, strerror(errno));
      return false;
    }
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(arduino_port_, &tty) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(arduino_port_);
      return false;
    }
    cfsetispeed(&tty, B9600);  // set baud rate
    cfsetospeed(&tty, B9600);  // set baud rate
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read and ignore control lines
    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes
    tty.c_oflag &= ~OPOST; // Disable output processing
    tty.c_oflag &= ~ONLCR; // Disable conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
    if (tcsetattr(arduino_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error %i from tcsetattr: %s", errno, strerror(errno));
        close(arduino_port_);
      return false;
    }
    
    // turn off leds and set brightness to maximum
    send_command(arduino_port_, "OFF");
    send_command(arduino_port_, "BRIGHTNESS 255");

    return true;
  }

  void init_camera()
  {
    auto_exposure_ = this->get_parameter("auto_exposure").as_bool();
    // open the first plugged in Kinect device
    device_ = k4a::device::open(K4A_DEVICE_DEFAULT);

    // set color controls (mostly same as camera defaults)
    if (auto_exposure_)
    {
      device_.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
        K4A_COLOR_CONTROL_MODE_AUTO, 0);
    }
    else
    {
      device_.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
        K4A_COLOR_CONTROL_MODE_MANUAL, 20000);
    }
    device_.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS,
      K4A_COLOR_CONTROL_MODE_MANUAL, 10);
    device_.set_color_control(K4A_COLOR_CONTROL_CONTRAST,
      K4A_COLOR_CONTROL_MODE_MANUAL, 50);
    device_.set_color_control(K4A_COLOR_CONTROL_SATURATION,
      K4A_COLOR_CONTROL_MODE_MANUAL, 64);
    device_.set_color_control(K4A_COLOR_CONTROL_SHARPNESS,
      K4A_COLOR_CONTROL_MODE_MANUAL, 24);
    device_.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE,
      K4A_COLOR_CONTROL_MODE_MANUAL, 6500);
    device_.set_color_control(K4A_COLOR_CONTROL_GAIN,
      K4A_COLOR_CONTROL_MODE_MANUAL, 0);
    device_.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
      K4A_COLOR_CONTROL_MODE_MANUAL, 1);

    k4a_color_resolution_t color_resolution = static_cast<k4a_color_resolution_t>(
      this->get_parameter("color_resolution").as_int());

    // define config
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;  // BGRA32 leads to problems
    config.color_resolution = color_resolution;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    config.synchronized_images_only = true;

    calibration_ = device_.get_calibration(config.depth_mode, config.color_resolution);
    // int resolution = calibration_.color_resolution;
    // int height = calibration_.color_camera_calibration.resolution_height;
    // int width = calibration_.color_camera_calibration.resolution_width;
    // float metric_radius = calibration_.color_camera_calibration.metric_radius;

    // float cx = calibration_.color_camera_calibration.extrinsics.rotation[0];
    // float cy = calibration_.color_camera_calibration.extrinsics.rotation[1];
    // float fx = calibration_.color_camera_calibration.extrinsics.rotation[2];
    // float fy = calibration_.color_camera_calibration.extrinsics.rotation[3];
    // float k1 = calibration_.color_camera_calibration.extrinsics.rotation[4];
    // float k2 = calibration_.color_camera_calibration.extrinsics.rotation[5];
    // float k3 = calibration_.color_camera_calibration.extrinsics.rotation[6];
    // float k4 = calibration_.color_camera_calibration.extrinsics.rotation[7];
    // float k5 = calibration_.color_camera_calibration.extrinsics.rotation[8];
    // float k6 = calibration_.color_camera_calibration.extrinsics.rotation[9];
    // float codx = calibration_.color_camera_calibration.extrinsics.translation[0];
    // float cody = calibration_.color_camera_calibration.extrinsics.translation[1];
    // float p2 = calibration_.color_camera_calibration.extrinsics.translation[2];
    // RCLCPP_INFO(this->get_logger(),
    //   "camera calibration: %d, %d, %d, %f. extrincis: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //   resolution, height, width, metric_radius,
    //   cx, cy, fx, fy, k1, k2, k3, k4, k5, k6, codx, cody, p2
    // );
    transformation_ = k4a::transformation(calibration_);

    device_.start_cameras(&config);

    running_ = true;
    capture_thread_ = std::thread(&WeedControlActionServer::capture_frames, this);
  }

  void capture_frames() {
    bool publish_color = this->get_parameter("publish_color").as_bool();
    if (publish_color)
    {
      color_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
    }

    // Capture frames in a loop until 'running_' is set to false
    int frames = 0;
    while (running_) {
      frames++;
      if (!device_.get_capture(&capture_, std::chrono::milliseconds(1000)))
      {
        RCLCPP_ERROR(this->get_logger(), "Could not get camera capture.");
      }
      if (publish_color && frames == 15)
      {
        // get images
        k4a::image color_image = capture_.get_color_image();

        // publish color image
        std::vector<std::uint8_t> color_buffer(color_image.get_buffer(), color_image.get_buffer() + color_image.get_size());
        cv::Mat color_mat = cv::imdecode(color_buffer, cv::IMREAD_ANYCOLOR);
        sensor_msgs::msg::Image ros_image;
        ros_image.header.stamp = this->now();
        ros_image.header.frame_id = "camera_color_frame";
        ros_image.height = color_mat.rows;
        ros_image.width = color_mat.cols;
        ros_image.encoding = "bgr8";
        ros_image.is_bigendian = false;
        ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(color_mat.step);
        ros_image.data.assign(color_mat.datastart, color_mat.dataend);
        color_publisher_->publish(ros_image);

        frames = 0;
      }
    }
  }

  void execute(const std::shared_ptr<GoalHandleWeedControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<WeedControl::Result>();
    auto feedback = std::make_shared<WeedControl::Feedback>();
    feedback->percent_complete = 0;

    // init movegroup
    moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "arm");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    // move_group.setPlanningTime(0.2);
    moveit::core::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // move to first position
    move_group.setJointValueTarget({-0.3, 1.3, 0.0});
    error_code = move_group.move();
    if (error_code)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to first photo position.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not reach first photo position: %s",
        moveit::core::error_code_to_string(error_code).c_str());
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // get transform of position 1
    geometry_msgs::msg::TransformStamped t1;
    try {
      t1 = tf_buffer_->lookupTransform(
        "world", "camera_color_frame",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s", ex.what());
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // start weed detection of position 1
    std::shared_future<GoalHandleClusterClassify::WrappedResult> result_future_1;
    if (!process_image("back", result_future_1))
    {
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // move to second position
    move_group.setJointValueTarget({-0.75, 1.3, 0.0});
    error_code = move_group.move();
    if (error_code)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to second photo position.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not reach second photo position: %s",
        moveit::core::error_code_to_string(error_code).c_str());
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // get transform of position 2
    geometry_msgs::msg::TransformStamped t2;
    try {
      t2 = tf_buffer_->lookupTransform(
        "world", "camera_color_frame",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s", ex.what());
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // start weed detection of position 2
    std::shared_future<GoalHandleClusterClassify::WrappedResult> result_future_2;
    if (!process_image("front", result_future_2))
    {
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }

    // move to home
    move_group.setNamedTarget("home");
    move_group.move();

    // wait for weed detection to finish
    RCLCPP_INFO(this->get_logger(), "Waiting for weed detection");
    auto result_1 = result_future_1.get();
    if (result_1.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Weed detection back failed");
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }
    auto result_2 = result_future_2.get();
    if (result_2.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Weed detection front failed");
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Weed detection done");

    // limit workspace to reachable positions
    double x_lower = this->get_parameter("x_lower").as_double();
    double x_upper = this->get_parameter("x_upper").as_double();
    double y_lower = this->get_parameter("y_lower").as_double();
    double y_upper = this->get_parameter("y_upper").as_double();
    double z_lower = this->get_parameter("z_lower").as_double();
    double z_upper = this->get_parameter("z_upper").as_double();

    /*
    Offsets in x, y, and z-direction
    Ideally the URDF would describe our setup perfectly, but practically it's easier to tune the offsets as parameters
    */ 
    double x_off = this->get_parameter("x_off").as_double();
    double y_off = this->get_parameter("y_off").as_double();
    double z_off = this->get_parameter("z_off").as_double();
    double collision_off = this->get_parameter("collision_off").as_double();
    int wait_shock = this->get_parameter("wait_shock").as_int();

    // get returned positions from weed detection
    auto positions_1 = result_1.result->positions;
    auto positions_2 = result_2.result->positions;

     // transform positions from camera to world frame
    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_color_frame";
    geometry_msgs::msg::PointStamped point_in_world;
    std::vector<std::vector<double>> world_positions_1;
    for (const geometry_msgs::msg::Vector3 & position : positions_1)
    {
      point_in_camera.point.x = position.x / 1000;
      point_in_camera.point.y = position.y / 1000;
      point_in_camera.point.z = position.z / 1000;
      RCLCPP_INFO(this->get_logger(), "Centroid in camera frame: [%f, %f, %f]",
        point_in_camera.point.x,
        point_in_camera.point.y,
        point_in_camera.point.z);
      tf2::doTransform(point_in_camera, point_in_world, t1);
      RCLCPP_INFO(this->get_logger(), "Centroid in world frame: [%f, %f, %f]",
        point_in_world.point.x,
        point_in_world.point.y,
        point_in_world.point.z);
      
      if (point_in_world.point.x + x_off < x_lower || point_in_world.point.x + x_off > x_upper) continue;
      if (point_in_world.point.y + y_off < y_lower || point_in_world.point.y + y_off > y_upper) continue;
      if (point_in_world.point.z + z_off < z_lower || point_in_world.point.z + z_off > z_upper) continue;
      world_positions_1.push_back(
        {
          point_in_world.point.x + x_off,
          point_in_world.point.y + y_off,
          point_in_world.point.z + z_off
        }
      );
    }
    std::vector<std::vector<double>> world_positions_2;
    for (const geometry_msgs::msg::Vector3 & position : positions_2)
    {
      point_in_camera.point.x = position.x / 1000;
      point_in_camera.point.y = position.y / 1000;
      point_in_camera.point.z = position.z / 1000;
      RCLCPP_INFO(this->get_logger(), "Centroid in camera frame: [%f, %f, %f]",
        point_in_camera.point.x,
        point_in_camera.point.y,
        point_in_camera.point.z);
      tf2::doTransform(point_in_camera, point_in_world, t2);
      RCLCPP_INFO(this->get_logger(), "Centroid in world frame: [%f, %f, %f]",
        point_in_world.point.x,
        point_in_world.point.y,
        point_in_world.point.z);
      
      if (point_in_world.point.x + x_off < x_lower || point_in_world.point.x + x_off > x_upper) continue;
      if (point_in_world.point.y + y_off < y_lower || point_in_world.point.y + y_off > y_upper) continue;
      if (point_in_world.point.z + z_off < z_lower || point_in_world.point.z + z_off > z_upper) continue;
      world_positions_1.push_back(
        {
          point_in_world.point.x + x_off,
          point_in_world.point.y + y_off,
          point_in_world.point.z + z_off
        }
      );
    }

    // stack positions
    std::vector<std::vector<double>> all_positions = world_positions_1;
    all_positions.insert(all_positions.end(), world_positions_2.begin(), world_positions_2.end());

    // sort world positions
    std::vector<std::vector<double>> positive_y;
    std::vector<std::vector<double>> negative_y;
    for (const std::vector<double> & position : all_positions) {
      if (position[1] > 0.0) {
        positive_y.push_back(position);
      } else {
        negative_y.push_back(position);
      }
    }
    std::sort(positive_y.begin(), positive_y.end(), 
      [](const std::vector<double>& a, const std::vector<double>& b) {
          return a[0] > b[0]; // Sort by x-coordinate in increasing order
      });
    std::sort(negative_y.begin(), negative_y.end(), 
      [](const std::vector<double>& a, const std::vector<double>& b) {
          return a[0] < b[0]; // Sort by x-coordinate in decreasing order
      });
    std::vector<std::vector<double>> sorted_positions = negative_y;
    sorted_positions.insert(sorted_positions.end(), positive_y.begin(), positive_y.end());

    // move to targets
    bool first = true;
    geometry_msgs::msg::Pose target_pose;
    move_group.setGoalOrientationTolerance(90 * M_PI / 180);
    float percent_increase = 100.0 / sorted_positions.size();
    for (const std::vector<double> & position : sorted_positions)
    {
      if (goal_handle->is_canceling()) {
        move_group.setNamedTarget("home");
        move_group.move();
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        is_busy_ = false;
        return;
      }
      geometry_msgs::msg::Pose old_target_pose = target_pose;
      
      target_pose.position.x = position[0];
      target_pose.position.y = position[1];
      target_pose.position.z = position[2];

      RCLCPP_INFO(this->get_logger(), "Target position: [%f, %f, %f]",
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);
      
      if (first || (old_target_pose.position.y <= 0 && target_pose.position.y > 0))
      {
        RCLCPP_INFO(this->get_logger(), "Moving to first plant");
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
        tf2::Quaternion q;
        // make sure arm is correctly positioned
        if (position[1] < 0)
        {
          q.setRPY(-90 * M_PI / 180, 0, 0);
        }
        else
        {
          q.setRPY(90 * M_PI / 180, 0, 0);
        }
        // RCLCPP_INFO(LOGGER, "quaternion: %f %f %f %f", q.x(), q.y(), q.z(), q.w());
        target_pose.orientation.w = q.w();
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        move_group.setPoseTarget(target_pose);
        error_code = move_group.plan(plan);

        if(error_code)
        {
          error_code = move_group.execute(plan);
          if (!error_code)
          {
            RCLCPP_ERROR(this->get_logger(), "Execution failed: %s",
              moveit::core::error_code_to_string(error_code).c_str());
            goal_handle->abort(result);
            is_busy_ = false;
            return;
          }
          first = false;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Planning failed: %s",
            moveit::core::error_code_to_string(error_code).c_str());
        }
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(1.0);
      }
      else
      {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        old_target_pose.position.z += collision_off;
        waypoints.push_back(old_target_pose);
        waypoints.push_back(target_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(this->get_logger(), "Fraction achieved: %f", fraction * 100.0);
        if (fraction > 0.99)
        {
          error_code = move_group.execute(trajectory);
          if (!error_code)
          {
            RCLCPP_ERROR(this->get_logger(), "Execution failed: %s",
              moveit::core::error_code_to_string(error_code).c_str());
            goal_handle->abort(result);
            is_busy_ = false;
            return;
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Planning failed: Fraction achieved %f", fraction * 100.0);
        }
      }

      // shock
      rclcpp::sleep_for(std::chrono::milliseconds(wait_shock));
      feedback->percent_complete += percent_increase;
      goal_handle->publish_feedback(feedback);
    }

    // move to home
    move_group.setNamedTarget("home");
    move_group.move();

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    is_busy_ = false;
  }


  bool process_image(std::string pos,
    std::shared_future<GoalHandleClusterClassify::WrappedResult> & result_future)
  {
    // get parameters
    const double exg_threshold = this->get_parameter("exg_threshold").as_double();;  // excess green threshold
    const double nir_threshold = this->get_parameter("nir_threshold").as_double();  // nir threshold
    const double ndvi_threshold = this->get_parameter("ndvi_threshold").as_double();  // nir threshold
    
    const bool old_version = this->get_parameter("old_version").as_bool();
    const int dilation_size = this->get_parameter("dilation_size").as_int();;  // dilation kernel size

    std::chrono::_V2::system_clock::time_point t1;
    std::chrono::_V2::system_clock::time_point t2;
    std::chrono::milliseconds ms_int;

    std::string log_text = "";

    cv::Mat depth_mat;
    cv::Mat ir_mat;
    cv::Mat color_mat;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ofa_main");

    if (!use_mock_hardware_)
    {
      // turn on LEDs
      send_command(arduino_port_, "COLOR 255,255,128");

      if (auto_exposure_)
      {
        // wait for auto exposure to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      // get images (capture done in seperate thread)
      k4a::image depth_image = capture_.get_depth_image();
      k4a::image color_image = capture_.get_color_image();
      k4a::image ir_image = capture_.get_ir_image();
      send_command(arduino_port_, "OFF");

      t1 = std::chrono::high_resolution_clock::now();
      
      // transform depth and ir image to camera viewpoint
      ir_image = k4a::image::create_from_buffer(
        K4A_IMAGE_FORMAT_CUSTOM16,
        ir_image.get_width_pixels(),
        ir_image.get_height_pixels(),
        ir_image.get_stride_bytes(),
        ir_image.get_buffer(),
        ir_image.get_size(),
        [](void *, void *) {},
        NULL
      );
      std::pair<k4a::image, k4a::image> transformed = transformation_.depth_image_to_color_camera_custom(
        depth_image,
        ir_image,
        K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
        0
      );
      depth_image = transformed.first;
      ir_image = transformed.second;

      t2 = std::chrono::high_resolution_clock::now();
      ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      log_text += "Transform images: " + std::to_string(ms_int.count()) + " ms\n";

      // decode JPG
      t1 = std::chrono::high_resolution_clock::now();
      std::vector<std::uint8_t> color_buffer(color_image.get_buffer(), color_image.get_buffer() + color_image.get_size());
      color_mat = cv::imdecode(color_buffer, cv::IMREAD_ANYCOLOR);
      t2 = std::chrono::high_resolution_clock::now();
      ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      log_text += "Decode JPEG: " + std::to_string(ms_int.count()) + " ms\n";

      // convert depth and ir to opencv
      t1 = std::chrono::high_resolution_clock::now();
      depth_mat = cv::Mat(
        depth_image.get_height_pixels(),
        depth_image.get_width_pixels(),
        CV_16UC1,
        reinterpret_cast<uint16_t*>(depth_image.get_buffer())
      );
      ir_mat = cv::Mat(
        ir_image.get_height_pixels(),
        ir_image.get_width_pixels(),
        CV_16UC1,
        reinterpret_cast<uint16_t*>(ir_image.get_buffer())
      );

      t2 = std::chrono::high_resolution_clock::now();
      ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      log_text += "Convert to OpenCV: " + std::to_string(ms_int.count()) + " ms\n";
    }
    else
    {
      std::string mock_images = this->get_parameter("mock_images").as_string();
      depth_mat = cv::imread(package_share_directory + "/images/" + mock_images + "/" + pos + "/depth16.png", cv::IMREAD_UNCHANGED);
      ir_mat = cv::imread(package_share_directory + "/images/" + mock_images + "/" + pos + "/ir16.png", cv::IMREAD_UNCHANGED);
      color_mat = cv::imread(package_share_directory + "/images/" + mock_images + "/" + pos + "/color.png", cv::IMREAD_COLOR);
    }

    cv::Mat ir_corrected;
    ir_mat.convertTo(ir_corrected, CV_32F);
    bool illumination_correction = this->get_parameter("illumination_correction").as_bool();
    if(illumination_correction)
    {
      cv::Mat ill_corr = cv::imread(package_share_directory + "/images/illumination.png", cv::IMREAD_GRAYSCALE);
      ill_corr.convertTo(ill_corr, CV_32F);
      ir_corrected = ir_corrected / ill_corr * 150;
    }

    cv::threshold(ir_corrected, ir_corrected, 2000, 2000, cv::THRESH_TRUNC); // remove outliers (reflections, saturated pixels)
    ir_corrected.convertTo(ir_corrected, CV_8UC1, 255.0 / 2000.0);

    // calculate ExG
    t1 = std::chrono::high_resolution_clock::now();
    std::vector<cv::Mat> bgr_channels;
    cv::split(color_mat, bgr_channels);
    cv::Mat blue_channel, green_channel, red_channel;
    bgr_channels[0].convertTo(blue_channel, CV_32F);
    bgr_channels[1].convertTo(green_channel, CV_32F);
    bgr_channels[2].convertTo(red_channel, CV_32F);
    cv::Mat exg = 2.0 * green_channel - red_channel - blue_channel;

    // remove some noise
    cv::GaussianBlur(ir_corrected, ir_corrected, cv::Size(15, 15), 0);
    cv::GaussianBlur(exg, exg, cv::Size(15, 15), 0);

    // NIR
    cv::Mat ir_float;
    ir_corrected.convertTo(ir_float, CV_32F);
    cv::Mat ndvi = (ir_float - red_channel) / (ir_float + red_channel);
    cv::Mat ndvi_binary;
    cv::threshold(ndvi, ndvi_binary, ndvi_threshold, 255, cv::THRESH_BINARY);
    ndvi_binary.convertTo(ndvi_binary, CV_8UC1);

    // threshold
    cv::Mat exg_binary;
    cv::threshold(exg, exg_binary, exg_threshold, 255, cv::THRESH_BINARY);
    exg_binary.convertTo(exg_binary, CV_8UC1);
    cv::Mat nir_binary;
    cv::threshold(ir_corrected, nir_binary, nir_threshold, 255, cv::THRESH_BINARY);
    cv::Mat combined_binary;
    // cv::bitwise_and(exg_binary, nir_binary, combined_binary);  // doesn't really work sadly
    combined_binary = exg_binary.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
    cv::morphologyEx(combined_binary, combined_binary, cv::MORPH_OPEN, kernel);

    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Segmentation: " + std::to_string(ms_int.count()) + " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    int rows = combined_binary.rows;
    int cols = combined_binary.cols;
    std::vector<float> cloud_data;
    cloud_data.reserve(rows * cols);  // speeds up computation
    for (int row = 0; row < combined_binary.rows; row++)
    {
      for (int col = 0; col < combined_binary.cols; col++)
      {
        if ((int)combined_binary.at<std::uint8_t>(row, col) == 0) continue;
        if ((int)depth_mat.at<std::uint16_t>(row, col) == 0) continue;
        k4a_float3_t point3d;
        k4a_float2_t point2d;
        point2d.xy.x = col;
        point2d.xy.y = row;
        int valid;
        k4a_calibration_2d_to_3d(
          &calibration_,
          &point2d,
          depth_mat.at<std::uint16_t>(row, col),
          K4A_CALIBRATION_TYPE_COLOR,
          K4A_CALIBRATION_TYPE_COLOR,
          &point3d,
          &valid
        );
        cloud_data.push_back(point3d.xyz.x);
        cloud_data.push_back(point3d.xyz.y);
        cloud_data.push_back(point3d.xyz.z);
      }
    }
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Project to 3D: " + std::to_string(ms_int.count()) + " ms\n";

    cv::Mat components2d = cv::Mat::zeros(combined_binary.size(), CV_8UC3);
    if (old_version)  // connecting components in 2d
    {
      t1 = std::chrono::high_resolution_clock::now();

      // morphological transforms
      cv::Mat clean_binary;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilation_size, dilation_size));
      cv::morphologyEx(combined_binary, clean_binary, cv::MORPH_DILATE, kernel);

      // seperate components
      cv::Mat labels, stats, centroids;
      int connectivity = 8;
      int num_components = cv::connectedComponentsWithStats(clean_binary, labels, stats, centroids, connectivity);
      t2 = std::chrono::high_resolution_clock::now();
      ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      log_text += "2D Clustering: " + std::to_string(ms_int.count()) + " ms\n";

      // display results
      int min_area = 2000;
      for (int i = 1; i < num_components; i++) {
        cv::Rect bounding_box = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                                        stats.at<int>(i, cv::CC_STAT_TOP),
                                        stats.at<int>(i, cv::CC_STAT_WIDTH),
                                        stats.at<int>(i, cv::CC_STAT_HEIGHT));
        // don't consider small components
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area >= min_area) {
          std::uint8_t r = 255 * (rand() / (1.0 + RAND_MAX));
          std::uint8_t g = 255 * (rand() / (1.0 + RAND_MAX));
          std::uint8_t b = 255 * (rand() / (1.0 + RAND_MAX));
          components2d.setTo(
            cv::Vec3b(r, g, b),
            labels == i
          );
          cv::Point centroid(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
          cv::rectangle(components2d, bounding_box, cv::Scalar(0, 255, 0), 5);
          cv::circle(components2d, centroid, 4, cv::Scalar(0, 0, 255), -1);
        }
      }
    }

    std::string folder_name;
    if (save_runs_)
    {
      // create sub folder
      folder_name = run_folder_ + pos + "_" + std::to_string(process_count_++);
      std::filesystem::create_directory(folder_name);

      // save log text
      std::string file_name = folder_name + "/log_file.txt";  
      std::ofstream outfile;
      outfile.open(file_name);
      outfile << log_text;
      outfile.close();

      // save images
      std::string image_folder = folder_name + "/images/";
      std::filesystem::create_directory(image_folder);

      cv::Mat depth_normalized;
      cv::normalize(depth_mat, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      cv::Mat exg_normalized;
      cv::normalize(exg, exg_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      cv::Mat ndvi_normalized;
      cv::normalize(ndvi, ndvi_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

      cv::imwrite(image_folder + "color.png", color_mat);
      cv::imwrite(image_folder + "depth.png", depth_normalized);
      cv::imwrite(image_folder + "depth16.png", depth_mat);
      cv::imwrite(image_folder + "ir16.png", ir_mat);
      cv::imwrite(image_folder + "ir_binary.png", nir_binary);
      cv::imwrite(image_folder + "ir.png", ir_corrected);
      cv::imwrite(image_folder + "red.png", bgr_channels[1]);
      cv::imwrite(image_folder + "green.png", bgr_channels[2]);
      cv::imwrite(image_folder + "blue.png", bgr_channels[0]);
      cv::imwrite(image_folder + "exg.png", exg_normalized);
      cv::imwrite(image_folder + "exg_binary.png", exg_binary);
      cv::imwrite(image_folder + "ndvi.png", ndvi_normalized);
      cv::imwrite(image_folder + "ndvi_binary.png", ndvi_binary);
      cv::imwrite(image_folder + "combined_binary.png", combined_binary);
      if (old_version)
      {
        cv::imwrite(image_folder + "components2d.png", components2d);
      }

      RCLCPP_INFO(this->get_logger(), "Saved images to %s", image_folder.c_str());
    }

    // create pc message
    sensor_msgs::msg::PointCloud2 ros_pc;
    sensor_msgs::PointCloud2Modifier mod(ros_pc);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t num_points = cloud_data.size() / 3;
    ros_pc.width = num_points;
    ros_pc.height = 1;
    ros_pc.point_step = 12;
    ros_pc.row_step = ros_pc.point_step * ros_pc.width;
    mod.resize(num_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(ros_pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(ros_pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(ros_pc, "z");
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
    {
      *iter_x = cloud_data[3 * i];
      *iter_y = cloud_data[3 * i + 1];
      *iter_z = cloud_data[3 * i + 2];
    }

    // create color_image message
    auto goal = ClusterClassify::Goal();
    sensor_msgs::msg::Image ros_image;
    ros_image.height = color_mat.rows;
    ros_image.width = color_mat.cols;
    ros_image.data.assign(color_mat.datastart, color_mat.dataend);

    // create goal
    goal.pc = ros_pc;
    goal.color_image = ros_image;
    goal.folder = folder_name;
    goal.save_runs = save_runs_;

    // send goal to python node
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_future = this->client_ptr_->async_send_goal(goal);
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }
    result_future = this->client_ptr_->async_get_result(goal_handle);

    return true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeedControlActionServer>());
  rclcpp::shutdown();
  return 0;
}