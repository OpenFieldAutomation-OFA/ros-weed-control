#include <cstdio>
#include <cstdint>
#include <k4a/k4a.hpp>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/ml/kmeans.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ofa_interfaces/action/weed_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Function to send command to the Arduino
void send_command(int arduino_port_, const std::string& command)
{
  write(arduino_port_, command.c_str(), command.length());
  write(arduino_port_, "\n", 1); // Send newline character
}

cv::Mat plotHistogram(const cv::Mat& image, int histSize = 256, int histWidth = 512, int histHeight = 400) {
    // Check if the input image is a single-channel 8-bit image
    if (image.channels() != 1 || image.type() != CV_8UC1) {
        throw std::invalid_argument("Input image must be an 8-bit single-channel image.");
    }

    // Parameters for histogram calculation
    float range[] = {0, 256}; // Range of pixel values
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    // Compute the histogram
    cv::Mat hist;
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
    // hist.convertTo(hist, CV_32F);

    // Normalize the histogram
    cv::Mat histNorm;
    // cv::divide(100, hist, histNorm);
    // histNorm.convertTo(histNorm, CV_8UC1);
    cv::normalize(hist, histNorm, 0, histHeight, cv::NORM_MINMAX);

    // Create an image to display the histogram
    int binWidth = cvRound((double) histWidth / histSize);
    cv::Mat histImage(histHeight, histWidth, CV_8UC1, cv::Scalar(255));

    // Draw the histogram
    for (int i = 1; i < histSize; i++) {
        cv::line(histImage,
                 cv::Point(binWidth * (i - 1), histHeight - cvRound(histNorm.at<float>(i - 1))),
                 cv::Point(binWidth * i, histHeight - cvRound(histNorm.at<float>(i))),
                 cv::Scalar(0), 2, 8, 0);
    }

    return histImage;
}


class WeedControlActionServer : public rclcpp::Node
{
public:
  using WeedControl = ofa_interfaces::action::WeedControl;
  using GoalHandleWeedControl = rclcpp_action::ServerGoalHandle<WeedControl>;
  
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
      oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
      folder_name_ = "/home/ubuntu/overlay/src/ofa_weed_detection/runs/" + oss.str() + "/";
      std::filesystem::create_directory(folder_name_);

    }

    this->init_action_server();
    
    if (!this->init_arduino())
    {
      rclcpp::shutdown();
    }
    this->init_camera();    
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
  std::string folder_name_;
  int process_count_ = 0;
  bool save_runs_;

  void init_action_server()
  {
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const WeedControl::Goal> /* goal */)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
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
    // open the first plugged in Kinect device
    device_ = k4a::device::open(K4A_DEVICE_DEFAULT);

    // set color controls (mostly same as camera defaults)
    device_.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
      K4A_COLOR_CONTROL_MODE_AUTO, 0);
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
        std_msgs::msg::Header header;
        header.frame_id = "camera_color_frame";
        header.stamp = this->now();
        color_publisher_->publish(
          *cv_bridge::CvImage(header, "bgr8", color_mat).toImageMsg().get()
        );

        frames = 0;
      }
    }
  }

  void execute(const std::shared_ptr<GoalHandleWeedControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<WeedControl::Result>();

    // init movegroup
    moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "arm");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    // move_group.setPlanningTime(0.2);

    // move to first position
    move_group.setJointValueTarget({-0.25, 0.8, 0.0});

    moveit::core::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    error_code = move_group.move();
    if (error_code)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to first photo position.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not reach first photo position: %s",
        moveit::core::errorCodeToString(error_code).c_str());
      goal_handle->abort(result);
      return;
    }

    if (!weed_control(move_group))
    {
      goal_handle->abort(result);
    }

    // move to second position
    move_group.setJointValueTarget({-0.75, 0.8, 0.0});

    error_code = move_group.move();
    if (error_code)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to second photo position.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not reach second photo position: %s",
        moveit::core::errorCodeToString(error_code).c_str());
      goal_handle->abort(result);
      return;
    }

    if (!weed_control(move_group))
    {
      goal_handle->abort(result);
    }

    // move to home
    move_group.setNamedTarget("home");
    move_group.move();

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  bool weed_control(moveit::planning_interface::MoveGroupInterface & move_group)
  {
    moveit::core::MoveItErrorCode error_code;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // get transform
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        "world", "camera_color_frame",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s", ex.what());
      return false;
    }

    std::vector<std::vector<float>> camera_positions = process_image();  

    double x_lower = this->get_parameter("x_lower").as_double();  // world points with lower x value will be ignored
    double x_upper = this->get_parameter("x_upper").as_double();  // world points with lower x value will be ignored
    
    /*
    Offsets in x, y, and z-direction
    Ideally the URDF would describe our setup perfectly, but practically it's easier to tune the offsets as parameters
    */ 
    double x_off = this->get_parameter("x_off").as_double();
    double y_off = this->get_parameter("y_off").as_double();
    double z_off = this->get_parameter("z_off").as_double();
    double collision_off = this->get_parameter("collision_off").as_double();
    int wait_shock = this->get_parameter("wait_shock").as_int();

    // transform positions
    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_color_frame";
    geometry_msgs::msg::PointStamped point_in_world;
    std::vector<std::vector<double>> world_positions;
    for (const std::vector<float> & centroid : camera_positions)
    {
      point_in_camera.point.x = centroid[0] / 1000;
      point_in_camera.point.y = centroid[1] / 1000;
      point_in_camera.point.z = centroid[2] / 1000;
      RCLCPP_INFO(this->get_logger(), "Centroid in camera frame: [%f, %f, %f]",
        point_in_camera.point.x,
        point_in_camera.point.y,
        point_in_camera.point.z);
      tf2::doTransform(point_in_camera, point_in_world, t);
      RCLCPP_INFO(this->get_logger(), "Centroid in world frame: [%f, %f, %f]",
        point_in_world.point.x,
        point_in_world.point.y,
        point_in_world.point.z);
      
      if (point_in_world.point.x < x_lower || point_in_world.point.x > x_upper) continue;
      world_positions.push_back(
        {
          point_in_world.point.x + x_off,
          point_in_world.point.y + y_off,
          point_in_world.point.z + z_off
        }
      );
    }

    // sort world positions
    std::vector<std::vector<double>> positive_y;
    std::vector<std::vector<double>> negative_y;
    for (const std::vector<double> & position : world_positions) {
      if (position[1] > 0.0) {
        positive_y.push_back(position);
      } else {
        negative_y.push_back(position);
      }
    }
    std::sort(positive_y.begin(), positive_y.end(), 
      [](const std::vector<double>& a, const std::vector<double>& b) {
          return a[0] < b[0]; // Sort by x-coordinate in increasing order
      });
    std::sort(negative_y.begin(), negative_y.end(), 
      [](const std::vector<double>& a, const std::vector<double>& b) {
          return a[0] > b[0]; // Sort by x-coordinate in decreasing order
      });
    std::vector<std::vector<double>> sorted_positions = positive_y;
    sorted_positions.insert(sorted_positions.end(), negative_y.begin(), negative_y.end());

    // move to targets
    bool first = true;
    geometry_msgs::msg::Pose target_pose;
    move_group.setGoalOrientationTolerance(45 * M_PI / 180);
    for (const std::vector<double> & position : sorted_positions)
    {
      geometry_msgs::msg::Pose old_target_pose = target_pose;
      
      target_pose.position.x = position[0];
      target_pose.position.y = position[1];
      target_pose.position.z = position[2];
      
      if (first || (old_target_pose.position.y > 0 && target_pose.position.y < 0))
      {
        RCLCPP_INFO(this->get_logger(), "Moving to first plant");
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
        tf2::Quaternion q;
        // make sure arm is correctly positioned
        if (position[1] < 0)
        {
          q.setRPY(-45 * M_PI / 180, 0, 0);
        }
        else
        {
          q.setRPY(45 * M_PI / 180, 0, 0);
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
              moveit::core::errorCodeToString(error_code).c_str());
            return false;
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Planning failed: %s",
            moveit::core::errorCodeToString(error_code).c_str());
          continue;
        }
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
        first = false;
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
              moveit::core::errorCodeToString(error_code).c_str());
            return false;
          }
        }
      }

      // shock
      rclcpp::sleep_for(std::chrono::milliseconds(wait_shock));
    }
    return true;
  }

  std::vector<std::vector<float>> process_image()
  {
    // get parameters
    const double cluster_distance = this->get_parameter("cluster_distance").as_double();  // tolerance in mm for point cloud clustering
    const double scale_z = this->get_parameter("scale_z").as_double();;  // scales depth value in point cloud
    const int split_size = this->get_parameter("split_size").as_int();  // big plant point cloud split
    const double exg_threshold = this->get_parameter("exg_threshold").as_double();;  // excess green threshold
    const double nir_threshold = this->get_parameter("nir_threshold").as_double();  // nir threshold
    const int wait_led = this->get_parameter("wait_led").as_int();  // dilation kernel size
    
    const bool old_version = this->get_parameter("old_version").as_bool();
    const int dilation_size = this->get_parameter("dilation_size").as_int();;  // dilation kernel size

    // turn on LEDs
    send_command(arduino_port_, "COLOR 255,255,128");

    // wait for auto exposure to settle

    std::this_thread::sleep_for(std::chrono::milliseconds(wait_led));

    // get images (capture done in seperate thread)
    k4a::image depth_image = capture_.get_depth_image();
    k4a::image color_image = capture_.get_color_image();
    k4a::image ir_image = capture_.get_ir_image();
    send_command(arduino_port_, "OFF");

    std::string log_text = "";
    auto t1_total = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    
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

    auto t2 = std::chrono::high_resolution_clock::now();
    auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Transform images: " + std::to_string(ms_int.count()) + " ms\n";

    // decode JPG
    t1 = std::chrono::high_resolution_clock::now();
    std::vector<std::uint8_t> color_buffer(color_image.get_buffer(), color_image.get_buffer() + color_image.get_size());
    cv::Mat color_mat = cv::imdecode(color_buffer, cv::IMREAD_ANYCOLOR);
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Decode JPEG: " + std::to_string(ms_int.count()) + " ms\n";

    // convert depth and ir to opencv
    t1 = std::chrono::high_resolution_clock::now();
    cv::Mat depth_mat(
      depth_image.get_height_pixels(),
      depth_image.get_width_pixels(),
      CV_16UC1,
      reinterpret_cast<uint16_t*>(depth_image.get_buffer())
    );
    cv::Mat ir_mat(
      ir_image.get_height_pixels(),
      ir_image.get_width_pixels(),
      CV_16UC1,
      reinterpret_cast<uint16_t*>(ir_image.get_buffer())
    );
    cv::min(ir_mat, 2048, ir_mat); // remove outliers (reflections, saturated pixels)
    cv::normalize(ir_mat, ir_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat depth_normalized;
    cv::normalize(depth_mat, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Convert to OpenCV: " + std::to_string(ms_int.count()) + " ms\n";

    // calculate ExG
    t1 = std::chrono::high_resolution_clock::now();
    std::vector<cv::Mat> bgr_channels;
    cv::split(color_mat, bgr_channels);
    cv::Mat blue_channel, green_channel, red_channel;
    bgr_channels[0].convertTo(blue_channel, CV_32F);
    bgr_channels[1].convertTo(green_channel, CV_32F);
    bgr_channels[2].convertTo(red_channel, CV_32F);
    cv::Mat exg = 2.0 * green_channel - red_channel - blue_channel;
    cv::Mat exg_normalized;  // only used to display
    cv::normalize(exg, exg_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // remove some noise
    cv::GaussianBlur(ir_mat, ir_mat, cv::Size(15, 15), 0);
    cv::GaussianBlur(exg, exg, cv::Size(15, 15), 0);

    // threshold
    cv::Mat exg_binary;
    cv::threshold(exg, exg_binary, exg_threshold, 255, cv::THRESH_BINARY);
    exg_binary.convertTo(exg_binary, CV_8UC1);
    cv::Mat nir_binary;
    cv::threshold(ir_mat, nir_binary, nir_threshold, 255, cv::THRESH_BINARY);
    cv::Mat combined_binary;
    cv::bitwise_and(exg_binary, nir_binary, combined_binary);
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Segmentation: " + std::to_string(ms_int.count()) + " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int rows = combined_binary.rows;
    int cols = combined_binary.cols;
    cloud->points.reserve(rows * cols);  // speeds up computation
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
        pcl::PointXYZ point;
        point.x = point3d.xyz.x;
        point.y = point3d.xyz.y;
        // we scale the z dimension such that only points with adjacent depth values are connected
        point.z = point3d.xyz.z * scale_z;
        cloud->points.push_back(point);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Project to 3D: " + std::to_string(ms_int.count()) + " ms\n";

    // downsample cloud
    t1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(1.0, 1.0, 1.0);
    sor.filter(*cloud_filtered);
    log_text += "Point cloud size: " + std::to_string(cloud->width) + "\n";
    log_text += "Filtered point cloud size: " + std::to_string(cloud_filtered->width) + "\n"; 
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "Filter 3d points: " + std::to_string(ms_int.count()) + " ms\n";  

    // cluster cloud
    t1 = std::chrono::high_resolution_clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_distance); // Set the distance tolerance
    ec.setMinClusterSize(100);    // Set the minimum number of points in a cluster
    ec.setMaxClusterSize(1000000);  // Set the maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "3D Clustering: " + std::to_string(ms_int.count()) + " ms\n";

    // project clusters back
    t1 = std::chrono::high_resolution_clock::now();
    cv::Mat components3d = cv::Mat::zeros(combined_binary.size(), CV_8UC3);
    cv::Mat components3dcolor = color_mat.clone();
    std::vector<std::vector<float>> positions;  // 3d coordinate(s) that should be treated

    for (const auto& indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::Kmeans kmeans(indices.indices.size(), 3);
      // int k = std::min(indices.indices.size() / split_size + 1, static_cast<unsigned long>(20));
      int k = indices.indices.size() / split_size + 1;
      RCLCPP_INFO(this->get_logger(), "Cluster size: %ld\n", indices.indices.size());
      kmeans.setClusterSize(k);

      std::uint8_t r = 255 * (rand() / (1.0 + RAND_MAX));
      std::uint8_t g = 255 * (rand() / (1.0 + RAND_MAX));
      std::uint8_t b = 255 * (rand() / (1.0 + RAND_MAX));

      int min_row = combined_binary.rows;
      int min_col = combined_binary.cols;
      int max_row = 0;
      int max_col = 0;

      for (const auto& idx : indices.indices) {
        cloud_filtered->points[idx].z /= scale_z;

        cloud_cluster->push_back((*cloud_filtered)[idx]);
        
        std::vector<float> data(3);
        data[0] = cloud_filtered->points[idx].x;
        data[1] = cloud_filtered->points[idx].y;
        data[2] = cloud_filtered->points[idx].z;
        kmeans.addDataPoint(data);

        k4a_float3_t point3d;
        k4a_float2_t point2d;
        point3d.xyz.x = cloud_filtered->points[idx].x;
        point3d.xyz.y = cloud_filtered->points[idx].y;
        point3d.xyz.z = cloud_filtered->points[idx].z;
        int valid = 0;
        k4a_calibration_3d_to_2d(
          &calibration_,
          &point3d,
          K4A_CALIBRATION_TYPE_COLOR,
          K4A_CALIBRATION_TYPE_COLOR,
          &point2d,
          &valid
        );
        int row = std::round(point2d.xy.y);
        int col = std::round(point2d.xy.x);
        if (row > max_row) max_row = row;
        if (row < min_row) min_row = row;
        if (col > max_col) max_col = col;
        if (col < min_col) min_col = col;

        // cv::Point projected(col, row);
        // cv::circle(components3d, projected, 2, cv::Scalar(r, g, b), 2);

        components3d.at<cv::Vec3b>(row, col)[0] = r; 
        components3d.at<cv::Vec3b>(row, col)[1] = g; 
        components3d.at<cv::Vec3b>(row, col)[2] = b;
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f\n", image_point.xy.x, image_point.xy.y);
      }
      cv::Rect bounding_box = cv::Rect(min_col, min_row, max_col - min_col, max_row - min_row);
      cv::rectangle(components3d, bounding_box, cv::Scalar(0, 255, 0), 2);
      cv::rectangle(components3dcolor, bounding_box, cv::Scalar(0, 255, 0), 2);

      // TODO: check if plant crop or not (classification)
      
      // positions
      std::vector<std::vector<float>> centroids;
      if (k == 1)
      {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        positions.push_back({centroid[0], centroid[1], centroid[2]});
      }
      else
      {
        kmeans.kMeans();
        int ccount = 0;
        std::vector<std::vector<float>> centroids = kmeans.get_centroids();
        for (const std::vector<float> & centroid : centroids)
        {          
          if (!isnan(centroid[0]) && !isnan(centroid[1]) && !isnan(centroid[2]))
          {
            positions.push_back(centroid);
            ccount++;
          }
        }
        RCLCPP_INFO(this->get_logger(), "Centroid count: %d\n", ccount);
      }
    }
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    log_text += "KMeans: " + std::to_string(ms_int.count()) + " ms\n";

    auto t2_total = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2_total - t1_total);
    log_text += "Total time to process image: " + std::to_string(ms_int.count()) + " ms\n";

    // draw 3d components
    for (const std::vector<float> & centroid : positions)
    {
      RCLCPP_INFO(this->get_logger(), "centroid %f: %f, %f\n",
        centroid[0], centroid[1], centroid[2]);
      k4a_float3_t point3d;
      k4a_float2_t point2d;
      point3d.xyz.x = centroid[0];
      point3d.xyz.y = centroid[1];
      point3d.xyz.z = centroid[2];
      int valid = 0;
      k4a_calibration_3d_to_2d(
        &calibration_,
        &point3d,
        K4A_CALIBRATION_TYPE_COLOR,
        K4A_CALIBRATION_TYPE_COLOR,
        &point2d,
        &valid
      );
      int row = std::round(point2d.xy.y);
      int col = std::round(point2d.xy.x);
      cv::Point projected_centroid(col, row);
      cv::circle(components3d, projected_centroid, 10, cv::Scalar(0, 0, 255), -1);
      cv::circle(components3dcolor, projected_centroid, 10, cv::Scalar(0, 0, 255), -1);
    }

    cv::Mat components2d = cv::Mat::zeros(combined_binary.size(), CV_8UC3);
    if (old_version)  // connecting components in 2d
    {
      t1 = std::chrono::high_resolution_clock::now();

      // morphological transforms
      cv::Mat clean_binary;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilation_size, dilation_size));
      cv::morphologyEx(combined_binary, clean_binary, cv::MORPH_DILATE, kernel);

      // seperate components
      cv::Mat labels, stats, centroids;
      int connectivity = 8;
      int num_components = cv::connectedComponentsWithStats(clean_binary, labels, stats, centroids, connectivity);
      t2 = std::chrono::high_resolution_clock::now();
      ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      log_text += "2D Clustering: " + std::to_string(ms_int.count()) + " ms\n";

      // display results
      int min_area = 1000;
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

    if (save_runs_)
    {
      // save log text
      std::string folder_name = folder_name_ + std::to_string(process_count_++) + "/";
      std::filesystem::create_directory(folder_name);
      std::string file_name = folder_name + "log_file.txt";  
      std::ofstream outfile;
      outfile.open(file_name);
      outfile << log_text;
      outfile.close();

      // save images
      folder_name += "images/";
      std::filesystem::create_directory(folder_name);

      cv::imwrite(folder_name + "color.png", color_mat);
      cv::imwrite(folder_name + "depth.png", depth_normalized);
      cv::imwrite(folder_name + "ir.png", ir_mat);
      cv::imwrite(folder_name + "ir_binary.png", nir_binary);
      cv::imwrite(folder_name + "red.png", bgr_channels[1]);
      cv::imwrite(folder_name + "green.png", bgr_channels[2]);
      cv::imwrite(folder_name + "blue.png", bgr_channels[0]);
      cv::imwrite(folder_name + "exg.png", exg_normalized);
      cv::imwrite(folder_name + "exg_binary.png", exg_binary);
      cv::imwrite(folder_name + "combined_binary.png", combined_binary);
      cv::imwrite(folder_name + "components3d.png", components3d);
      cv::imwrite(folder_name + "components3dcolor.png", components3dcolor);
      if (old_version)
      {
        cv::imwrite(folder_name + "components2d.png", components2d);
      }

      RCLCPP_INFO(this->get_logger(), "Saved images to %s", folder_name.c_str());
    }

    return positions;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeedControlActionServer>());
  rclcpp::shutdown();
  return 0;
}