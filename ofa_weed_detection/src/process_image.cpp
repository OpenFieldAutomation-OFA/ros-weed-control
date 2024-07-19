#include <cstdio>
#include <k4a/k4a.hpp>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_publisher");
  auto color_publisher = node->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
  auto depth_publisher = node->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
  auto ir_publisher = node->create_publisher<sensor_msgs::msg::Image>("ir_image", 10);

  // open the first plugged in Kinect device
  k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

  // set color controls (mostly same as defaults)
  device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
    K4A_COLOR_CONTROL_MODE_MANUAL, 20000);
  device.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS,
    K4A_COLOR_CONTROL_MODE_MANUAL, 10);
  device.set_color_control(K4A_COLOR_CONTROL_CONTRAST,
    K4A_COLOR_CONTROL_MODE_MANUAL, 50);
  device.set_color_control(K4A_COLOR_CONTROL_SATURATION,
    K4A_COLOR_CONTROL_MODE_MANUAL, 64);
  device.set_color_control(K4A_COLOR_CONTROL_SHARPNESS,
    K4A_COLOR_CONTROL_MODE_MANUAL, 24);
  device.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE,
    K4A_COLOR_CONTROL_MODE_MANUAL, 6500);
  device.set_color_control(K4A_COLOR_CONTROL_GAIN,
    K4A_COLOR_CONTROL_MODE_MANUAL, 0);
  device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
    K4A_COLOR_CONTROL_MODE_MANUAL, 1);

  // define config
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.camera_fps = K4A_FRAMES_PER_SECOND_5;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
  config.synchronized_images_only = true;

  k4a_calibration_t calibration = device.get_calibration(config.depth_mode, config.color_resolution);
  k4a::transformation transformation(calibration);

  device.start_cameras(&config);

  k4a::capture capture = k4a::capture::create();

  rclcpp::WallRate loop_rate(1);
  while (rclcpp::ok()) {
    // capture
    device.get_capture(&capture);

    // get images
    k4a::image color_image = capture.get_color_image();
    k4a::image depth_image = capture.get_depth_image();
    k4a::image ir_image = capture.get_ir_image();
    
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
    std::pair<k4a::image, k4a::image> transformed = transformation.depth_image_to_color_camera_custom(
      depth_image,
      ir_image,
      K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR,
      0
    );
    depth_image = transformed.first;
    ir_image = transformed.second;

    // buffer data to vector
    uint8_t *color_buffer = color_image.get_buffer();
    std::vector<uint8_t> color_vector(color_buffer, color_buffer + color_image.get_size());
    uint8_t *depth_buffer = depth_image.get_buffer();
    std::vector<uint8_t> depth_vector(depth_buffer, depth_buffer + depth_image.get_size());
    uint8_t *ir_buffer = ir_image.get_buffer();
    std::vector<uint8_t> ir_vector(ir_buffer, ir_buffer + ir_image.get_size());

    // int stride = ir_image.get_stride_bytes();
    // size_t size = ir_image.get_size();
    // int ir_width = ir_image.get_width_pixels();
    // int ir_height = ir_image.get_height_pixels();
    // printf("width: %i, height: %i, stride: %i, size: %lu\n",
    //   ir_width, ir_height, stride, size);

    // printf("first pixel value: %u, %u, %u, %u\n",
    //   ir_buffer[500000], ir_buffer[500001], ir_buffer[500002], ir_buffer[500003]);
    // printf("second pixel value: %u, %u, %u, %u\n",
    //   ir_buffer[500004], ir_buffer[500005], ir_buffer[500006], ir_buffer[500007]);
    // printf("first pixel value: %u, %u, %u, %u\n",
    //   ir_vector[500000], ir_vector[500001], ir_vector[500002], ir_vector[500003]);
    // printf("second pixel value: %u, %u, %u, %u\n",
    //   ir_vector[500004], ir_vector[500005], ir_vector[500006], ir_vector[500007]);

    // publish images
    sensor_msgs::msg::Image ros_image;
    ros_image.header.stamp = node->get_clock()->now();
    ros_image.header.frame_id = "color_camera";
    ros_image.width = color_image.get_width_pixels();
    ros_image.height = color_image.get_height_pixels();
    ros_image.encoding = "bgra8";
    ros_image.is_bigendian = 0;
    ros_image.step = color_image.get_stride_bytes();
    ros_image.data = color_vector;
    color_publisher->publish(ros_image);
    ros_image.encoding = "mono16";;
    ros_image.width = ir_image.get_width_pixels();
    ros_image.height = ir_image.get_height_pixels();
    ros_image.step = ir_image.get_stride_bytes();
    ros_image.data = ir_vector;
    ir_publisher->publish(ros_image);
    ros_image.data = depth_vector;
    depth_publisher->publish(ros_image);

    RCLCPP_INFO(node->get_logger(), "Published");
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  

  // transform

  device.stop_cameras();

  rclcpp::shutdown();
  return 0;
}
