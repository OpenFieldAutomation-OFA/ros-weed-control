#include <cstdio>
#include <k4a/k4a.hpp>
#include <chrono>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

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

    // Normalize the histogram
    cv::Mat histNorm;
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_publisher");
  auto color_publisher = node->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
  auto depth_publisher = node->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
  auto ir_active_publisher = node->create_publisher<sensor_msgs::msg::Image>("ir_active_image", 10);
  auto ir_passive_publisher = node->create_publisher<sensor_msgs::msg::Image>("ir_passive_image", 10);
  auto red_publisher = node->create_publisher<sensor_msgs::msg::Image>("red_image", 10);
  auto nir_publisher = node->create_publisher<sensor_msgs::msg::Image>("nir_image", 10);
  auto ndvi_publisher = node->create_publisher<sensor_msgs::msg::Image>("ndvi_image", 10);
  auto exg_publisher = node->create_publisher<sensor_msgs::msg::Image>("exg_image", 10);
  auto exg_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("exg_binary_image", 10);
  auto ndvi_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("ndvi_binary_image", 10);
  auto combined_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("combined_binary_image", 10);
  auto hist_publisher = node->create_publisher<sensor_msgs::msg::Image>("hist_image", 10);
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
  k4a_device_configuration_t config_active = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config_active.camera_fps = K4A_FRAMES_PER_SECOND_15;
  config_active.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config_active.color_resolution = K4A_COLOR_RESOLUTION_2160P;
  config_active.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
  config_active.synchronized_images_only = true;
  k4a_device_configuration_t config_passive = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config_passive.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;

  k4a_calibration_t calibration = device.get_calibration(config_active.depth_mode, config_active.color_resolution);
  k4a::transformation transformation(calibration);

  k4a::capture capture_active = k4a::capture::create();
  k4a::capture capture_passive = k4a::capture::create();

  rclcpp::WallRate loop_rate(0.2);
  while (rclcpp::ok()) {
    // capture in passive mode
    device.start_cameras(&config_passive);
    device.get_capture(&capture_passive);
    k4a::image ir_passive_image = capture_passive.get_ir_image();
    device.stop_cameras();

    // capture in active mode
    device.start_cameras(&config_active);
    device.get_capture(&capture_active);
    k4a::image depth_image = capture_active.get_depth_image();
    k4a::image color_image = capture_active.get_color_image();
    k4a::image ir_active_image = capture_active.get_ir_image();
    device.stop_cameras();
    
    printf("color image %i, %i, %li\n",
      color_image.get_width_pixels(),
      color_image.get_height_pixels(),
      color_image.get_size());
    printf("depth image %i, %i, %li\n",
      depth_image.get_width_pixels(),
      depth_image.get_height_pixels(),
      depth_image.get_size());
    printf("active ir %i, %i, %li\n",
      ir_active_image.get_width_pixels(),
      ir_active_image.get_height_pixels(),
      ir_active_image.get_size());
    printf("passive ir %i, %i, %li\n",
      ir_passive_image.get_width_pixels(),
      ir_passive_image.get_height_pixels(),
      ir_passive_image.get_size());
    
    // transform depth and ir images to camera viewpoint
    ir_active_image = k4a::image::create_from_buffer(
      K4A_IMAGE_FORMAT_CUSTOM16,
      ir_active_image.get_width_pixels(),
      ir_active_image.get_height_pixels(),
      ir_active_image.get_stride_bytes(),
      ir_active_image.get_buffer(),
      ir_active_image.get_size(),
      [](void *, void *) {},
      NULL
    );
    std::pair<k4a::image, k4a::image> transformed = transformation.depth_image_to_color_camera_custom(
      depth_image,
      ir_active_image,
      K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
      0
    );
    ir_active_image = transformed.second;
    ir_passive_image = k4a::image::create_from_buffer(
      K4A_IMAGE_FORMAT_CUSTOM16,
      ir_passive_image.get_width_pixels(),
      ir_passive_image.get_height_pixels(),
      ir_passive_image.get_stride_bytes(),
      ir_passive_image.get_buffer(),
      ir_passive_image.get_size(),
      [](void *, void *) {},
      NULL
    );
    transformed = transformation.depth_image_to_color_camera_custom(
      depth_image,
      ir_passive_image,
      K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR,
      0
    );
    depth_image = transformed.first;
    ir_passive_image = transformed.second;

    // buffer data to cv2
    cv::Mat color_mat(
      color_image.get_height_pixels(),
      color_image.get_width_pixels(),
      CV_8UC4,
      color_image.get_buffer()
    );
    std::size_t mono_size = color_image.get_width_pixels() * color_image.get_height_pixels();
    uint8_t *depth_buffer = depth_image.get_buffer();
    std::vector<uint16_t> depth_vector(mono_size);
    for (std::size_t i = 0; i < mono_size; i++)
    {
      depth_vector[i] = static_cast<uint16_t>(depth_buffer[2 * i] | (depth_buffer[2 * i + 1] << 8));
    }
    cv::Mat depth_mat(
      depth_image.get_height_pixels(),
      depth_image.get_width_pixels(),
      CV_16UC1,
      depth_vector.data()
    );
    uint8_t *ir_passive_buffer = ir_passive_image.get_buffer();
    std::vector<uint16_t> ir_passive_vector(mono_size);
    for (std::size_t i = 0; i < mono_size; i++)
    {
      ir_passive_vector[i] = static_cast<uint16_t>(ir_passive_buffer[2 * i] | (ir_passive_buffer[2 * i + 1] << 8));
    }
    cv::Mat ir_passive_mat(
      ir_passive_image.get_height_pixels(),
      ir_passive_image.get_width_pixels(),
      CV_16UC1,
      ir_passive_vector.data()
    );
    uint8_t *ir_active_buffer = ir_active_image.get_buffer();
    std::vector<uint16_t> ir_active_vector(mono_size);
    for (std::size_t i = 0; i < mono_size; i++)
    {
      ir_active_vector[i] = static_cast<uint16_t>(ir_active_buffer[2 * i] | (ir_active_buffer[2 * i + 1] << 8));
    }
    cv::Mat ir_active_mat(
      ir_active_image.get_height_pixels(),
      ir_active_image.get_width_pixels(),
      CV_16UC1,
      ir_active_vector.data()
    );

    // calculate NDVI
    cv::Mat bgr_mat;
    cv::cvtColor(color_mat, bgr_mat, cv::COLOR_BGRA2BGR);
    std::vector<cv::Mat> bgr_channels;
    cv::split(bgr_mat, bgr_channels);
    cv::Mat blue_channel = bgr_channels[0];
    cv::Mat green_channel = bgr_channels[1];
    cv::Mat red_channel = bgr_channels[2];
    cv::Mat blue_channel_float, green_channel_float, red_channel_float;
    blue_channel.convertTo(blue_channel_float, CV_32F);
    green_channel.convertTo(green_channel_float, CV_32F);
    red_channel.convertTo(red_channel_float, CV_32F);
    cv::Mat nir_normalized;
    cv::normalize(ir_passive_mat, nir_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat nir_float;
    nir_normalized.convertTo(nir_float, CV_32F);
    cv::Mat ndvi;
    cv::divide((nir_float - red_channel_float), (nir_float + red_channel_float), ndvi);
    cv::Mat ndvi_normalized;
    cv::normalize(ndvi, ndvi_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // calculate ExG
    cv::Mat exg = 2 * green_channel_float - red_channel_float - blue_channel_float;
    cv::Mat exg_normalized;
    cv::normalize(exg, exg_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // threshold
    double exg_threshold = 128;
    double ndvi_threshold = 128;
    cv::Mat exg_binary;
    cv::threshold(exg_normalized, exg_binary, exg_threshold, 255, cv::THRESH_BINARY);
    cv::Mat ndvi_binary;
    cv::threshold(ndvi_normalized, ndvi_binary, ndvi_threshold, 255, cv::THRESH_BINARY);
    cv::Mat combined_binary;
    cv::bitwise_and(exg_binary, ndvi_binary, combined_binary);
    // threshold_value = cv::threshold(ndvi_normalized, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // printf("otsus value: %f\n", threshold_value);

    // draw histogram
    cv::Mat hist = plotHistogram(exg_normalized);

    // publish images
    std_msgs::msg::Header header;
    header.stamp = node->now();
    color_publisher->publish(
      *cv_bridge::CvImage(header, "bgra8", color_mat).toImageMsg().get()
    );
    ir_active_publisher->publish(
      *cv_bridge::CvImage(header, "mono16", ir_active_mat).toImageMsg().get()
    );
    ir_passive_publisher->publish(
      *cv_bridge::CvImage(header, "mono16", ir_passive_mat).toImageMsg().get()
    );
    depth_publisher->publish(
      *cv_bridge::CvImage(header, "mono16", depth_mat).toImageMsg().get()
    );
    red_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", red_channel).toImageMsg().get()
    );
    nir_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", nir_normalized).toImageMsg().get()
    );
    ndvi_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", ndvi_normalized).toImageMsg().get()
    );
    exg_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", exg_normalized).toImageMsg().get()
    );
    hist_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", hist).toImageMsg().get()
    );
    exg_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", exg_binary).toImageMsg().get()
    );
    ndvi_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", ndvi_binary).toImageMsg().get()
    );
    combined_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", combined_binary).toImageMsg().get()
    );

    RCLCPP_INFO(node->get_logger(), "Published");
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }



  device.stop_cameras();

  rclcpp::shutdown();
  return 0;
}
