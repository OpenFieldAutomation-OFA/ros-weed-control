#include <cstdio>
#include <k4a/k4a.hpp>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <termios.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#define SAVE_IMAGES true

struct PointXYZHash {
    std::size_t operator()(const pcl::PointXYZ& point) const {
        std::size_t hx = std::hash<int>()(static_cast<int>(point.x));
        std::size_t hy = std::hash<int>()(static_cast<int>(point.y));
        std::size_t hz = std::hash<int>()(static_cast<int>(point.z));
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};
struct PointXYZEqual {
    bool operator()(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) const {
        float epsilon = 2;
        return (std::abs(p1.x - p2.x) < epsilon) &&
               (std::abs(p1.y - p2.y) < epsilon) &&
               (std::abs(p1.z - p2.z) < epsilon);
    }
};

// Function to send command to the Arduino
void send_command(int serial_port, const std::string& command)
{
    write(serial_port, command.c_str(), command.length());
    write(serial_port, "\n", 1); // Send newline character
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_publisher");

  // create image publishers
  auto color_publisher = node->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
  auto depth_publisher = node->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
  auto hist_publisher = node->create_publisher<sensor_msgs::msg::Image>("hist_image", 10);  auto ir_active_publisher = node->create_publisher<sensor_msgs::msg::Image>("ir_active_image", 10);
  auto red_publisher = node->create_publisher<sensor_msgs::msg::Image>("red_image", 10);
  auto green_publisher = node->create_publisher<sensor_msgs::msg::Image>("green_image", 10);
  auto blue_publisher = node->create_publisher<sensor_msgs::msg::Image>("blue_image", 10);
  auto nir_publisher = node->create_publisher<sensor_msgs::msg::Image>("nir_image", 10);
  auto exg_publisher = node->create_publisher<sensor_msgs::msg::Image>("exg_image", 10);
  auto exg_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("exg_binary_image", 10);
  auto nir_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("nir_binary_image", 10);
  auto combined_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("combined_binary_image", 10);
  auto clean_binary_publisher = node->create_publisher<sensor_msgs::msg::Image>("clean_binary_image", 10);
  auto components_publisher = node->create_publisher<sensor_msgs::msg::Image>("components_image", 10);
  
  // open serial comm to arduino
  const char* arduino = "/dev/ttyACM0";
  int serial_port = open(arduino, O_RDWR);
  if (serial_port < 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Error %i from opening Arduino port: %s", errno, strerror(errno));
    return 0;
  }
  termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_port, &tty) != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Error %i from tcgetattr: %s", errno, strerror(errno));
      close(serial_port);
    return 0;
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
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Error %i from tcsetattr: %s", errno, strerror(errno));
      close(serial_port);
    return 0;
  }
  
  // turn off leds and set brightness to maximum
  send_command(serial_port, "OFF");
  send_command(serial_port, "BRIGHTNESS 255");

  // open the first plugged in Kinect device
  k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

  // set color controls (mostly same as camera defaults)
  device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
    K4A_COLOR_CONTROL_MODE_AUTO, 0);
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
  // k4a_device_configuration_t config_passive = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  // config_passive.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;

  k4a_calibration_t calibration = device.get_calibration(config_active.depth_mode, config_active.color_resolution);
  k4a::transformation transformation(calibration);

  k4a::capture capture_active = k4a::capture::create();
  // k4a::capture capture_passive = k4a::capture::create();

  // rclcpp::WallRate loop_rate(0.2);
  // while (rclcpp::ok()) {
    send_command(serial_port, "COLOR 255,255,150");
    device.start_cameras(&config_active);
    
    // wait for auto exposure to settle
    for (int i = 0; i < 20; i++)
    {
      device.get_capture(&capture_active);
    }

    // get images
    device.get_capture(&capture_active);
    k4a::image depth_image = capture_active.get_depth_image();
    k4a::image color_image = capture_active.get_color_image();
    k4a::image ir_active_image = capture_active.get_ir_image();
    send_command(serial_port, "OFF");
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
    
    // transform depth and ir image to camera viewpoint
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
      K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR,
      0
    );
    depth_image = transformed.first;
    ir_active_image = transformed.second;

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
    cv::Mat bgr_mat;
    cv::cvtColor(color_mat, bgr_mat, cv::COLOR_BGRA2BGR);
    cv::Mat nir_normalized;
    cv::min(ir_active_mat, 2048, ir_active_mat); // remove outliers (reflections, saturated pixels)
    cv::normalize(ir_active_mat, nir_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat depth_normalized;
    cv::min(depth_mat, 600, depth_mat);
    cv::max(depth_mat, 300, depth_mat);
    cv::normalize(depth_mat, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::Scalar mean, stddev;
    cv::meanStdDev(depth_mat, mean, stddev);
    printf("mean: %f, std: %f\n", mean[0], stddev[0]);
    cv::Mat hist = plotHistogram(depth_normalized);

    if (SAVE_IMAGES)
    {
      cv::imwrite("/home/ros/overlay/src/ofa_weed_detection/images/color.png", bgr_mat);
      cv::imwrite("/home/ros/overlay/src/ofa_weed_detection/images/depth.png", depth_mat);
      cv::imwrite("/home/ros/overlay/src/ofa_weed_detection/images/ir.png", nir_normalized);
    }

    // calculate ExG
    std::vector<cv::Mat> bgr_channels;
    cv::split(bgr_mat, bgr_channels);
    cv::Mat blue_channel = bgr_channels[0];
    cv::Mat green_channel = bgr_channels[1];
    cv::Mat red_channel = bgr_channels[2];
    cv::Mat blue_channel_float, green_channel_float, red_channel_float, nir_float;
    blue_channel.convertTo(blue_channel_float, CV_32F);
    green_channel.convertTo(green_channel_float, CV_32F);
    red_channel.convertTo(red_channel_float, CV_32F);
    cv::Mat exg = 2 * green_channel_float - red_channel_float - blue_channel_float;
    cv::Mat exg_normalized;  // only used to display
    cv::normalize(exg, exg_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // remove some noise
    cv::GaussianBlur(nir_normalized, nir_normalized, cv::Size(15, 15), 0);
    cv::GaussianBlur(exg, exg, cv::Size(15, 15), 0);

    // threshold
    double exg_threshold = -20;
    double nir_threshold = 80;
    cv::Mat exg_binary;
    cv::threshold(exg, exg_binary, exg_threshold, 255, cv::THRESH_BINARY);
    exg_binary.convertTo(exg_binary, CV_8UC1);
    cv::Mat nir_binary;
    cv::threshold(nir_normalized, nir_binary, nir_threshold, 255, cv::THRESH_BINARY);
    cv::Mat combined_binary;
    cv::bitwise_and(exg_binary, nir_binary, combined_binary);
    // threshold_value = cv::threshold(ndvi_normalized, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // printf("otsus value: %f\n", threshold_value);

    // // convert to point cloud
    // k4a::image point_cloud_image = transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_COLOR);
    // int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();

    std::unordered_set<pcl::PointXYZ, PointXYZHash, PointXYZEqual> uniquePoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = 0; row < combined_binary.rows; row++)
    {
      for (int col = 0; col < combined_binary.cols; col++)
      {
        
        if ((int)combined_binary.at<uchar>(row, col) == 0) continue;
        // int k = i * combined_binary.cols + j;
        // int16_t x = point_cloud_image_data[3 * k + 0];
        // int16_t y = point_cloud_image_data[3 * k + 1];
        // int16_t z = point_cloud_image_data[3 * k + 2];
        // if (z == 0) continue;
        // pcl::PointXYZ point;
        // point.x = x;
        // point.y = y;
        // point.z = z;
        k4a_float3_t point3d;
        k4a_float2_t point2d;
        point2d.xy.x = col;
        point2d.xy.y = row;
        int valid;
        k4a_calibration_2d_to_3d(
          &calibration,
          &point2d,
          depth_mat.at<uint16_t>(row, col),
          K4A_CALIBRATION_TYPE_COLOR,
          K4A_CALIBRATION_TYPE_COLOR,
          &point3d,
          &valid
        );
        pcl::PointXYZ point;
        point.x = point3d.xyz.x;
        point.y = point3d.xyz.y;
        point.z = point3d.xyz.z;
        if (uniquePoints.find(point) == uniquePoints.end()) {
          uniquePoints.insert(point);
          cloud->points.push_back(point);
        }

        int k = row * combined_binary.rows + col;

        if (k < 300000)
        {
          k4a_float2_t point2d2;
          k4a_calibration_3d_to_2d(
            &calibration,
            &point3d,
            K4A_CALIBRATION_TYPE_COLOR,
            K4A_CALIBRATION_TYPE_COLOR,
            &point2d2,
            &valid
          );
          int image_x = std::round(point2d2.xy.x);
          int image_y = std::round(point2d2.xy.y);
          printf("position: %d, %d, 3d point: %f, %f, %f, reprojected: %d, %d\n",
            row, col, point3d.xyz.x, point3d.xyz.y, point3d.xyz.z, image_y, image_x);
        }
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;

    printf("size: %d\n", cloud->width);

    printf("start\n");

    // cluster cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(10); // Set the distance tolerance
    ec.setMinClusterSize(1000);    // Set the minimum number of points in a cluster
    ec.setMaxClusterSize(100000);  // Set the maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    printf("done\n");

    // project clusters back
    cv::Mat components = cv::Mat::zeros(combined_binary.size(), CV_8UC3);
    for (const auto& indices : cluster_indices) {
      uint8_t r = 255 * (rand() / (1.0 + RAND_MAX));
      uint8_t g = 255 * (rand() / (1.0 + RAND_MAX));
      uint8_t b = 255 * (rand() / (1.0 + RAND_MAX));
      for (const auto& idx : indices.indices) {
        k4a_float3_t point3d;
        k4a_float2_t point2d;
        point3d.xyz.x = cloud->points[idx].x;
        point3d.xyz.y = cloud->points[idx].y;
        point3d.xyz.z = cloud->points[idx].z;
        int valid = 0;
        k4a_calibration_3d_to_2d(
          &calibration,
          &point3d,
          K4A_CALIBRATION_TYPE_COLOR,
          K4A_CALIBRATION_TYPE_COLOR,
          &point2d,
          &valid
        );
        int row = std::round(point2d.xy.y);
        int col = std::round(point2d.xy.x);

        if (row > components.rows || col > components.cols || row < 0 || col < 0)
        {
          printf("wtf: %d, %d\n", row, col);
          continue;
        }
        components.at<cv::Vec3b>(row, col)[0] = r; 
        components.at<cv::Vec3b>(row, col)[1] = g; 
        components.at<cv::Vec3b>(row, col)[2] = b; 
        // printf("x: %f, y: %f\n", image_point.xy.x, image_point.xy.y);
      }
    }

    // morphological transforms
    // cv::Mat clean_binary;
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
    // cv::morphologyEx(combined_binary, clean_binary, cv::MORPH_OPEN, kernel);
    // kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
    // cv::morphologyEx(combined_binary, clean_binary, cv::MORPH_DILATE, kernel);

    // // seperate components
    // cv::Mat labels, stats, centroids;
    // int connectivity = 8;
    // int num_components = cv::connectedComponentsWithStats(clean_binary, labels, stats, centroids, connectivity);

    // // display results
    // std::vector<cv::Vec3b> colors(num_components);
    // colors[0] = cv::Vec3b(0, 0, 0); // Background color
    // for (int i = 1; i < num_components; i++) {
    //     colors[i] = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
    // }
    // cv::Mat components = cv::Mat::zeros(combined_binary.size(), CV_8UC3);

    // int min_area = 1000;
    // for (int i = 1; i < num_components; i++) {
    //   cv::Rect bounding_box = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
    //                                   stats.at<int>(i, cv::CC_STAT_TOP),
    //                                   stats.at<int>(i, cv::CC_STAT_WIDTH),
    //                                   stats.at<int>(i, cv::CC_STAT_HEIGHT));
    //   // don't consider small components
    //   int area = stats.at<int>(i, cv::CC_STAT_AREA);
    //   if (area >= min_area) {
    //     components.setTo(
    //       cv::Vec3b(rand() % 256, rand() % 256, rand() % 256),
    //       labels == i
    //     );
    //     cv::Point centroid(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
    //     std::cout << "Component " << i << ": Area = " << area << ", Centroid = " << centroid << std::endl;
    //     cv::rectangle(components, bounding_box, cv::Scalar(0, 255, 0), 5);
    //     cv::circle(components, centroid, 10, cv::Scalar(0, 0, 255), 10);



    //   }
    // }

    // publish images
    std_msgs::msg::Header header;
    header.stamp = node->now();
    color_publisher->publish(
      *cv_bridge::CvImage(header, "bgr8", bgr_mat).toImageMsg().get()
    );
    hist_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", hist).toImageMsg().get()
    );
    ir_active_publisher->publish(
      *cv_bridge::CvImage(header, "mono16", ir_active_mat).toImageMsg().get()
    );
    depth_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", depth_normalized).toImageMsg().get()
    );
    red_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", red_channel).toImageMsg().get()
    );
    green_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", green_channel).toImageMsg().get()
    );
    blue_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", blue_channel).toImageMsg().get()
    );
    nir_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", nir_normalized).toImageMsg().get()
    );
    exg_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", exg_normalized).toImageMsg().get()
    );
    exg_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", exg_binary).toImageMsg().get()
    );
    nir_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", nir_binary).toImageMsg().get()
    );
    combined_binary_publisher->publish(
      *cv_bridge::CvImage(header, "mono8", combined_binary).toImageMsg().get()
    );
    // clean_binary_publisher->publish(
    //   *cv_bridge::CvImage(header, "mono8", clean_binary).toImageMsg().get()
    // );
    components_publisher->publish(
      *cv_bridge::CvImage(header, "rgb8", components).toImageMsg().get()
    );

    RCLCPP_INFO(node->get_logger(), "Published");
  //   rclcpp::spin_some(node);
  //   loop_rate.sleep();
  // }



  device.stop_cameras();
  close(serial_port);

  rclcpp::shutdown();
  return 0;
}
