#include <cstdio>
#include <k4a/k4a.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  uint32_t count = k4a::device::get_installed_count();

  printf("Found %u connected devices:\n", count);

  // Open the first plugged in Kinect device
  k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

  std::string serialnum = device.get_serialnum();

  printf("Device serial number: %s", serialnum.c_str());

  // set color controls
  device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
    K4A_COLOR_CONTROL_MODE_MANUAL, 1);
  device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
    K4A_COLOR_CONTROL_MODE_MANUAL, 20000);
  device.set_color_control(K4A_COLOR_CONTROL_GAIN,
    K4A_COLOR_CONTROL_MODE_MANUAL, 20);
  device.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE,
    K4A_COLOR_CONTROL_MODE_MANUAL, 8000);
  // we leave contrast, saturation and sharpness on defaults

  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.camera_fps = K4A_FRAMES_PER_SECOND_5;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
  config.synchronized_images_only = true;

  device.start_cameras(&config);

  k4a::capture capture = k4a::capture::create();

  if (device.get_capture(&capture))
  {
    k4a::image color_image = capture.get_color_image();
    std::chrono::microseconds time = color_image.get_exposure();
    printf("exposure time: %ld", time.count());
    color_image.set_exposure_time(std::chrono::microseconds(100));
    k4a::image ir_image = capture.get_ir_image();
    
  }


  device.stop_cameras();

  return 0;
}
