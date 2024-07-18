#include <cstdio>
#include <k4a/k4a.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  // Open the first plugged in Kinect device
  k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

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

  k4a_calibration_t calibration = device.get_calibration(config.depth_mode, config.color_resolution);
  k4a::transformation transformation(calibration);

  device.start_cameras(&config);

  k4a::capture capture = k4a::capture::create();
  
  if (device.get_capture(&capture))
  {
    k4a::image color_image = capture.get_color_image();
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();
    printf("width: %i, height: %i\n", width, height);
    k4a::image ir_image = capture.get_ir_image();
  }


  device.stop_cameras();

  return 0;
}
