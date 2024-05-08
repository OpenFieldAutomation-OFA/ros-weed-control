#include "cubemars_hardware/socket_can.hpp"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
bool SocketCanInterface::connect(std::string can_port)
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not create socket");
    return false;
  }

  // Register CAN socket number
  struct ifreq ifr;
  strcpy(ifr.ifr_name, can_port.c_str());
  ioctl(socket_, SIOCGIFINDEX, &ifr);

  // Bind CAN socket
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not bind CAN interface");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Bus test message, fails if socket is available but not usable
  struct can_frame frame;
  frame.can_id = 93U | CAN_EFF_FLAG;
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "can_id: %x", frame.can_id);
  frame.len = 5;
  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not test CAN socket");
    return false;
  }

  int nbytes;
  struct can_frame frame2;
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start read");
  nbytes = read(socket_, &frame2, sizeof(struct can_frame));
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "end read");
  if (nbytes < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not read CAN socket");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "0x%03X [%d] ",frame2.can_id, frame2.len);
  for (int i = 0; i < frame2.len; i++)
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "%02X ",frame2.data[i]);

  return true;
}
}