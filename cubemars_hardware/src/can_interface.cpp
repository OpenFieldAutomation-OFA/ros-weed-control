#include "cubemars_hardware/can_interface.hpp"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
bool CanInterfaceSocketCAN::connect(std::string can_port)
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
  frame.can_id = 0x55;
  frame.can_dlc = 5;
  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not test CAN socket");
    return false;
  }

  return true;
}
}