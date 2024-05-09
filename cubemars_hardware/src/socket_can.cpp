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
  if (!write_message(0, NULL, 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not read CAN socket");
    return false;
  }

  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  read_message(id, data, len);
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "0x%03X [%d] ",id, len);
  for (int i = 0; i < len; i++)
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "%02X ",data[i]);
}

bool SocketCanInterface::read_message(uint32_t & id, uint8_t data[], uint8_t & len)
{
  struct can_frame frame;
  if (read(socket_, &frame, sizeof(struct can_frame)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not read CAN socket");
    return false;
  }
  memcpy(data, frame.data, frame.len);
  id = frame.can_id;
  len = frame.len;

  return true;
}

bool SocketCanInterface::write_message(uint32_t id, const uint8_t data[], uint8_t len)
{
  struct can_frame frame;
  frame.can_id = id | CAN_EFF_FLAG;
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "can_id: %x", frame.can_id);
  frame.len = len;
  memcpy(frame.data, data, len);
  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Could not write message to CAN socket");
    return false;
  }
  return true;
}
}