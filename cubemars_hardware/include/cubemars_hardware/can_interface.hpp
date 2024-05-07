#ifndef CUBEMARS_HARDWARE__CAN_SOCKET_HPP_
#define CUBEMARS_HARDWARE__CAN_SOCKET_HPP_

#include <string>

namespace cubemars_hardware
{
/**
 * @brief CAN access via SocketCAN (Linux)
 */
class CanInterfaceSocketCAN
{
public:
  /**
   * @brief Connect to CAN bus
   * @return true on success
   */
  bool connect(std::string can_port = "can0");

private:
  /**
   * @brief SocketCAN socket number
   */
  int socket_ = -1;

  /**
   * @brief true if connected, set by read_thread()
   */
  bool is_socket_connected_ = false;

  /**
   * @brief set to true to stop the read thread and disconnect
   */
  bool is_disconnect_requested_ = false;

  /**
   * @brief Last error code from sending a message
   */
  int last_send_error_ = 0;

  /**
   * @brief Count of last error code
   */
  int last_send_error_count_ = 0;
};
}

#endif  // CUBEMARS_HARDWARE__CAN_SOCKET_HPP_