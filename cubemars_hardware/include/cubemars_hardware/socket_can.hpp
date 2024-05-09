#ifndef CUBEMARS_HARDWARE__SOCKET_CAN_HPP_
#define CUBEMARS_HARDWARE__SOCKET_CAN_HPP_

#include <string>

namespace cubemars_hardware
{

/**
 * @brief SocketCAN interface for extended frame format
 */
class SocketCanInterface
{
public:
  /**
   * @brief Connect to CAN bus
   * @return true on success
   */
  bool connect(std::string can_port = "can0");

  /**
   * @brief Write message to CAN bus
   * @param id CAN extended identifier
   * @param data Data to be transmitted
   * @param len Number of bytes of data (0-8)
   * @return true on success
   */
  bool write_message(uint32_t id, const uint8_t data[], uint8_t len);

  /**
   * @brief Read message from CAN bus
   * @param id CAN extended identifier
   * @param data Data to be received
   * @param len Received number of bytes of data (0-8)
   * @return true on success
   */
  bool read_message(uint32_t & id, uint8_t data[8], uint8_t & len);

private:
  /**
   * @brief SocketCAN socket number
   */
  int socket_;

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

#endif  // CUBEMARS_HARDWARE__SOCKET_CAN_HPP_