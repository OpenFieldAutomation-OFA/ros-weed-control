#include "cubemars_hardware/system.hpp"


#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
hardware_interface::CallbackReturn CubeMarsSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_port_ = info_.hardware_parameters["can"];

//   for (const hardware_interface::ComponentInfo & joint : info_.joints)
//   {
//      TODO: parameter checks
//   }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_interface_.connect(can_port_)
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;


  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication active");

  return result;
}

std::vector<hardware_interface::StateInterface>
CubeMarsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CubeMarsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("CubeMarsSystemHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("CubeMarsSystemHardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CubeMarsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("CubeMarsSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)
