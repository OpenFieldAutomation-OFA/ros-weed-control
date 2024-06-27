#include "teknic_hardware/system.hpp"

#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace teknic_hardware
{
hardware_interface::CallbackReturn TeknicSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("TeknicSystemHardware"),
    "Hello World!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TeknicSystemHardware"),
    "Hello World2!");
	std::size_t portCount;
	std::vector<std::string> comHubPorts;
  try
  {
		sFnd::SysManager::FindComHubPorts(comHubPorts);

    RCLCPP_INFO(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Found %ld SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}

  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger("TeknicSystemHardware"),
        "sFoundation error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  

  RCLCPP_INFO(rclcpp::get_logger("TeknicSystemHardware"), "Communication active");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TeknicSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature", &hw_states_temperatures_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TeknicSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  return command_interfaces;
}

hardware_interface::return_type TeknicSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TeknicSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn TeknicSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TeknicSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TeknicSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace teknic_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  teknic_hardware::TeknicSystemHardware, hardware_interface::SystemInterface)
