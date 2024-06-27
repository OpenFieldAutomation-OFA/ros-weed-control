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
  try
  {
    // detect all ports
	  std::vector<std::string> comHubPorts;
		sFnd::SysManager::FindComHubPorts(comHubPorts);
    RCLCPP_INFO(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Found %ld SC Hubs\n", comHubPorts.size());
		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++)
    {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
		}
    if (portCount > 0) {
			myMgr->PortsOpen(portCount);
			for (size_t i = 0; i < portCount; i++) {
				sFnd::IPort &myPort = myMgr->Ports(i);
        RCLCPP_INFO(
          rclcpp::get_logger("TeknicSystemHardware"),
          "Port[%d]: state=%d, nodes=%d",
          myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
			}
		}
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Unable to locate SC hub port");
      return hardware_interface::CallbackReturn::FAILURE;
    }

    // clear all alerts
    for (size_t iPort = 0; iPort < portCount; iPort++) {
			sFnd::IPort &myPort = myMgr->Ports(iPort);
			char alertList[256];
      RCLCPP_INFO(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Checking for Alerts");
			for (unsigned iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				sFnd::INode &theNode = myPort.Nodes(iNode);
				theNode.Status.RT.Refresh();
				theNode.Status.Alerts.Refresh();
        RCLCPP_INFO(
          rclcpp::get_logger("TeknicSystemHardware"),
          "Checking node %i for Alerts", iNode);
				if (!theNode.Status.RT.Value().cpm.AlertPresent) {
          RCLCPP_INFO(
            rclcpp::get_logger("TeknicSystemHardware"),
            "Node has no alerts");
				}
				if (theNode.Status.HadTorqueSaturation()) {
          RCLCPP_INFO(
            rclcpp::get_logger("TeknicSystemHardware"),
            "Node has experienced torque saturation since last checking");
				}
				if (theNode.Status.Alerts.Value().isInAlert()) {
					theNode.Status.Alerts.Value().StateStr(alertList, 256);
          RCLCPP_INFO(
            rclcpp::get_logger("TeknicSystemHardware"),
            "Node has alerts! Alerts:\n%s\n", alertList);
					if (theNode.Status.Alerts.Value().cpm.Common.EStopped) {
            RCLCPP_INFO(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Node is e-stopped: Clearing E-Stop");
						theNode.Motion.NodeStopClear();
					}
					if (theNode.Status.Alerts.Value().cpm.TrackingShutdown) {
            RCLCPP_INFO(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Node exceeded Tracking error limit");
					}
					theNode.Status.Alerts.Refresh();
					if (theNode.Status.Alerts.Value().isInAlert()) {
						theNode.Status.Alerts.Value().StateStr(alertList, 256);
            RCLCPP_INFO(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Node has non-estop alerts: %s", alertList);
            RCLCPP_INFO(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Clearing non-serious alerts");
						theNode.Status.AlertsClear();
						theNode.Status.Alerts.Refresh();
						if (theNode.Status.Alerts.Value().isInAlert()) {
							theNode.Status.Alerts.Value().StateStr(alertList, 256);
              RCLCPP_INFO(
                rclcpp::get_logger("TeknicSystemHardware"),
                "Node has serious, non-clearing alerts: %s", alertList);
						}
						else {
              RCLCPP_INFO(
                rclcpp::get_logger("TeknicSystemHardware"),
                "Node %d: all alerts have been cleared", theNode.Info.Ex.Addr());
						}
					}
					else {
            RCLCPP_INFO(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Node %d: all alerts have been cleared\n", theNode.Info.Ex.Addr());
					}

				}
			}
    }
  }
  catch(sFnd::mnErr& theErr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		myMgr->PortsClose();
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("TeknicSystemHardware"), "Communication active");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
  	myMgr->PortsClose();
  }
  catch(sFnd::mnErr& theErr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
    return hardware_interface::CallbackReturn::FAILURE;
  }
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
