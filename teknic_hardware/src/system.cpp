#include "teknic_hardware/system.hpp"

#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define TIME_TILL_TIMEOUT	10000

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

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_mode_.resize(info_.joints.size(), control_mode_t::UNDEFINED);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.parameters.count("port") != 0 &&
      joint.parameters.count("node") != 0)
    {
      std::pair<std::size_t, std::size_t> node;
      std::string port = joint.parameters.at("port");
      auto it = std::find(chports.begin(), chports.end(), port);
      if (it != chports.end())
      {
        node.first = std::distance(chports.begin(), it);
      }
      else
      {
        node.first = chports.size();
        chports.emplace_back(port);
      }
      node.second = std::stoul(joint.parameters.at("node"));
      nodes.emplace_back(node);
    }
    else
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Missing parameters in URDF for %s", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    for (size_t pc = 0; pc < chports.size(); pc++)
    {
      myMgr->ComHubPort(pc, chports[pc].c_str());
    }
    myMgr->PortsOpen(chports.size());
    for (size_t i = 0; i < chports.size(); i++) {
      sFnd::IPort &myPort = myMgr->Ports(i);
      RCLCPP_INFO(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Port[%d]: state=%d, nodes=%d",
        myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
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
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  stop_modes_.clear();
  start_modes_.clear();

  // Define allowed combination of command interfaces
  std::unordered_set<std::string> vel {"velocity"};
  std::unordered_set<std::string> pos {"position"};
  
  std::unordered_set<std::string> joint_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // find stop modes
    stop_modes_.push_back(false);
    for (std::string key : stop_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        stop_modes_[i] = true;
        break;
      }
    }

    // find start modes
    joint_interfaces.clear();
    for (std::string key : start_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        joint_interfaces.insert(key.substr(key.find("/") + 1));
      }
    }
    if (joint_interfaces == vel)
    {
      start_modes_.push_back(SPEED_LOOP);
    }
    else if (joint_interfaces == pos)
    {
      start_modes_.push_back(POSITION_LOOP);
    }
    else if (joint_interfaces.empty())
    {
      // don't change control mode
      start_modes_.push_back(control_mode_[i]);
    }
    else
    {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TeknicSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (stop_modes_[i])
    {
      switch (control_mode_[i])
      {
        case SPEED_LOOP:
          hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case POSITION_LOOP:
          hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case UNDEFINED:
          return hardware_interface::return_type::ERROR;
      }
    }
    // switch control mode
    control_mode_[i] = start_modes_[i];
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn TeknicSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    for (const std::pair<std::size_t, std::size_t> & node : nodes)
    {
      // enable node
      sFnd::INode &inode = myMgr->Ports(node.first).Nodes(node.second);
      RCLCPP_INFO(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Node[%zu]: type=%d\nuserID: %s\nFW version: %s\nSerial #: %d\nModel: %s\n",
        node.first, inode.Info.NodeType(), inode.Info.UserID.Value(),
        inode.Info.FirmwareVersion.Value(), inode.Info.SerialNumber.Value(),
        inode.Info.Model.Value());
      inode.Status.AlertsClear();
      inode.Motion.NodeStopClear();
      inode.EnableReq(true);
      double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
      while (!inode.Motion.IsReady()) {
        if (myMgr->TimeStampMsec() > timeout) {
          if (inode.Status.Power.Value().fld.InBusLoss) {
            RCLCPP_ERROR(
              rclcpp::get_logger("TeknicSystemHardware"),
              "Bus Power low");
            return hardware_interface::CallbackReturn::ERROR;
          }
          RCLCPP_ERROR(
            rclcpp::get_logger("TeknicSystemHardware"),
            "Timed out waiting for Node %zu to enable", node.first);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      RCLCPP_INFO(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Node %zu enabled", node.first);
      
      // TODO: homing
      //   //At this point the Node is enabled, and we will now check to see if the Node has been homed
      //   //Check the Node to see if it has already been homed, 
      //   if (theNode.Motion.Homing.HomingValid())
      //   {
      //     if (theNode.Motion.Homing.WasHomed())
      //     {
      //       printf("Node %d has already been homed, current position is: \t%8.0f \n", iNode, theNode.Motion.PosnMeasured.Value());
      //       printf("Rehoming Node... \n");
      //     }
      //     else
      //     {
      //       printf("Node [%d] has not been homed.  Homing Node now...\n", iNode);
      //     }
      //     //Now we will home the Node
      //     theNode.Motion.Homing.Initiate();

      //     timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
      //                                 // Basic mode - Poll until disabled
      //     while (!theNode.Motion.Homing.WasHomed()) {
      //       if (myMgr->TimeStampMsec() > timeout) {
      //                       if (IsBusPowerLow(theNode)) {
      //                           printf("Error: Bus Power low. Make sure 75V supply is powered on.\n");
      //                           msgUser("Press any key to continue.");
      //                           return -1;
      //                       }
      //         printf("Node did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n");
      //         msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
      //         return -2;
      //       }
      //     }
      //     printf("Node completed homing\n");
      //   }
      //   else {
      //     printf("Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
      //   }
    }
  }
  catch(sFnd::mnErr& theErr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeknicSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    for (const std::pair<std::size_t, std::size_t> & node : nodes)
    {
      // disable node
      sFnd::INode &inode = myMgr->Ports(node.first).Nodes(node.second);
      RCLCPP_INFO(
        rclcpp::get_logger("TeknicSystemHardware"),
        "Disabling Node %zu", node.first);
      inode.EnableReq(false);
    }
  }
  catch(sFnd::mnErr& theErr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeknicSystemHardware"),
      "Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
    return hardware_interface::CallbackReturn::ERROR;
  }
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
