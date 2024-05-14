#include "cubemars_hardware/system.hpp"

#include <cmath>
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

  can_itf_ = info_.hardware_parameters["can_interface"];

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_mode_.resize(info_.joints.size(), control_mode_t::UNDEFINED);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    can_ids_.emplace_back(std::stoul(joint.parameters.at("can_id")));
    torque_constants_.emplace_back(std::stod(joint.parameters.at("kt")));
    erpm_conversion_.emplace_back(std::stoi(joint.parameters.at("pole_pairs")) * 
      std::stoi(joint.parameters.at("reduction_ratio")) * 60 / (2 * M_PI));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_.connect(can_itf_, can_ids_, 0xFFU)
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;


  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication active");

  return result;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_.disconnect()
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;


  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication closed");

  return result;
}

std::vector<hardware_interface::StateInterface>
CubeMarsSystemHardware::export_state_interfaces()
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
CubeMarsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type CubeMarsSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  stop_modes_.clear();
  start_modes_.clear();

  // Define allowed combination of command interfaces
  std::unordered_set<std::string> eff {"effort"};
  std::unordered_set<std::string> vel {"velocity"};
  std::unordered_set<std::string> pos {"position"};
  std::unordered_set<std::string> pos_spd {"position", "velocity", "acceleration"};
  
  std::unordered_set<std::string> joint_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // find stop modes
    stop_modes_.push_back(false);
    for (std::string key : stop_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop interface: %s", key.c_str());
      if (key.find(info_.joints[i].name))
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
    if (joint_interfaces == eff)
    {
      start_modes_.push_back(CURRENT_LOOP);
    }
    else if (joint_interfaces == vel)
    {
      start_modes_.push_back(SPEED_LOOP);
    }
    else if (joint_interfaces == pos)
    {
      start_modes_.push_back(POSITION_LOOP);
    }
    else if (joint_interfaces == pos_spd)
    {
      start_modes_.push_back(POSITION_SPEED_LOOP);
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

hardware_interface::return_type CubeMarsSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (stop_modes_[i])
    {
      switch (control_mode_[i])
      {
        case CURRENT_LOOP:
          hw_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case SPEED_LOOP:
          hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case POSITION_LOOP:
          hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case POSITION_SPEED_LOOP:
          hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
          hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
          hw_commands_accelerations_[i] = std::numeric_limits<double>::quiet_NaN();
          break;
        case UNDEFINED:
          return hardware_interface::return_type::ERROR;
      }
    }
    control_mode_[i] = start_modes_[i];
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn CubeMarsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("CubeMarsSystemHardware"), "Activating... please wait...");

  // Set some default values
  // for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  // {
  //   if (std::isnan(hw_states_positions_[i]))
  //   {
  //     hw_states_positions_[i] = 0;
  //   }
  //   if (std::isnan(hw_states_velocities_[i]))
  //   {
  //     hw_states_velocities_[i] = 0;
  //   }
  //   if (std::isnan(hw_states_efforts_[i]))
  //   {
  //     hw_states_efforts_[i] = 0;
  //   }
  //   if (std::isnan(hw_commands_positions_[i]))
  //   {
  //     hw_commands_positions_[i] = 0;
  //   }
  //   if (std::isnan(hw_commands_velocities_[i]))
  //   {
  //     hw_commands_velocities_[i] = 0;
  //   }
  //   if (std::isnan(hw_commands_accelerations_[i]))
  //   {
  //     hw_commands_accelerations_[i] = 0;
  //   }
  //   control_level_[i] = integration_level_t::UNDEFINED;
  // }

  RCLCPP_INFO(
    rclcpp::get_logger("CubeMarsSystemHardware"), "System successfully activated! %u",
    control_mode_[0]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CubeMarsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool all_ids[can_ids_.size()] = { false };
  uint32_t read_id;
  uint8_t read_data[8];
  uint8_t read_len;

  // read all buffered CAN messages
  while (can_.read_nonblocking(read_id, read_data, read_len))
  {
    if (read_data[7] != 0)
    {
      switch(read_data[7])
      {
        case 1:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor over-temperature fault.");
          break;
        case 2:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-current fault.");
          break;
        case 3:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-voltage fault.");
          break;
        case 4:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Under-voltage fault.");
          break;
        case 5:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Encoder fault.");
          break;
        case 6:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "MOSFET over-temperature fault.");
          break;
        case 7:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor stall.");
          break;
        return hardware_interface::return_type::ERROR;
      }
    }
    auto it = std::find(can_ids_.begin(), can_ids_.end(), read_id);
    if (it != can_ids_.end())
    {
      int i = std::distance(can_ids_.begin(), it);
      all_ids[i] = true;
      hw_states_positions_[i] = int16_t (read_data[0] << 8 | read_data[1]);
      hw_states_velocities_[i] = int16_t (read_data[2] << 8 | read_data[3]);
      hw_states_efforts_[i] = int16_t (read_data[4] << 8 | read_data[5]);
    }
  }

  // Check if all CAN IDs have received a message
  for (size_t i = 0; i < can_ids_.size(); i++)
  {
    if (!all_ids[i])
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"), "No CAN message received from CAN ID: %u. "
        "Lower the update rate of the controller manager or increase the update frequency of the actuator.", can_ids_[i]);
      // return hardware_interface::return_type::ERROR;
    }
    else
    {
      // Unit conversions
      hw_states_positions_[i] = hw_states_positions_[i] * 0.1 * M_PI / 180;
      hw_states_velocities_[i] = hw_states_velocities_[i] * 10 / erpm_conversion_[i];
      hw_states_efforts_[i] = hw_states_efforts_[i] * 0.01 * torque_constants_[i];
      hw_states_temperatures_[i] = read_data[6];
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "pos: %f, spd: %f, eff: %f, temp: %f",
        hw_states_positions_[i], hw_states_velocities_[i], hw_states_efforts_[i], hw_states_temperatures_[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < can_ids_.size(); i++)
  {
    hw_commands_positions_[i] = hw_states_positions_[i] + 0.01;
    hw_commands_velocities_[i] = 1.0;
    uint8_t data[4];

    switch (control_mode_[i])
    {
      case UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
      case CURRENT_LOOP:
        break;
      case SPEED_LOOP:
      {
        int32_t speed = hw_commands_velocities_[i] * erpm_conversion_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "speed: %d", speed);

        data[0] = speed >> 24;
        data[1] = speed >> 16;
        data[2] = speed >> 8;
        data[3] = speed;
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "data 3: %d", data[3]);
        can_.write_message(can_ids_[i] | SPEED_LOOP << 8, data, 4);
        break;
      }
      case POSITION_LOOP:
      {
        int32_t position = hw_commands_velocities_[i] * erpm_conversion_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "speed: %d", position);

        data[0] = position >> 24;
        data[1] = position >> 16;
        data[2] = position >> 8;
        data[3] = position;
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "data 3: %d", data[3]);
        can_.write_message(can_ids_[i] | SPEED_LOOP << 8, data, 4);
        break;
      }
      case POSITION_SPEED_LOOP:
        break;
    }

    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Got the commands pos: %.5f, vel: %.5f, acc: %.5f for joint %lu",
      hw_commands_positions_[i], hw_commands_velocities_[i], hw_commands_accelerations_[i], i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)
