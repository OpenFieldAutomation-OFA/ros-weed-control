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
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_mode_.resize(info_.joints.size(), control_mode_t::POSITION_LOOP);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    can_ids_.emplace_back(std::stoul(joint.parameters.at("can_id")));
    // int poles = std::stoi(joint.parameters.at("pole_pairs"));
    // int ratio = std::stoi(joint.parameters.at("reduction_ratio"));
    // double conversion = poles * ratio * 60 / (2 * M_PI);
    
    // RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "vars: %d, %d, %f", poles, ratio, conversion);

    erpm_conversion_.emplace_back(std::stoi(joint.parameters.at("pole_pairs")) * 
      std::stoi(joint.parameters.at("reduction_ratio")) * 60 / (2 * M_PI));
    torque_constants_.emplace_back(std::stod(joint.parameters.at("kt")));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_.connect(can_itf_)
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
  for (uint i = 0; i < info_.joints.size(); i++)
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
  // Prepare for new command modes
  std::vector<control_mode_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start_interfaces: %s", key.c_str());
  }

  for (std::string key : stop_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop_interfaces: %s", key.c_str());
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
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Reading...");

  // create map for all can_ids
  std::unordered_map<uint32_t, bool> ids_read;
  for (uint32_t & can_id : can_ids_)
  {
    ids_read.insert({can_id, false});
  }

  uint32_t read_id;
  uint8_t read_data[8];
  uint8_t read_len;

  int16_t pos_int;
  int16_t spd_int;
  int16_t cur_int;

  int i = 0;

  while (can_.read_nonblocking(read_id, read_data, read_len)) {
    read_id = read_id & 0xFF;
    ids_read.at(read_id) = true;
    
    pos_int = read_data[0] << 8 | read_data[1];
    spd_int = read_data[2] << 8 | read_data[3];
    cur_int = read_data[4] << 8 | read_data[5];

    hw_states_positions_[i] = pos_int * 0.1 * M_PI / 180;
    hw_states_velocities_[i] = spd_int * 10 / erpm_conversion_[i];
    hw_states_efforts_[i] = cur_int * 0.01 * torque_constants_[i];

    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "pos: %f, spd: %f, eff: %f",
      hw_states_positions_[i], hw_states_velocities_[i], hw_states_efforts_[i]);

  }
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Loop done");

  for (auto const & [id, val] : ids_read)
  {
    if (!val) {
      RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "No CAN message received from CAN ID: %u. "
        "Lower the update rate of the controller manager or increase the update frequency of the actuator.", id);
      return hardware_interface::return_type::ERROR;
    }
  }

  // for (uint i = 0; i < info_.joints.size(); i++)
  // {
  //   can_.read_message(read_id, read_data, read_len);
  //   RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "0x%03X [%d] ", read_id, read_len);
  //   read_id = read_id & 0xFF;
  //   // if (ids_done[read_id])
  //   // {
  //   //   RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
  //   //   "Read same CAN identifier twice in one loop iteration. Make sure all actuators use "
  //   //   "the same upload rate.");
  //   //   return hardware_interface::return_type::ERROR;
  //   // }
  //   ids_done[read_id] = true;

  //   int16_t pos_int = read_data[0] << 8 | read_data[1];
  //   int16_t spd_int = read_data[2] << 8 | read_data[3];
  //   int16_t cur_int = read_data[4] << 8 | read_data[5];

  //   hw_states_positions_[i] = pos_int * 0.1 * M_PI / 180;
  //   hw_states_velocities_[i] = spd_int * 10 / erpm_conversion_[i];
  //   hw_states_efforts_[i] = cur_int * 0.01 * torque_constants_[i];

  //   RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "pos: %f, spd: %f, cur: %f",
  //     hw_states_positions_[i], hw_states_velocities_[i], hw_states_efforts_[i]);

  //   // &motor_pos= (float)( pos_int * 0.1f); // Motor Position
  //   // &motor_spd= (float)( spd_int * 10.0f);// Motor Speed
  //   // &motor_cur= (float) ( cur_int * 0.01f);// Motor Current
  //   // &motor_temp= (rx_message)->Data[6] ;// Motor Temperature
  //   // &motor_error= (rx_message)->Data[7] ;// Motor Error Code
    
  // }

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)
