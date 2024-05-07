// credit: https://github.com/odriverobotics/ros_odrive

#include "cubemars_hardware/system.hpp"

#include "hardware_interface/system_interface.hpp"
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

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_interface_.connect(can_port_)
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;

  return result;
}

// CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
//     can_intf_.deinit();
//     return CallbackReturn::SUCCESS;
// }

// CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
//     RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

//     // This can be called several seconds before the controller finishes starting.
//     // Therefore we enable the ODrives only in perform_command_mode_switch().
//     return CallbackReturn::SUCCESS;
// }

// CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
//     RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

//     for (auto& axis : axes_) {
//         Set_Axis_State_msg_t msg;
//         msg.Axis_Requested_State = AXIS_STATE_IDLE;
//         axis.send(msg);
//     }

//     return CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
//     std::vector<hardware_interface::StateInterface> state_interfaces;

//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_EFFORT,
//             &axes_[i].torque_target_
//         ));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_VELOCITY,
//             &axes_[i].vel_estimate_
//         ));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_POSITION,
//             &axes_[i].pos_estimate_
//         ));
//     }

//     return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
//     std::vector<hardware_interface::CommandInterface> command_interfaces;

//     for (size_t i = 0; i < info_.joints.size(); i++) {
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_EFFORT,
//             &axes_[i].torque_setpoint_
//         ));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_VELOCITY,
//             &axes_[i].vel_setpoint_
//         ));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             info_.joints[i].name,
//             hardware_interface::HW_IF_POSITION,
//             &axes_[i].pos_setpoint_
//         )); 
//     }

//     return command_interfaces;
// }

// return_type ODriveHardwareInterface::perform_command_mode_switch(
//     const std::vector<std::string>& start_interfaces,
//     const std::vector<std::string>& stop_interfaces
// ) {
//     for (size_t i = 0; i < axes_.size(); ++i) {
//         Axis& axis = axes_[i];
//         std::array<std::pair<std::string, bool*>, 3> interfaces = {
//             {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
//              {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
//              {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
//         };

//         bool mode_switch = false;

//         for (const std::string& key : stop_interfaces) {
//             for (auto& kv : interfaces) {
//                 if (kv.first == key) {
//                     *kv.second = false;
//                     mode_switch = true;
//                 }
//             }
//         }

//         for (const std::string& key : start_interfaces) {
//             for (auto& kv : interfaces) {
//                 if (kv.first == key) {
//                     *kv.second = true;
//                     mode_switch = true;
//                 }
//             }
//         }

//         if (mode_switch) {
//             Set_Controller_Mode_msg_t msg;
//             if (axis.pos_input_enabled_) {
//                 RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to position control", info_.joints[i].name.c_str());
//                 msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
//                 msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
//             } else if (axis.vel_input_enabled_) {
//                 RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to velocity control", info_.joints[i].name.c_str());
//                 msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
//                 msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
//             } else {
//                 RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to torque control", info_.joints[i].name.c_str());
//                 msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
//                 msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
//             }

//             bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

//             if (any_enabled) {
//                 axis.send(msg); // Set control mode
//             }

//             // Set axis state
//             Clear_Errors_msg_t msg1;
//             msg1.Identify = 0;
//             axis.send(msg1);

//             // Set axis state
//             Set_Axis_State_msg_t msg2;
//             msg2.Axis_Requested_State = any_enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
//             axis.send(msg2);
//         }
//     }

//     return return_type::OK;
// }

// return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
//     timestamp_ = timestamp;

//     while (can_intf_.read_nonblocking()) {
//         // repeat until CAN interface has no more messages
//     }

//     return return_type::OK;
// }

// return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
//     for (auto& axis : axes_) {
//         // Send the CAN message that fits the set of enabled setpoints
//         if (axis.pos_input_enabled_) {
//             Set_Input_Pos_msg_t msg;
//             msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
//             msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_  / (2 * M_PI)) : 0.0f;
//             msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
//             axis.send(msg);
//         } else if (axis.vel_input_enabled_) {
//             Set_Input_Vel_msg_t msg;
//             msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
//             msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
//             axis.send(msg);
//         } else if (axis.torque_input_enabled_) {
//             Set_Input_Torque_msg_t msg;
//             msg.Input_Torque = axis.torque_setpoint_;
//             axis.send(msg);
//         } else {
//             // no control enabled - don't send any setpoint
//         }
//     }

//     return return_type::OK;
// }

// void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
//     for (auto& axis : axes_) {
//         if ((frame.can_id >> 5) == axis.node_id_) {
//             axis.on_can_msg(timestamp_, frame);
//         }
//     }
// }

// void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
//     uint8_t cmd = frame.can_id & 0x1f;

//     auto try_decode = [&]<typename TMsg>(TMsg& msg) {
//         if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
//             RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
//             return false;
//         }
//         msg.decode_buf(frame.data);
//         return true;
//     };

//     switch (cmd) {
//         case Get_Encoder_Estimates_msg_t::cmd_id: {
//             if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
//                 pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
//                 vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
//             }
//         } break;
//         case Get_Torques_msg_t::cmd_id: {
//             if (Get_Torques_msg_t msg; try_decode(msg)) {
//                 torque_target_ = msg.Torque_Target;
//                 torque_estimate_ = msg.Torque_Estimate;
//             }
//         } break;
//             // silently ignore unimplemented command IDs
//     }
// }

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)
