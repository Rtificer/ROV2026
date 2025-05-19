#include "rov_control/thruster_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rov_control
{
  // Recieve hardware information during initialization
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    command_.resize(info.joints.size(), 0.0);
    state_.resize(info.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  // Exports state and command interfaces to the ROS 2 control framework
  std::vector<hardware_interface::StateInterface> ThrusterHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ThrusterHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &command_[i]);
    }
    return interfaces;
  }

  hardware_interface::return_type ThrusterHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < state_.size(); ++i)
    {
      state_[i] = command_[i]; // Returns the expected state based on the commands given, as the ESCs do not provide feedback
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ThrusterHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < command_.size(); ++i)
    {
      // TODO: Send command_[i] to hardware
      RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster %zu: %.2f", i, command_[i]);
    }
    return hardware_interface::return_type::OK;
  }

}

PLUGINLIB_EXPORT_CLASS(rov_control::ThrusterHardwareInterface, hardware_interface::SystemInterface)
