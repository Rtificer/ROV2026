#ifndef ROV_CONTROL_THRUSTER_INTERFACE_HPP
#define ROV_CONTROL_THRUSTER_INTERFACE_HPP

#pragma once
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include <vector>

namespace rov_control
{
  // Hardware interface for controlling thrusters, inheriting from SystemInterface
  class ThrusterHardwareInterface : public hardware_interface::SystemInterface {
    public:
      // Macro to define shared pointer stuff
      RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterHardwareInterface)

      // Receive hardware information during initialization
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

      // Exports state and command interfaces to the ROS 2 control framework
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      // Reads the current state from the thrusters
      hardware_interface::return_type read(); // Add override when needed.

      // Writes commands to the thrusters
      hardware_interface::return_type write(); // Add override when needed.

      // Reset or initialize hardware after a change in configuration
      hardware_interface::CallbackReturn on_configure();

      // Reset or shutdown hardware in preparation for reconfiguration or shutdown
      hardware_interface::CallbackReturn on_cleanup();

      hardware_interface::CallbackReturn on_shutdown();

      hardware_interface::CallbackReturn on_activate();

      hardware_interface::CallbackReturn on_deactivate();

      hardware_interface::CallbackReturn on_error();

    private:
      // Stores the latest command values for each thruster
      std::vector<double> command_;

      // Stores the latest state values for each thruster
      std::vector<double> state_;
  };
}

#endif // ROV_CONTROL_THRUSTER_INTERFACE_HPP