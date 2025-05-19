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
  class ThrusterHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    // Macro to define shared pointer stuff
    RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterHardwareInterface)

    // Recieve hardware information during initialization
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    // Exports state and command interfaces to the ROS 2 control framework
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Reads the current state from the thrusters
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

    // Writes commands to the thrusters
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  private:
    // Stores the latest command values for each thruster
    std::vector<double> command_;

    // Stores the latest state values for each thruster
    std::vector<double> state_;
  };
}

#endif  // ROV_CONTROL_THRUSTER_INTERFACE_HPP