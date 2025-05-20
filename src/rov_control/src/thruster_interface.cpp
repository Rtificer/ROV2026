#include "rov_control/thruster_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rov_control
{

  // Receive hardware information during initialization
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
    // Check if all required parameters are set and valid.
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Initialize a vector of command and state values to 0 where the length of the vector is equal to the number of joints (thrusters)
    command_.resize(info.joints.size(), 0.0);
    state_.resize(info.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  //Exports state interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
  std::vector<hardware_interface::StateInterface> ThrusterHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_[i]);
    }
    return interfaces;
  }

   //Exports command interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
/**
 * @brief Export command interfaces for the thruster hardware.
 *
 * This method creates and returns a vector of command interfaces for each thruster joint,
 * as defined in the description/urdf/ROV2026.urdf.xacro. Each interface allows the ROS 2 control framework to send
 * effort (command) values to the corresponding thruster.
 *
 * @return std::vector<hardware_interface::CommandInterface> A vector containing the command interfaces for all thrusters.
 */
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

/**
 * @brief Configure the thruster hardware interface.
 *
 * This method is called during the transition from the unconfigured state to the inactive state
 * in the ROS 2 lifecycle. It resets the command and state vectors to zero and prepares the hardware
 * interface for activation. Any hardware initialization or setup required before activation should
 * be performed here.
 *
 * @param previous_state The previous lifecycle state before configuration.
 * @return hardware_interface::CallbackReturn Returns SUCCESS if configuration was successful, ERROR otherwise.
 */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: Reset or initialize or reset hardware here.

    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }


/**
 * @brief Cleanup the thruster hardware interface.
 *
 * This method is called during the transition from the inactive state to the unconfigured state
 * in the ROS 2 lifecycle. It resets the command and state vectors to zero and releases any resources
 * allocated during configuration or activation. This prepares the hardware interface for possible
 * reconfiguration or safe shutdown.
 *
 * @param previous_state The previous lifecycle state before cleanup.
 * @return hardware_interface::CallbackReturn Returns SUCCESS if cleanup was successful, ERROR otherwise.
 */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: Reset hardware here in preparation for reconfiguration or shutdown.
    
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);


    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
}

PLUGINLIB_EXPORT_CLASS(rov_control::ThrusterHardwareInterface, hardware_interface::SystemInterface)
