#include "rov_control/thruster_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rov_control
{

  /**
   * @brief Initialize the thruster hardware interface with hardware information.
   *
   * This method is called during the initialization phase of the hardware interface lifecycle.
   * It checks if all required parameters are set and valid by calling the base class implementation.
   * If successful, it initializes the command and state vectors to zero, with a length equal to the number of thruster joints.
   *
   * @param info The hardware information structure containing joint and interface definitions.
   * @return hardware_interface::CallbackReturn Returns SUCCESS if initialization was successful, ERROR otherwise.
   */
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

  /**
   * @brief Export state interfaces for the thruster hardware.
   *
   * This method creates and returns a vector of state interfaces for each thruster joint,
   * as defined in the description/urdf/ROV2026.urdf.xacro. Each interface allows the ROS 2 control framework
   * to read the current effort (state) values from the corresponding thruster.
   *
   * @return std::vector<hardware_interface::StateInterface> A vector containing the state interfaces for all thrusters.
   */
  std::vector<hardware_interface::StateInterface> ThrusterHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_[i]);
    }
    return interfaces;
  }

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

  /**
   * @brief Read the current state of the thruster hardware.
   *
   * This method updates the internal state vector to reflect the current state of each thruster.
   * Since the ESCs do not provide feedback, the state is set to match the last command sent.
   * This allows the ROS 2 control framework to assume the thrusters are following the commanded values.
   *
   * @return hardware_interface::return_type Returns OK after updating the state.
   */
  hardware_interface::return_type ThrusterHardwareInterface::read() {
    for (size_t i = 0; i < state_.size(); ++i)
    {
      state_[i] = command_[i]; // Returns the expected state based on the commands given, as the ESCs do not provide feedback
    }
    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Write the current command values to the thruster hardware.
   *
   * This method sends the command values stored in the internal command vector to the thruster hardware.
   * For each thruster, the corresponding command value is sent to the hardware interface.
   * Currently, this implementation logs the command values for each thruster. 
   * TODO: Replace/add to the logging with actual hardware communication as needed.
   *
   * @return hardware_interface::return_type Returns OK after sending the commands.
   */
  hardware_interface::return_type ThrusterHardwareInterface::write() {
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
   * interface for activation. 
   * TODO: Initialization or setup hardware required before activation.
   *
   * @return hardware_interface::CallbackReturn Returns SUCCESS if configuration was successful, ERROR otherwise.
   */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_configure() {
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
   * TODO: Cleanup or release hardware resources in preparation for reconfiguration or shutdown.
   *
   * @return hardware_interface::CallbackReturn Returns SUCCESS if cleanup was successful, ERROR otherwise.
   */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_cleanup() {
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Shutdown the thruster hardware interface.
   *
   * This method is called during the transition to the finalized (shutdown) state in the ROS 2 lifecycle.
   * It is responsible for safely stopping all thrusters, releasing hardware resources, and resetting
   * the command and state vectors to zero. This ensures the hardware is left in a safe state before
   * the node is destroyed or the process exits.
   * TODO: Shutdown hardware.
   *
   * @return hardware_interface::CallbackReturn Returns SUCCESS if shutdown was successful, ERROR otherwise.
   */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_shutdown() {
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Activate the thruster hardware interface.
   *
   * This method is called during the transition from the inactive state to the active state
   * in the ROS 2 lifecycle. It is responsible for preparing the hardware interface to start
   * accepting and executing commands.
   * TODO: Activate or enable hardware.
   *
   * @return hardware_interface::CallbackReturn Returns SUCCESS if activation was successful, ERROR otherwise.
   */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_activate() {
    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware activated up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
 * @brief Deactivate the thruster hardware interface.
  *
  * This method is called during the transition from the active state to the inactive state
  * in the ROS 2 lifecycle. It is responsible for stopping the hardware interface from accepting
  * and executing commands. We reset the state and command vectors to zero to ensure safe deactivation.
  * TODO: Deactivate or disable hardware.
  *
  * @return hardware_interface::CallbackReturn Returns SUCCESS if deactivation was successful, ERROR otherwise.
  */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_deactivate() {
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware deactivated up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Handle error state for the thruster hardware interface.
   *
   * This method is called when the hardware interface enters the error state in the ROS 2 lifecycle.
   * It is responsible for putting the hardware into a safe state, such as stopping all thrusters by
   * setting the command and state vectors to zero, and logging the error. This ensures that the hardware
   * does not continue operating in an unsafe or undefined state after an error has occurred.
   *
   * @return hardware_interface::CallbackReturn Returns ERROR to indicate the hardware is in an error state.
   */
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_error() {
    // Handle error state: log the error and leave hardware in a safe state.
    // Do not reset command or state vectors, as this may interfere with debugging or recovery.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardwareInterface"), "Thruster hardware encountered an error state! All thrusters stopped.");

    // Return ERROR to indicate the hardware is in an error state.
    return hardware_interface::CallbackReturn::ERROR;
  }
}

PLUGINLIB_EXPORT_CLASS(rov_control::ThrusterHardwareInterface, hardware_interface::SystemInterface)
