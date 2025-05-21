#include "rov_control/thruster_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Include the LibDriver PCA9685 header (its a c library so extern "C" is nessasary)
extern "C"
{
#include "rov_control/driver_pca9685.h"
#include "rov_control/driver_pca9685_interface.h"
}

namespace rov_control
{
  static pca9685_handle_t pca9685_handle;

   static uint16_t milliseconds_to_ticks(uint16_t pulse_us, uint16_t pwm_freq_hz) {
    // 1 cycle = 20,000 us (50Hz), 4096 steps
    return (pulse_us * 4096) / (1000000UL / pwm_freq_hz);
  }

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

    // Initialize a vector of command and state values to 0 where the length of the vector is equal to the number of
    // joints (thrusters)
    command_.resize(info.joints.size(), 0.0);
    state_.resize(info.joints.size(), 0.0);

    // Link interface functions
    pca9685_handle.debug_print = pca9685_interface_debug_print;
    pca9685_handle.delay_ms = pca9685_interface_delay_ms;
    pca9685_handle.iic_init = pca9685_interface_iic_init;
    pca9685_handle.iic_deinit = pca9685_interface_iic_deinit;
    pca9685_handle.iic_read = pca9685_interface_iic_read;
    pca9685_handle.iic_write = pca9685_interface_iic_write;
    pca9685_handle.oe_gpio_init = pca9685_interface_oe_init;
    pca9685_handle.oe_gpio_deinit = pca9685_interface_oe_deinit;
    pca9685_handle.oe_gpio_write = pca9685_interface_oe_write;

    // Set I2C address (default 0x40)
    pca9685_set_addr(&pca9685_handle, 0x40);

    // Initialize PCA9685
    if (pca9685_init(&pca9685_handle) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardwareInterface"), "PCA9685 init failed!");
      return CallbackReturn::ERROR;
    }

    // Try to read pwm_freq_hz from hardware parameters, default to 50 if not set
    auto it = info_.hardware_parameters.find("pwm_freq_hz");
    if (it != info_.hardware_parameters.end()) {
        pwm_freq_hz_ = static_cast<uint16_t>(std::stoi(it->second));
    } else {
        pwm_freq_hz_ = 50;
    }


    uint8_t prescaler_reg = 0;
    if (pca9685_output_frequency_convert_to_register(&pca9685_handle, PCA9685_OSCILLATOR_INTERNAL_FREQUENCY, pwm_freq_hz_, &prescaler_reg) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardwareInterface"), "Failed to convert frequency to prescaler!");
      return CallbackReturn::ERROR;
    }
    if (pca9685_set_prescaler(&pca9685_handle, prescaler_reg) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardwareInterface"), "Set PWM prescaler failed!");
      return CallbackReturn::ERROR;
    }

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
  // Exports state interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
  std::vector<hardware_interface::StateInterface> ThrusterHardwareInterface::export_state_interfaces()
  {
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
  // Exports command interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
  /**
   * @brief Export command interfaces for the thruster hardware.
   *
   * This method creates and returns a vector of command interfaces for each thruster joint,
   * as defined in the description/urdf/ROV2026.urdf.xacro. Each interface allows the ROS 2 control framework to send
   * effort (command) values to the corresponding thruster.
   *
   * @return std::vector<hardware_interface::CommandInterface> A vector containing the command interfaces for all
   * thrusters.
   */
  std::vector<hardware_interface::CommandInterface> ThrusterHardwareInterface::export_command_interfaces()
  {
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
  hardware_interface::return_type ThrusterHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < state_.size(); ++i)
    {
      state_[i] =
          command_[i]; // Returns the expected state based on the commands given, as the ESCs do not provide feedback
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
  hardware_interface::return_type ThrusterHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
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
  hardware_interface::CallbackReturn
  ThrusterHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO: Reset or initialize or reset hardware here.

    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    // Try to read pwm_freq_hz from hardware parameters, default to 50 if not set
    auto it = info_.hardware_parameters.find("pwm_freq_hz");
    if (it != info_.hardware_parameters.end()) {
        pwm_freq_hz_ = static_cast<uint16_t>(std::stoi(it->second));
    } else {
        pwm_freq_hz_ = 50;
    }

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
  hardware_interface::CallbackReturn
  ThrusterHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO: Reset hardware here in preparation for reconfiguration or shutdown.

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
} // namespace rov_control

PLUGINLIB_EXPORT_CLASS(rov_control::ThrusterHardwareInterface, hardware_interface::SystemInterface)
