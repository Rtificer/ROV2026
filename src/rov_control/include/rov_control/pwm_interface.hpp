#ifndef ROV_CONTROL_PWM_INTERFACE_HPP
#define ROV_CONTROL_PWM_INTERFACE_HPP

#pragma once
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include <vector>

namespace rov_control
{
  // Hardware interface for controlling pwm components inheriting from SystemInterface
  class PWMInterface : public hardware_interface::SystemInterface
  {
  public:
    // Macro to define shared pointer stuff
    RCLCPP_SHARED_PTR_DEFINITIONS(PWMInterface)

    /**
     * @brief Initialize the pwm hardware interface with hardware information.
     *
     * This method is called during the initialization phase of the hardware interface lifecycle.
     * It checks if all required parameters are set and valid by calling the base class implementation.
     * If successful, it initializes the command and state vectors to zero, with a length equal to the number of pwm joints.
     *
     * @param info The hardware information structure containing joint and interface definitions.
     * @return hardware_interface::CallbackReturn Returns SUCCESS if initialization was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    /**
     * @brief Export state interfaces for the pwm hardware.
     *
     * This method creates and returns a vector of state interfaces for each pwm joint,
     * as defined in the description/urdf/ROV2026.urdf.xacro. Each interface allows the ROS 2 control framework
     * to read the current effort (state) values from the corresponding pwm component.
     *
     * @return std::vector<hardware_interface::StateInterface> A vector containing the state interfaces for all pwm components.
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * @brief Export command interfaces for the pwm hardware.
     *
     * This method creates and returns a vector of command interfaces for each pwm joint,
     * as defined in the description/urdf/ROV2026.urdf.xacro. Each interface allows the ROS 2 control framework to send
     * command values to the corresponding pwm component.
     *
     * @return std::vector<hardware_interface::CommandInterface> A vector containing the command interfaces for all pwm components.
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * @brief Read the current state of the pwm hardware.
     *
     * This method updates the internal state vector to reflect the current state of each pwm component.
     *
     * @return hardware_interface::return_type Returns OK after updating the state.
     */
    hardware_interface::return_type read(); // Add override when needed.

    /**
     * @brief Write the current command values to the pwm hardware.
     *
     * This method sends the command values stored in the internal command vector to each pwm component.
     * For each pwm component, the corresponding command value is converted to PCA9685 ticks using the configured
     * PWM parameters (min, max, mid pulse widths, and frequency). The pulse is always set to start at the
     * beginning of the PWM cycle (on=0), and the width is set by the calculated ticks value (off=ticks).
     *
     * The function calls pca9685_write_channel() for each pwm channel:
     *   - The third argument (on=0) means the PWM pulse starts at the beginning of the cycle.
     *   - The fourth argument (off=ticks) sets the pulse width.
     * If writing to any channel fails, an error is logged and the function returns ERROR.
     * Otherwise, it logs the command sent to each pwm component and returns OK.
     *
     * @return hardware_interface::return_type Returns OK after sending the commands, or ERROR if any write fails.
     */
    hardware_interface::return_type write(); // Add override when needed.

    /**
     * @brief Configure the pwm hardware interface.
     *
     * This method is called during the transition from the unconfigured state to the inactive state
     * in the ROS 2 lifecycle. It resets the command and state vectors to zero and prepares the hardware
     * interface for activation. 
     * TODO: Initialization or setup hardware required before activation.
     *
     * @return hardware_interface::CallbackReturn Returns SUCCESS if configuration was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_configure();

    /**
     * @brief Cleanup the pwm hardware interface.
     *
     * This method is called during the transition from the inactive state to the unconfigured state
     * in the ROS 2 lifecycle. It resets the command and state vectors to zero and releases any resources
     * allocated during configuration or activation. This prepares the hardware interface for possible
     * reconfiguration or safe shutdown.
     * TODO: Cleanup or release hardware resources in preparation for reconfiguration or shutdown.
     *
     * @return hardware_interface::CallbackReturn Returns SUCCESS if cleanup was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_cleanup();

    /**
     * @brief Shutdown the pwm hardware interface.
     *
     * This method is called during the transition to the finalized (shutdown) state in the ROS 2 lifecycle.
     * It is responsible for safely stopping all pwm components, releasing hardware resources, and resetting
     * the command and state vectors to zero. This ensures the hardware is left in a safe state before
     * the node is destroyed or the process exits.
     * TODO: Shutdown hardware.
     *
     * @return hardware_interface::CallbackReturn Returns SUCCESS if shutdown was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_shutdown();

    /**
     * @brief Activate the pwm hardware interface.
     *
     * This method is called during the transition from the inactive state to the active state
     * in the ROS 2 lifecycle. It is responsible for preparing the hardware interface to start
     * accepting and executing commands.
     * TODO: Activate or enable hardware.
     *
     * @return hardware_interface::CallbackReturn Returns SUCCESS if activation was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_activate();

    /**
     * @brief Deactivate the pwm hardware interface.
     *
     * This method is called during the transition from the active state to the inactive state
     * in the ROS 2 lifecycle. It is responsible for stopping the hardware interface from accepting
     * and executing commands. We reset the state and command vectors to zero to ensure safe deactivation.
     * TODO: Deactivate or disable hardware.
     *
     * @return hardware_interface::CallbackReturn Returns SUCCESS if deactivation was successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_deactivate();

    /**
     * @brief Handle error state for the pwm hardware interface.
     *
     * This method is called when the hardware interface enters the error state in the ROS 2 lifecycle.
     * It is responsible for putting the hardware into a safe state, such as stopping all pwm compoents by
     * setting the command and state vectors to zero, and logging the error. This ensures that the hardware
     * does not continue operating in an unsafe or undefined state after an error has occurred.
     *
     * @return hardware_interface::CallbackReturn Returns ERROR to indicate the hardware is in an error state.
     */
    hardware_interface::CallbackReturn on_error();

  private:
    // Stores the latest command values for each pwm component
    std::vector<double> command_;

    // Stores the latest state values for each pwm component.
    std::vector<double> state_;

    uint16_t pwm_freq_hz_{50}; // PWM frequency in Hz

    uint16_t pwm_min_µs_{1000}; // Minimum pulse width in microseconds
    uint16_t pwm_max_µs_{2000}; // Maximum pulse width in microseconds
    uint16_t pwm_mid_µs_{1500}; // Mid pulse width in microseconds

    /**
     * @brief Loads hardware info.
     *
     * This method reads the PWM-related parameters (frequency, minimum, maximum, and midpoint pulse widths)
     * from the provided hardware_interface::HardwareInfo structure. 
     * If a parameter is present in the hardware parameters map, its value is parsed and assigned to the corresponding member variable. 
     * If a parameter is not present, the existing value (typically the default) is retained.
     *
     * @param info The hardware information structure containing hardware parameters.
     */
    void load_parameters(const hardware_interface::HardwareInfo &info);

    /**
     * @brief Convert a normalized pwm command to PCA9685 ticks.
     *
     * This function maps a normalized command value (-1 to 1)
     * to a PWM pulse width in microseconds, then converts that pulse width to PCA9685 ticks.
     * The mapping uses the configured minimum, maximum, and midpoint pulse widths.
     *
     * The mapping formula is:
     *   pulse_µs = pwm_mid_µs + command * (command >= 0 ? (pwm_max_µs - pwm_mid_µs) : (pwm_mid_µs - pwm_min_µs))
     *
     * - For command = 0.0, returns the midpoint pulse width (neutral/stop).
     * - For command > 0.0, linearly interpolates between midpoint and maximum (forward thrust).
     * - For command < 0.0, linearly interpolates between midpoint and minimum (reverse thrust).
     * - The command value is clamped to [-1.0, 1.0] for safety.
     *
     * For our Blue Robotics Basic ESCs, and with most other ESCs, the mapping is:
     *   - Minimum pulse (e.g., 1100 µs) = full reverse
     *   - Midpoint pulse (e.g., 1500 µs) = stop
     *   - Maximum pulse (e.g., 1900 µs) = full forward
     *
     * @param command The normalized pwm command [-1.0, 1.0].
     * @param pwm_min_µs Minimum pulse width in microseconds.
     * @param pwm_max_µs Maximum pulse width in microseconds.
     * @param pwm_mid_µs Midpoint pulse width in microseconds.
     * @param pwm_freq_hz PWM frequency in Hertz.
     * @return The number of PCA9685 ticks corresponding to the command.
     */
    uint16_t command_to_ticks(
        double command,
        uint16_t pwm_min_µs,
        uint16_t pwm_max_µs,
        uint16_t pwm_mid_µs,
        uint16_t pwm_freq_hz);
  };
}

#endif // ROV_CONTROL_PWM_INTERFACE_HPP