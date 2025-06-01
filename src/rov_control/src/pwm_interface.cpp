/**
 * @file pwm_interface.cpp
 * @brief Implementation of the PWMInterface for ROV2026.
 */

#include "rov_control/pwm_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <algorithm>

// Include the LibDriver PCA9685 header (its a c library so extern "C" is nessasary)
extern "C"
{
#include "rov_control/libdriver_pca9685/driver_pca9685.h"
#include "rov_control/libdriver_pca9685/driver_pca9685_interface.h"
}

namespace rov_control
{
  static pca9685_handle_t pca9685_handle;

  static uint16_t command_to_ticks(
      double command,
      uint16_t pwm_min_µs,
      uint16_t pwm_max_µs,
      uint16_t pwm_mid_µs,
      uint16_t pwm_freq_hz)
  {
    command = std::clamp(command, -1.0, 1.0);
    // Map command to pulse width
    uint16_t pulse_µs = static_cast<uint16_t>(pwm_mid_µs + command * (command >= 0 ? (pwm_max_µs - pwm_mid_µs) : (pwm_mid_µs - pwm_min_µs)));
    // Convert pulse width to ticks
    return (pulse_µs * 4096) / (1000000UL / pwm_freq_hz);
  }

  void PWMInterface::load_parameters(const hardware_interface::HardwareInfo &info)
  {
    auto param = info.hardware_parameters.find("pwm_freq_hz");
    if (param != info.hardware_parameters.end())
    {
      pwm_freq_hz_ = static_cast<uint16_t>(std::stoi(param->second));
    }

    param = info.hardware_parameters.find("pwm_min_µs");
    if (param != info.hardware_parameters.end())
    {
      pwm_min_µs_ = static_cast<uint16_t>(std::stoi(param->second));
    }

    param = info.hardware_parameters.find("pwm_max_µs");
    if (param != info.hardware_parameters.end())
    {
      pwm_max_µs_ = static_cast<uint16_t>(std::stoi(param->second));
    }

    param = info.hardware_parameters.find("pwm_mid_µs");
    if (param != info.hardware_parameters.end())
    {
      pwm_mid_µs_ = static_cast<uint16_t>(std::stoi(param->second));
    }
  }

  // Receive hardware information during initialization
  hardware_interface::CallbackReturn PWMInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    // Check if all required parameters are set and valid.
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Initialize a vector of command and state values to 0 where the length of the vector is equal to the number of joints
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
      RCLCPP_ERROR(rclcpp::get_logger("PWMInterface"), "PCA9685 init failed!");
      return CallbackReturn::ERROR;
    }

    load_parameters(info);

    uint8_t prescaler_reg = 0;
    if (pca9685_output_frequency_convert_to_register(&pca9685_handle, PCA9685_OSCILLATOR_INTERNAL_FREQUENCY, pwm_freq_hz_, &prescaler_reg) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PWMInterface"), "Failed to convert frequency to prescaler!");
      return CallbackReturn::ERROR;
    }
    if (pca9685_set_prescaler(&pca9685_handle, prescaler_reg) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PWMInterface"), "Set PWM prescaler failed!");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // Exports state interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
  std::vector<hardware_interface::StateInterface> PWMInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_[i]);
    }
    for (size_t i = 8; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_[i]);
    }
    return interfaces;
  }

  // Exports command interfaces based on the interfaces defined in description/urdf/ROV2026.urdf.xacro
  std::vector<hardware_interface::CommandInterface> PWMInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size() - 3; ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &command_[i]);
    }
    for (size_t i = 7; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_[i]);
    }
    return interfaces;
  }

  // Reads the current state from the thrusters
  hardware_interface::return_type PWMInterface::read(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < state_.size(); ++i)
    {
      state_[i] =
          command_[i]; // Returns the expected state based on the commands given, as the ESCs do not provide feedback
    }
    return hardware_interface::return_type::OK;
  }

  // Writes commands to the thrusters
  hardware_interface::return_type PWMInterface::write(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < command_.size(); ++i)
    {
      uint16_t ticks = command_to_ticks(command_[i], pwm_min_µs_, pwm_max_µs_, pwm_mid_µs_, pwm_freq_hz_);
      if (pca9685_write_channel(&pca9685_handle, static_cast<pca9685_channel_t>(i), 0, ticks) != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("PWMInterface"), "Failed to set PWM for thruster %zu", i);
        return hardware_interface::return_type::ERROR;
      }
      if (i < 8)
      {
        RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster %zu: %.2f", i + 1, command_[i]);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Arm Actuator: %zu: %.2f", i - 7, command_[i]);
      }
    }
    return hardware_interface::return_type::OK;
  }

  // Reset or initialize hardware after a change in configuration
  hardware_interface::CallbackReturn PWMInterface::on_configure()
  {
    // TODO: Reset or initialize or reset hardware here.

    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    // Try to read pwm_freq_hz from hardware parameters, default to 50 if not set
    load_parameters(info_);

    RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster hardware configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Reset or shutdown hardware in preparation for reconfiguration or shutdown
  hardware_interface::CallbackReturn PWMInterface::on_cleanup()
  {
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn PWMInterface::on_shutdown()
  {
    // TODO: Reset hardware here in preparation for reconfiguration or shutdown.

    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn PWMInterface::on_activate()
  {

    RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster hardware activated up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn PWMInterface::on_deactivate()
  {
    // Reset command and state vectors to zero.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("PWMInterface"), "Thruster hardware deactivated up.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn PWMInterface::on_error()
  {
    // Handle error state: log the error and leave hardware in a safe state.
    // Do not reset command or state vectors, as this may interfere with debugging or recovery.
    std::fill(command_.begin(), command_.end(), 0.0);
    std::fill(state_.begin(), state_.end(), 0.0);

    RCLCPP_ERROR(rclcpp::get_logger("PWMInterface"), "Thruster hardware encountered an error state! All thrusters stopped.");

    // Return ERROR to indicate the hardware is in an error state.
    return hardware_interface::CallbackReturn::ERROR;
  }
} // namespace rov_control

PLUGINLIB_EXPORT_CLASS(rov_control::PWMInterface, hardware_interface::SystemInterface)
