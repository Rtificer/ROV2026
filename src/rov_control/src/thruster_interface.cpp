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

  static uint16_t microseconds_to_ticks(uint16_t microseconds, uint16_t pwm_freq_hz)
  {
    // Convert microseconds to ticks based on frequency
    return static_cast<uint16_t>((static_cast<uint32_t>(microseconds) * pwm_freq_hz * 4096) / 1000000);
  }

  // Receive hardware information during initialization
  hardware_interface::CallbackReturn ThrusterHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
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

  hardware_interface::return_type ThrusterHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < state_.size(); ++i)
    {
      state_[i] =
          command_[i]; // Returns the expected state based on the commands given, as the ESCs do not provide feedback
    }
    return hardware_interface::return_type::OK;
  }

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
} // namespace rov_control

PLUGINLIB_EXPORT_CLASS(rov_control::ThrusterHardwareInterface, hardware_interface::SystemInterface)
