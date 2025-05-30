#ifndef ROV_CONTROLLERS__AXIS_TO_COMMAND_CONTROLLER_HPP_
#define ROV_CONTROLLERS__AXIS_TO_COMMAND_CONTROLLER_HPP_

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>

extern "C"
{
#include "osqp.h"
}

namespace rov_controllers
{

  class AxisToCommandController : public controller_interface::ChainableControllerInterface
  {
  public:
    AxisToCommandController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

  private:
    std::vector<std::string> joint_names_;
    std::vector<double> wrench_weights_;
    std::vector<double> effort_weights_;
    size_t num_joints_ = 0;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;

    OSQPSettings settings_;
    OSQPWorkspace *qp_workspace_ = nullptr;
  };

} // namespace rov_controllers

#endif // ROV_CONTROLLERS__AXIS_TO_COMMAND_CONTROLLER_HPP_
