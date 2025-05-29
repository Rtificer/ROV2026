#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
extern "C"
{
#include "osqp.h"
}


namespace rov_controllers {

class AxisToCommandController : public controller_interface::ChainableControllerInterface
{
public:
  AxisToCommandController() = default;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::interface_configuration_type::INDIVIDUAL;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::interface_configuration_type::INDIVIDUAL;
  }

  CallbackReturn on_init() override
  {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("wrench_weights", {1,1,1,1,1,1});
    auto_declare<std::vector<double>>("effort_weights", {});  // same size as joints
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    wrench_weights_ = get_node()->get_parameter("wrench_weights").as_double_array();
    effort_weights_ = get_node()->get_parameter("effort_weights").as_double_array();
    
    if (effort_weights_.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Effort weights must match number of joints.");
      return CallbackReturn::ERROR;
    }

    num_joints_ = joint_names_.size();

    // Initialize OSQP
    osqp_set_default_settings(&settings_);
    settings_.alpha = 1.0;

    // Allocate space for QP problem (placeholder sizes)
    qp_data_ = std::make_unique<OSQPData>();

    // Setup actual matrices later during update when dimensions are known

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // Construct desired wrench vector (placeholder)
    Eigen::VectorXd desired_wrench(6);
    desired_wrench.setZero();

    // Construct optimization problem
    // Let T = T+ - T-, where T+, T- >= 0
    // Total thrust = M+ * T+ + M- * T-

    // residual = W (w - (M+ T+ + M- T-))
    // cost = residual.norm()^2 + T^T Q T

    // Example: Set up OSQP problem
    //   min 0.5 x^T P x + q^T x
    //   s.t. l <= A x <= u
    // x = [T+, T-] 

    // TODO: Fill in matrices using Eigen, convert to CSC, and call osqp_solve

    // Apply results to hardware interfaces

    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> wrench_weights_;
  std::vector<double> effort_weights_;
  size_t num_joints_ = 0;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;

  OSQPSettings settings_;
  std::unique_ptr<OSQPData> qp_data_;
  OSQPWorkspace* qp_workspace_ = nullptr;
};

}  // namespace rov_controllers

PLUGINLIB_EXPORT_CLASS(rov_controllers::AxisToCommandController, controller_interface::ChainableControllerInterface)
