#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <control_toolbox/pid.hpp>

#include "rov_control/thruster_pid_controller.hpp"

namespace rov_controllers
{
  PidController::PidController() = default;

  controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  controller_interface::CallbackReturn PidController::on_init()
  {
    auto_declare<std::vector<std::string>>("dof_names", {"surge", "sway", "heave", "roll", "pitch", "yaw"});
    auto_declare<std::string>("reference_topic", "/desired_velocity");
    auto_declare<std::string>("state_topic", "/current_velocity");
    auto_declare<std::string>("output_topic", "/desired_wrench");

    dof_names_ = get_node()->get_parameter("dof_names").as_string_array();
    for(const auto &dof : dof_names_)
    {
      auto_declare<std::vector<double>>("gains." + dof, {0.0, 0.0, 0.0, 0.0, 0.0});
      if (!get_node()->has_parameter("gains." + dof)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter gains.%s not found!", dof.c_str());
      }
    }

    RCLCPP_INFO(get_node()->get_logger(), "dof_names size: %zu", dof_names_.size());
    for (const auto& name : dof_names_) {
        RCLCPP_INFO(get_node()->get_logger(), "dof_name: %s", name.c_str());
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PidController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    dof_names_ = get_node()->get_parameter("dof_names").as_string_array();
    reference_topic_ = get_node()->get_parameter("reference_topic").as_string();
    state_topic_ = get_node()->get_parameter("state_topic").as_string();
    output_topic_ = get_node()->get_parameter("output_topic").as_string();

    // Load PID gains for each DOF
    pids_.clear();
    for (const auto &dof : dof_names_)
    {
      if (!get_node()->has_parameter("gains." + dof)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter gains.%s not found!", dof.c_str());
      }
      auto dof_params = get_node()->get_parameter("gains." + dof).as_double_array();

      auto pid = std::make_shared<control_toolbox::Pid>();
      pid->initialize(dof_params[0], dof_params[1], dof_params[2], dof_params[3], dof_params[4]);
      pids_.push_back(pid);
    }

    ref_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        reference_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          ref_ = *msg;
        });

    state_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        state_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          state_ = *msg;
        });

    wrench_pub_ = get_node()->create_publisher<geometry_msgs::msg::Wrench>(output_topic_, 10);

    RCLCPP_INFO(get_node()->get_logger(), "dof_names size: %zu", dof_names_.size());
    for (const auto& name : dof_names_) {
        RCLCPP_INFO(get_node()->get_logger(), "dof_name: %s", name.c_str());
    }

    for (const auto &param : get_node()->list_parameters({}, 10).names) {
    RCLCPP_INFO(get_node()->get_logger(), "Loaded param: %s", param.c_str());
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PidController::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PidController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type PidController::update(
      const rclcpp::Time &time,
      const rclcpp::Duration &period)
  {
    geometry_msgs::msg::Twist ref, state;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      ref = ref_;
      state = state_;
    }

    geometry_msgs::msg::Wrench wrench;
    // Linear
    wrench.force.x = pids_[0]->compute_command(ref.linear.x - state.linear.x, period.seconds());
    wrench.force.y = pids_[1]->compute_command(ref.linear.y - state.linear.y, period.seconds());
    wrench.force.z = pids_[2]->compute_command(ref.linear.z - state.linear.z, period.seconds());
    // Angular
    wrench.torque.x = pids_[3]->compute_command(ref.angular.x - state.angular.x, period.seconds());
    wrench.torque.y = pids_[4]->compute_command(ref.angular.y - state.angular.y, period.seconds());
    wrench.torque.z = pids_[5]->compute_command(ref.angular.z - state.angular.z, period.seconds());

    wrench_pub_->publish(wrench);
    return controller_interface::return_type::OK;
  }
} // namespace rov_controllers

PLUGINLIB_EXPORT_CLASS(rov_controllers::PidController, controller_interface::ControllerInterface)