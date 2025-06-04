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
      std::string prefix = "gains." + dof;
      double p = get_node()->get_parameter(prefix + ".p").as_double();
      double i = get_node()->get_parameter(prefix + ".i").as_double();
      double d = get_node()->get_parameter(prefix + ".d").as_double();
      double i_max = get_node()->get_parameter(prefix + ".i_clamp_max").as_double();
      double i_min = get_node()->get_parameter(prefix + ".i_clamp_min").as_double();

      auto pid = std::make_shared<control_toolbox::Pid>();
      pid->initPid(p, i, d, i_max, i_min);
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
    wrench.force.x = pids_[0]->computeCommand(ref.linear.x - state.linear.x, period.seconds());
    wrench.force.y = pids_[1]->computeCommand(ref.linear.y - state.linear.y, period.seconds());
    wrench.force.z = pids_[2]->computeCommand(ref.linear.z - state.linear.z, period.seconds());
    // Angular
    wrench.torque.x = pids_[3]->computeCommand(ref.angular.x - state.angular.x, period.seconds());
    wrench.torque.y = pids_[4]->computeCommand(ref.angular.y - state.angular.y, period.seconds());
    wrench.torque.z = pids_[5]->computeCommand(ref.angular.z - state.angular.z, period.seconds());

    wrench_pub_->publish(wrench);
    return controller_interface::return_type::OK;
  }
} // namespace rov_controllers

PLUGINLIB_EXPORT_CLASS(rov_controllers::PidController, controller_interface::ControllerInterface)