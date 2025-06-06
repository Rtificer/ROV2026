#ifndef ROV_CONTROL__THRUSTER_PID_CONTROLLER_HPP_
#define ROV_CONTROL__THRUSTER_PID_CONTROLLER_HPP_

#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <control_toolbox/pid.hpp>

namespace rov_controllers
{

  class PidController : public controller_interface::ControllerInterface
  {
  public:
    PidController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief Declare controller parameters and perform initial setup.
     *
     * This method is called once when the controller is created. It declares all configurable
     * parameters required for the PID controller. No hardware or ROS interfaces are accessed at this stage.
     *
     * The following parameters are declared:
     * @param dof_names        Vector of strings. Names of the degrees of freedom to control (default: {"surge", "sway", "heave", "roll", "pitch", "yaw"}).
     * @param reference_topic  String. Topic name for the desired velocity input (default: "/desired_velocity").
     * @param state_topic      String. Topic name for the current velocity input (default: "/current_velocity").
     * @param output_topic     String. Topic name for the desired wrench output (default: "/desired_wrench").
     *
     * @return CallbackReturn::SUCCESS if initialization is successful.
     */
    controller_interface::CallbackReturn on_init() override;

    /**
     * @brief Retrieve and configure controller parameters from the ROS parameter server.
     *
     * This method is called when the controller transitions to the "configured" state.
     * It loads all required parameters, initializes PID controllers for each degree of freedom (DOF),
     * and sets up ROS publishers and subscribers.
     *
     * The following parameters are retrieved:
     * @param dof_names        Vector of strings. Names of the degrees of freedom to control.
     * @param reference_topic  String. Topic name for the desired velocity input.
     * @param state_topic      String. Topic name for the current velocity input.
     * @param output_topic     String. Topic name for the desired wrench output.
     *
     * For each DOF in @p dof_names, the following PID gain parameters are also retrieved:
     *   - gains.<dof>.p           (double) Proportional gain
     *   - gains.<dof>.i           (double) Integral gain
     *   - gains.<dof>.d           (double) Derivative gain
     *   - gains.<dof>.i_clamp_max (double) Maximum integral clamp
     *   - gains.<dof>.i_clamp_min (double) Minimum integral clamp
     *
     * @param state The previous lifecycle state (unused).
     * @return CallbackReturn::SUCCESS if configuration is successful, CallbackReturn::ERROR otherwise.
     */
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Compute and publish the desired wrench using PID controllers for each DOF.
     *
     * This method is called periodically to update the controller output. It computes the error
     * between the reference and state for each degree of freedom (DOF), applies the corresponding
     * PID controller, and publishes the resulting wrench.
     *
     * @param time   The current time.
     * @param period The time elapsed since the last update.
     *
     * Internal parameters and topics used:
     * - @p ref_         The latest reference (desired) velocity (geometry_msgs::msg::Twist).
     * - @p state_       The latest measured (current) velocity (geometry_msgs::msg::Twist).
     * - @p pids_        Vector of PID controllers, one for each DOF.
     * - @p wrench_pub_  Publisher for the computed wrench (geometry_msgs::msg::Wrench).
     *
     * @return controller_interface::return_type::OK if successful.
     */
    controller_interface::return_type update(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

  private:
    std::vector<std::string> dof_names_;
    std::vector<std::shared_ptr<control_toolbox::Pid>> pids_;
    std::string reference_topic_, state_topic_, output_topic_;
    geometry_msgs::msg::Twist ref_, state_;
    std::mutex data_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_sub_, state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
  };

} // namespace rov_controllers

#endif // THRUSTER_PID_CONTROLLER_HPP