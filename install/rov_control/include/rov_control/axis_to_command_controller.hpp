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

    /**
     * @brief Declare controller parameters and perform initial setup.
     *
     * This method is called once when the controller is created. It declares all configurable
     * parameters required for the controller. No hardware or ROS interfaces are accessed at this stage.
     *
     * The following parameters are declared:
     * @param position_offset      Vector of 3 doubles {x, y, z} (cm). Offset of thrusters from the center of mass.
     * @param rotation_offset      Vector of 2 doubles {pitch, yaw} (deg). Orientation offset for each thruster.
     * @param forward_max_thrust   Maximum forward thrust per thruster (double, Newtons).
     * @param reverse_max_thrust   Maximum reverse thrust per thruster (double, Newtons).
     * @param joints               Vector of joint names (strings) for each thruster.
     * @param wrench_weights       Vector of 6 doubles. Weighting factors for each wrench axis (force/torque).
     * @param effort_weights       Vector of doubles, same size as joints. Weighting factors for each thruster's effort.
     * @param qp_time_limit        Maximum time allowed for QP solve (double, milliseconds).
     * @param prim_res_threshold   Threshold for QP solver's primal residual (double).
     * @param dual_res_threshold   Threshold for QP solver's dual residual (double).
     *
     * @return CallbackReturn::SUCCESS if initialization is successful.
     */
    CallbackReturn on_init() override;

    /**
     * @brief Retrieve and validate controller parameters from the ROS parameter server.
     *
     * This method is called when the controller transitions to the "configured" state.
     * It loads all required parameters, checks their validity, and initializes internal variables.
     * If any parameter is invalid, the method returns an error.
     *
     * The following parameters are retrieved:
     * @param position_offset      Vector of 3 doubles {x, y, z} (cm). Offset of thrusters from the center of mass.
     * @param rotation_offset      Vector of 2 doubles {pitch, yaw} (deg). Orientation offset for each thruster.
     * @param forward_max_thrust   Maximum forward thrust per thruster (double, Newtons).
     * @param reverse_max_thrust   Maximum reverse thrust per thruster (double, Newtons).
     * @param joints               Vector of joint names (strings) for each thruster.
     * @param wrench_weights       Vector of 6 doubles. Weighting factors for each wrench axis (force/torque).
     * @param effort_weights       Vector of doubles, same size as joints. Weighting factors for each thruster's effort.
     * @param qp_time_limit        Maximum time allowed for QP solve (double, milliseconds).
     * @param prim_res_threshold   Threshold for QP solver's primal residual (double).
     * @param dual_res_threshold   Threshold for QP solver's dual residual (double).
     *
     * @param previous_state       The previous lifecycle state (unused).
     * @return CallbackReturn::SUCCESS if configuration is successful, CallbackReturn::ERROR otherwise.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Computes and writes thruster commands to achieve a desired wrench using quadratic programming.
     *
     * This function calculates the optimal commands for each thruster on the ROV to achieve a desired "wrench" (a combination of forces and torques)
     * on the robot's body. It does this by formulating and solving a quadratic programming (QP) problem, which is a type of optimization problem
     * that finds the best solution (here, thruster outputs) while minimizing a cost (such as energy use) and respecting constraints (such as thruster limits).
     *
     * **Key Steps:**
     * 1. **Weight Matrices:**
     *    - Builds a matrix `W` to weight the importance of each component of the wrench (force/torque). Used describe to the solver the drag and other forces leading to movement in certain directions being less efficent than in others.
     *    - Builds a matrix `Q` to weight the effort used by each thruster. This is used to compensate for thrusters underperforming or allow the robot to function in the case of loss of functionality of a thruster (usually something is stuck, etc.)
     *
     * 2. **Thruster Geometry:**
     *    - Calculates the position and orientation of each thruster relative to the robot's center based on information in control_parameters.yaml
     *    - For each thruster, computes its direction (using pitch and yaw angles) and how much force it can apply in both forward and reverse directions.
     *    - Uses the cross product to determine the torque each thruster can generate about the robot's center.
     *
     * 3. **Mapping Forces and Torques:**
     *    - Constructs matrices (`M_plus` and `M_minus`) that describe how each thruster's force contributes to the overall wrench.
     *    - `M_plus` maps forward thrust (an individual thruster forward thrust not the ROV moving forward) to its effect on the vehicles wrench.
     *   - `M_minus` does the same for reverse thrust, which is usually lower (like is the case with the T200 we're using).
     *
     * 4. **Quadratic Programming (QP) Solver:**
     *    - A QP solver is a mathematical tool that finds the best set of thruster commands to achieve the desired wrench, while minimizing the cost (weighted by `Q`) and staying within thruster limits.
     *    - The solver considers both forward and reverse thrust capabilities.
     *    - If a solution is found, the resulting thruster commands are written to the hardware interfaces.
     *    - If not, a warning is logged.
     *
     * **Linear Algebra Concepts Used:**
     * - **Cross Product:** Used to compute the torque generated by a thruster force applied at a position offset from the center.
     * - **Dot Product and Matrix Multiplication:** Used in the optimization cost and constraints.
     *
     * @param time   The current time (not directly used in this function).
     * @param period The time since the last update (not directly used in this function).
     * @return controller_interface::return_type OK if successful, otherwise logs a warning.
     *
     * @note The function assumes the desired wrench is set elsewhere and that the robot's center of mass is at the origin.
     *       The QP solver ensures that the solution respects physical limits and tries to use the least amount of effort.
     */
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

  private:
    std::vector<double> position_offset_; // [x, y, z]
    std::vector<double> rotation_offset_; // [pitch, yaw]
    double forward_max_thrust_;
    double reverse_max_thrust_;
    std::vector<std::string> joint_names_;
    std::vector<double> wrench_weights_;
    std::vector<double> effort_weights_;
    double qp_time_limit_;
    double prim_res_threshold_;
    double dual_res_threshold_;
    size_t num_joints_ = 8;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;

    OSQPSettings settings_;
    OSQPWorkspace *qp_workspace_ = nullptr;
  };

} // namespace rov_controllers

#endif // ROV_CONTROLLERS__AXIS_TO_COMMAND_CONTROLLER_HPP_
