#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rov_control/axis_to_command_controller.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
extern "C"
{
#include "osqp.h"
}

bool solve_thruster_qp(
    const Eigen::MatrixXd &M_plus,
    const Eigen::MatrixXd &M_minus,
    const Eigen::MatrixXd &W,
    const Eigen::MatrixXd &Q,
    const Eigen::VectorXd &desired_wrench,
    Eigen::VectorXd &T_plus,
    Eigen::VectorXd &T_minus,
    double qp_time_limit,
    double *prim_residual,
    double *dual_residual)
{
  int n = Q.rows();

  // Construct A = [M_plus, M_minus]
  Eigen::MatrixXd A_wrench(6, 2 * n);
  A_wrench << M_plus, M_minus;

  // Compute cost: P = A^T W^T W A + Q_total
  Eigen::MatrixXd WtW = W.transpose() * W;
  Eigen::MatrixXd P = A_wrench.transpose() * WtW * A_wrench;

  // Effort minimization term
  Eigen::MatrixXd Q_total = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  Q_total.topLeftCorner(n, n) = Q;
  Q_total.topRightCorner(n, n) = Q;
  Q_total.bottomLeftCorner(n, n) = Q;
  Q_total.bottomRightCorner(n, n) = Q;

  P += Q_total;
  P = 0.5 * (P + P.transpose()); // Ensure symmetry
  P = P.triangularView<Eigen::Upper>();

  // q = -2 * A^T * W^T * W * desired_wrench
  Eigen::VectorXd q = -2.0 * A_wrench.transpose() * WtW * desired_wrench;

  // Bounds for T_plus in [0, 1], T_minus in [-1, 0]
  Eigen::VectorXd lower(2 * n), upper(2 * n);
  for (int i = 0; i < n; ++i)
  {
    lower(i) = 0.0;
    upper(i) = 1.0;
    lower(n + i) = -1.0;
    upper(n + i) = 0.0;
  }

  // Convert to CSC
  Eigen::SparseMatrix<double> P_sparse = P.sparseView();
  P_sparse.makeCompressed();
  Eigen::SparseMatrix<double> A(2 * n, 2 * n);
  A.setIdentity();
  A.makeCompressed();

  // Copy index arrays to OSQPInt (long long) vectors
  std::vector<OSQPInt> P_i(P_sparse.innerIndexPtr(), P_sparse.innerIndexPtr() + P_sparse.nonZeros());
  std::vector<OSQPInt> P_p(P_sparse.outerIndexPtr(), P_sparse.outerIndexPtr() + P_sparse.cols() + 1);
  std::vector<OSQPInt> A_i(A.innerIndexPtr(), A.innerIndexPtr() + A.nonZeros());
  std::vector<OSQPInt> A_p(A.outerIndexPtr(), A.outerIndexPtr() + A.cols() + 1);

  // Create OSQP CSC matrices
  OSQPCscMatrix *P_csc = OSQPCscMatrix_new(
      2 * n, 2 * n, static_cast<OSQPInt>(P_sparse.nonZeros()),
      const_cast<OSQPFloat *>(P_sparse.valuePtr()),
      P_i.data(),
      P_p.data());
  OSQPCscMatrix *A_csc = OSQPCscMatrix_new(
      2 * n, 2 * n, static_cast<OSQPInt>(A.nonZeros()),
      const_cast<OSQPFloat *>(A.valuePtr()),
      A_i.data(),
      A_p.data());

  // Setup OSQP settings
  OSQPSettings *settings = OSQPSettings_new();
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;
  settings->time_limit = qp_time_limit / 1000; // ms to seconds.
  settings->eps_abs = 1e-4;                    // just try as hard as you can until you reach the time limit
  settings->eps_rel = 1e-4;                    // just try as hard as you can until you reach the time limit
  settings->verbose = false;                   // stop yapping

  // Setup solver
  OSQPSolver *solver = nullptr;
  OSQPInt exitflag = osqp_setup(
      &solver,
      P_csc,
      q.data(),
      A_csc,
      lower.data(),
      upper.data(),
      2 * n, // m (constraints)
      2 * n, // n (variables)
      settings);

  bool success = false;
  if (exitflag == 0 && solver)
  {
    osqp_solve(solver);
    // Accept solution if OSQP_SOLVED or OSQP_MAX_TIME and solution exists
    if (solver->solution &&
        (solver->info->status_val == OSQP_SOLVED ||
         solver->info->status_val == OSQP_TIME_LIMIT_REACHED))
    {
      Eigen::Map<Eigen::VectorXd> sol(solver->solution->x, 2 * n);
      T_plus = sol.head(n);
      T_minus = sol.tail(n);
      if (prim_residual)
        *prim_residual = solver->info->prim_res;
      if (dual_residual)
        *dual_residual = solver->info->dual_res;
      success = true;
    }
    osqp_cleanup(solver);
  }

  if (P_csc)
    OSQPCscMatrix_free(P_csc);
  if (A_csc)
    OSQPCscMatrix_free(A_csc);
  if (settings)
    OSQPSettings_free(settings);

  return success;
}

namespace rov_controllers
{

  controller_interface::InterfaceConfiguration AxisToCommandController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
  }

  controller_interface::InterfaceConfiguration AxisToCommandController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  AxisToCommandController::AxisToCommandController() = default;

  controller_interface::CallbackReturn AxisToCommandController::on_init()
  {
    // Declare parameters
    auto_declare<std::vector<double>>("position_offset", {16, 30, 16}); // {x, y, z} (cm)
    auto_declare<std::vector<double>>("rotation_offset", {45, 45});     // {pitch, yaw} (deg)
    auto_declare<double>("forward_max_thrust", 20.0);
    auto_declare<double>("reverse_max_thrust", 15.0);
    auto_declare<std::vector<std::string>>("joints",
                                           {"front_left_bottom_thruster_joint",
                                            "front_right_bottom_thruster_joint",
                                            "front_left_top_thruster_joint",
                                            "front_right_top_thruster_joint",
                                            "back_left_bottom_thruster_joint",
                                            "back_right_bottom_thruster_joint",
                                            "back_left_top_thruster_joint",
                                            "back_right_top_thruster_joint"});
    auto_declare<std::vector<double>>("wrench_weights", {1, 1, 1, 1, 1, 1});
    auto_declare<std::vector<double>>("effort_weights", {1, 1, 1, 1, 1, 1, 1, 1}); // same size as joints
    auto_declare<double>("qp_time_limit", 50);                                     // ms
    auto_declare<double>("prim_res_threshold", 1e-2);
    auto_declare<double>("dual_res_threshold", 1e-2);

    // This may seem redudant but is actually needed so that joint_names_ is populated before on_export_reference_interfaces() is called
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AxisToCommandController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    position_offset_ = get_node()->get_parameter("position_offset").as_double_array();
    rotation_offset_ = get_node()->get_parameter("rotation_offset").as_double_array();
    forward_max_thrust_ = get_node()->get_parameter("forward_max_thrust").as_double();
    reverse_max_thrust_ = get_node()->get_parameter("reverse_max_thrust").as_double();
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    wrench_weights_ = get_node()->get_parameter("wrench_weights").as_double_array();
    effort_weights_ = get_node()->get_parameter("effort_weights").as_double_array();
    qp_time_limit_ = get_node()->get_parameter("qp_time_limit").as_double();
    prim_res_threshold_ = get_node()->get_parameter("prim_res_threshold").as_double();
    dual_res_threshold_ = get_node()->get_parameter("dual_res_threshold").as_double();
    desired_wrench_sub_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
        "/desired_wrench", 10,
        [this](const geometry_msgs::msg::Wrench::SharedPtr msg)
        {
          latest_wrench_ = msg;
        });

    if (effort_weights_.size() != joint_names_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Effort weights must match number of joints.");
      return CallbackReturn::ERROR;
    }

    num_joints_ = joint_names_.size();
    reference_interfaces_.resize(num_joints_);

    // Initialize OSQP
    osqp_set_default_settings(&settings_);
    settings_.alpha = 1.0;
    settings_.time_limit = qp_time_limit_;

    // Setup actual matrices later during update when dimensions are known

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AxisToCommandController::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AxisToCommandController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type AxisToCommandController::update_and_write_commands(
      const rclcpp::Time &time,
      const rclcpp::Duration &period)
  {
    // Example: build W from wrench_weights_
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
    for (size_t i = 0; i < 6; ++i)
    {
      W(i, i) = wrench_weights_.at(i);
    }

    // Example: build Q from effort_weights_
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
    for (size_t i = 0; i < num_joints_; ++i)
    {
      Q(i, i) = effort_weights_.at(i);
    }

    // Build per-thruster positions and orientations to match URDF/Xacro
    std::vector<double> x_offsets = {
        -position_offset_[0], // front_left_bottom_thruster
        position_offset_[0],  // front_right_bottom_thruster
        -position_offset_[0], // front_left_top_thruster
        position_offset_[0],  // front_right_top_thruster
        -position_offset_[0], // back_left_bottom_thruster
        position_offset_[0],  // back_right_bottom_thruster
        -position_offset_[0], // back_left_top_thruster
        position_offset_[0]   // back_right_top_thruster
    };
    std::vector<double> y_offsets = {
        position_offset_[1],  // front_left_bottom_thruster
        position_offset_[1],  // front_right_bottom_thruster
        position_offset_[1],  // front_left_top_thruster
        position_offset_[1],  // front_right_top_thruster
        -position_offset_[1], // back_left_bottom_thruster
        -position_offset_[1], // back_right_bottom_thruster
        -position_offset_[1], // back_left_top_thruster
        -position_offset_[1]  // back_right_top_thruster
    };
    std::vector<double> z_offsets = {
        -position_offset_[2], // front_left_bottom_thruster
        -position_offset_[2], // front_right_bottom_thruster
        position_offset_[2],  // front_left_top_thruster
        position_offset_[2],  // front_right_top_thruster
        -position_offset_[2], // back_left_bottom_thruster
        -position_offset_[2], // back_right_bottom_thruster
        position_offset_[2],  // back_left_top_thruster
        position_offset_[2]   // back_right_top_thruster
    };
    std::vector<double> pitch_offsets = {
        -rotation_offset_[0], // front_left_bottom_thruster
        -rotation_offset_[0], // front_right_bottom_thruster
        rotation_offset_[0],  // front_left_top_thruster
        rotation_offset_[0],  // front_right_top_thruster
        -rotation_offset_[0], // back_left_bottom_thruster
        -rotation_offset_[0], // back_right_bottom_thruster
        rotation_offset_[0],  // back_left_top_thruster
        rotation_offset_[0]   // back_right_top_thruster
    };
    std::vector<double> yaw_offsets = {
        270.0 + rotation_offset_[1], // front_left_bottom_thruster
        0.0 + rotation_offset_[1],   // front_right_bottom_thruster
        270.0 + rotation_offset_[1], // front_left_top_thruster
        0.0 + rotation_offset_[1],   // front_right_top_thruster
        180.0 + rotation_offset_[1], // back_left_bottom_thruster
        90.0 + rotation_offset_[1],  // back_right_bottom_thruster
        180.0 + rotation_offset_[1], // back_left_top_thruster
        90.0 + rotation_offset_[1]   // back_right_top_thruster
    };

    // Convert degrees to radians for pitch and yaw
    for (size_t i = 0; i < num_joints_; ++i)
    {
      pitch_offsets[i] = pitch_offsets[i] * M_PI / 180.0;
      yaw_offsets[i] = yaw_offsets[i] * M_PI / 180.0;
    }

    Eigen::MatrixXd M_plus(6, num_joints_);
    Eigen::MatrixXd M_minus(6, num_joints_);

    for (size_t i = 0; i < num_joints_; ++i)
    {
      // Position
      double x = x_offsets[i];
      double y = y_offsets[i];
      double z = z_offsets[i];

      // Orientation
      double pitch = pitch_offsets[i];
      double yaw = yaw_offsets[i];

      // Thrust direction (unit vector in body frame)
      Eigen::Vector3d thrust_dir;
      thrust_dir << std::cos(pitch) * std::cos(yaw),
          std::cos(pitch) * std::sin(yaw),
          std::sin(pitch);

      // Linear force for forward/reverse
      Eigen::Vector3d f_plus = forward_max_thrust_ * thrust_dir;
      Eigen::Vector3d f_minus = reverse_max_thrust_ * thrust_dir;

      // Torque = F (+ or -) * (r x d)
      // r is the position vector from the thruster to the center of mass
      // Assuming the center of mass is at the origin (0, 0, 0) for simplicity
      // d is the direction of the thruster force.
      // F (+ or -) is a scaler for the maximum thrust in the forward or reverse directions respectivly.
      Eigen::Vector3d r(x, y, z);
      Eigen::Vector3d tau_plus = r.cross(f_plus);
      Eigen::Vector3d tau_minus = r.cross(f_minus);

      // Fill columns
      M_plus.block<3, 1>(0, i) = f_plus;
      M_plus.block<3, 1>(3, i) = tau_plus;
      M_minus.block<3, 1>(0, i) = f_minus;
      M_minus.block<3, 1>(3, i) = tau_minus;
    }
    Eigen::VectorXd desired_wrench(6);
    if (latest_wrench_)
    {
      desired_wrench << latest_wrench_->force.x, 
                        latest_wrench_->force.y, 
                        latest_wrench_->force.z,
                        latest_wrench_->torque.x, 
                        latest_wrench_->torque.y, 
                        latest_wrench_->torque.z;
    }
    else
    {
      desired_wrench.setZero();
    }
    Eigen::VectorXd T_plus, T_minus;

    double prim_res = 0.0, dual_res = 0.0;

    if (solve_thruster_qp(M_plus, M_minus, W, Q, desired_wrench, T_plus, T_minus, qp_time_limit_, &prim_res, &dual_res))
    {
      Eigen::VectorXd T = T_plus + T_minus;

      // // Print T to the console
      // std::ostringstream oss;
      // oss << "Thruster output T: [";
      // for (int i = 0; i < T.size(); ++i) {
      //   oss << T(i);
      //   if (i < T.size() - 1) oss << ", ";
      // }
      // oss << "]";
      // RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());

      double max_abs = T.cwiseAbs().maxCoeff();
      if (max_abs > 1.0)
      {
        T /= max_abs; // Normalize to [-1, 1]
      }
      for (size_t i = 0; i < num_joints_ && i < command_interfaces_.size(); ++i)
      {
        if (!command_interfaces_[i].get().set_value(T(i)))
        {
          RCLCPP_WARN(get_node()->get_logger(), "Failed to set value for command interface %zu", i);
        }
      }
      // Only warn if residuals are above threshold
      if (prim_res > prim_res_threshold_ || dual_res > dual_res_threshold_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "QP solution residuals high: primal=%.3g, dual=%.3g", prim_res, dual_res);
      }
    }
    else
    {
      RCLCPP_WARN(get_node()->get_logger(), "QP solve failed!");
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type rov_controllers::AxisToCommandController::update_reference_from_subscribers(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/)
  {
    // Implement your logic here, or just return OK if not needed
    return controller_interface::return_type::OK;
  }
  std::vector<hardware_interface::CommandInterface>
  rov_controllers::AxisToCommandController::on_export_reference_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> refs;
    const std::string prefix = this->get_node()->get_name();
    for (const auto &joint : joint_names_)
    {
      std::string full_name = prefix + "/" + joint;
      RCLCPP_INFO(get_node()->get_logger(), "Exporting reference interface: %s", full_name.c_str());
      refs.emplace_back(full_name, hardware_interface::HW_IF_EFFORT);
    }
    RCLCPP_INFO(rclcpp::get_logger("AxisToCommandController"), "Exporting %zu reference interfaces", refs.size());
    return refs;
  }

  std::vector<hardware_interface::StateInterface>
  rov_controllers::AxisToCommandController::on_export_state_interfaces()
  {
    return {};
  }
} // namespace rov_controllers

PLUGINLIB_EXPORT_CLASS(rov_controllers::AxisToCommandController, controller_interface::ChainableControllerInterface)