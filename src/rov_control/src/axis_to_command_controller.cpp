#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
extern "C" {
  #include "osqp.h"
}

bool solve_thruster_qp(
    const Eigen::MatrixXd& M_plus,
    const Eigen::MatrixXd& M_minus,
    const Eigen::MatrixXd& W,
    const Eigen::MatrixXd& Q,
    const Eigen::VectorXd& desired_wrench,
    Eigen::VectorXd& T_plus,
    Eigen::VectorXd& T_minus
) {
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
    P = 0.5 * (P + P.transpose());  // Ensure symmetry

    // q = -2 * A^T * W^T * W * desired_wrench
    Eigen::VectorXd q = -2.0 * A_wrench.transpose() * WtW * desired_wrench;

    // Bounds for T_plus in [0, 1], T_minus in [-1, 0]
    Eigen::VectorXd lower(2 * n), upper(2 * n);
    for (int i = 0; i < n; ++i) {
        lower(i) = 0.0;
        upper(i) = 1.0;
        lower(n + i) = -1.0;
        upper(n + i) = 0.0;
    }

    // Convert to CSC
    Eigen::SparseMatrix<double> P_sparse = P.sparseView();
    P_sparse.makeCompressed();
    Eigen::SparseMatrix<double> A(2*n, 2*n);
    A.setIdentity();
    A.makeCompressed();

    // Copy index arrays to OSQPInt (long long) vectors
    std::vector<OSQPInt> P_i(P_sparse.innerIndexPtr(), P_sparse.innerIndexPtr() + P_sparse.nonZeros());
    std::vector<OSQPInt> P_p(P_sparse.outerIndexPtr(), P_sparse.outerIndexPtr() + P_sparse.cols() + 1);
    std::vector<OSQPInt> A_i(A.innerIndexPtr(), A.innerIndexPtr() + A.nonZeros());
    std::vector<OSQPInt> A_p(A.outerIndexPtr(), A.outerIndexPtr() + A.cols() + 1);

    // Create OSQP CSC matrices
    OSQPCscMatrix* P_csc = OSQPCscMatrix_new(
        2*n, 2*n, static_cast<OSQPInt>(P_sparse.nonZeros()),
        const_cast<OSQPFloat*>(P_sparse.valuePtr()),
        P_i.data(),
        P_p.data()
    );
    OSQPCscMatrix* A_csc = OSQPCscMatrix_new(
        2*n, 2*n, static_cast<OSQPInt>(A.nonZeros()),
        const_cast<OSQPFloat*>(A.valuePtr()),
        A_i.data(),
        A_p.data()
    );

    // Setup OSQP settings
    OSQPSettings* settings = OSQPSettings_new();
    osqp_set_default_settings(settings);
    settings->alpha = 1.0;
    settings->time_limit = 0.05; //seconds

    // Setup solver
    OSQPSolver* solver = nullptr;
    OSQPInt exitflag = osqp_setup(
        &solver,
        P_csc,
        q.data(),
        A_csc,
        lower.data(),
        upper.data(),
        2*n, // m (constraints)
        2*n, // n (variables)
        settings
    );

    bool success = false;
    if (exitflag == 0 && solver) {
        osqp_solve(solver);
        if (solver->solution && solver->info->status_val == OSQP_SOLVED) {
            Eigen::Map<Eigen::VectorXd> sol(solver->solution->x, 2*n);
            T_plus = sol.head(n);
            T_minus = sol.tail(n);
            success = true;
        }
        osqp_cleanup(solver);
    }

    if (P_csc) OSQPCscMatrix_free(P_csc);
    if (A_csc) OSQPCscMatrix_free(A_csc);
    if (settings) OSQPSettings_free(settings);

    return success;
}

namespace rov_controllers {

class AxisToCommandController : public controller_interface::ChainableControllerInterface
{
public:
  AxisToCommandController() = default;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  }

  CallbackReturn on_init() override
  {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("wrench_weights", {1,1,1,1,1,1});
    auto_declare<std::vector<double>>("effort_weights", {});  // same size as joints
    auto_declare<double>("qp_time_limit", 0.05);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    wrench_weights_ = get_node()->get_parameter("wrench_weights").as_double_array();
    effort_weights_ = get_node()->get_parameter("effort_weights").as_double_array();
    double qp_time_limit = get_node()->get_parameter("qp_time_limit").as_double();
    
    if (effort_weights_.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Effort weights must match number of joints.");
      return CallbackReturn::ERROR;
    }

    num_joints_ = joint_names_.size();

    // Initialize OSQP
    osqp_set_default_settings(&settings_);
    settings_.alpha = 1.0;
    settings_.time_limit = qp_time_limit;

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

  void update()
  {
    // Example: build W from wrench_weights_
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
    for (size_t i = 0; i < 6; ++i) {
        W(i, i) = wrench_weights_.at(i);
    }

    // Example: build Q from effort_weights_
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
        Q(i, i) = effort_weights_.at(i);
    }

    // TODO: Fill these with your actual data
    Eigen::MatrixXd M_plus(6, num_joints_);   // Fill with your mixer matrix
    Eigen::MatrixXd M_minus(6, num_joints_);  // Fill with your mixer matrix
    Eigen::VectorXd desired_wrench(6);        // Fill with your desired wrench

    // Example: zero initialize for demonstration
    M_plus.setZero();
    M_minus.setZero();
    desired_wrench.setZero();

    Eigen::VectorXd T_plus, T_minus;

    if (solve_thruster_qp(M_plus, M_minus, W, Q, desired_wrench, T_plus, T_minus)) {
        // Write T_plus + T_minus to your command interfaces
        Eigen::VectorXd T = T_plus + T_minus;
        for (size_t i = 0; i < num_joints_ && i < command_interfaces_.size(); ++i) {
            command_interfaces_[i].get().set_value(T(i));
        }
    } else {
        RCLCPP_WARN(get_node()->get_logger(), "QP solve failed!");
    }
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> wrench_weights_;
  std::vector<double> effort_weights_;
  size_t num_joints_ = 0;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;

  OSQPSettings settings_;
  OSQPWorkspace* qp_workspace_ = nullptr;
};

}  // namespace rov_controllers

PLUGINLIB_EXPORT_CLASS(rov_controllers::AxisToCommandController, controller_interface::ChainableControllerInterface)