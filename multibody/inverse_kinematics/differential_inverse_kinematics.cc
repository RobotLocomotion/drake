#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "drake/solvers/clp_solver.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace multibody {

using Eigen::SparseMatrix;

namespace {
DifferentialInverseKinematicsResult DoDifferentialInverseKinematicsImpl(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const math::RigidTransform<double>& X_WE,
    const Eigen::Ref<const Matrix6X<double>>& J_WE_W,
    const SpatialVelocity<double>& V_WE_desired,
    const DifferentialInverseKinematicsParameters& parameters,
    const std::optional<Eigen::Ref<const SparseMatrix<double>>>& N,
    const std::optional<Eigen::Ref<const SparseMatrix<double>>>& Nplus) {
  const math::RotationMatrix<double> R_EW = X_WE.rotation().transpose();
  const SpatialVelocity<double> V_WE_E = R_EW * V_WE_desired;

  // Rotate the 6 x n Jacobian from the world frame W to the E frame.
  // TODO(Mitiguy) Switch to direct application of RotationMatrix multiplied by
  // a `6 x n` array if that becomes available.
  const int num_columns = J_WE_W.cols();
  Matrix6X<double> J_WE_E{6, num_columns};
  J_WE_E.topRows<3>() = R_EW * J_WE_W.topRows<3>();
  J_WE_E.bottomRows<3>() = R_EW * J_WE_W.bottomRows<3>();

  Vector6<double> V_WE_E_with_flags;
  MatrixX<double> J_WE_E_with_flags{6, num_columns};
  int num_cart_constraints = 0;
  for (int i = 0; i < 6; i++) {
    if (parameters.get_end_effector_velocity_flag()(i)) {
      J_WE_E_with_flags.row(num_cart_constraints) = J_WE_E.row(i);
      V_WE_E_with_flags(num_cart_constraints) = V_WE_E[i];
      num_cart_constraints++;
    }
  }

  return DoDifferentialInverseKinematics(
      q_current, v_current, V_WE_E_with_flags.head(num_cart_constraints),
      J_WE_E_with_flags.topRows(num_cart_constraints), parameters, N, Nplus);
}

}  // namespace

std::ostream& operator<<(std::ostream& os,
                         const DifferentialInverseKinematicsStatus value) {
  switch (value) {
    case (DifferentialInverseKinematicsStatus::kSolutionFound):
      return os << "Solution found.";
    case (DifferentialInverseKinematicsStatus::kNoSolutionFound):
      return os << "No solution found.";
    case (DifferentialInverseKinematicsStatus::kStuck):
      return os << "Stuck!";
  }
  DRAKE_UNREACHABLE();
}

const std::vector<std::shared_ptr<solvers::LinearConstraint>>&
DifferentialInverseKinematicsParameters::get_linear_velocity_constraints()
    const {
  return linear_velocity_constraints_;
}

void DifferentialInverseKinematicsParameters::AddLinearVelocityConstraint(
    const std::shared_ptr<solvers::LinearConstraint> constraint) {
  if (constraint->num_vars() != get_num_velocities()) {
    throw std::invalid_argument(fmt::format(
        "Number of variables, {}, does not match number of velocities, {}.",
        constraint->num_vars(), get_num_velocities()));
  }
  linear_velocity_constraints_.push_back(constraint);
}

void DifferentialInverseKinematicsParameters::ClearLinearVelocityConstraints() {
  linear_velocity_constraints_.clear();
}

Vector6<double> ComputePoseDiffInCommonFrame(
    const math::RigidTransform<double>& X_C0,
    const math::RigidTransform<double>& X_C1) {
  Vector6<double> diff = Vector6<double>::Zero();

  // Linear.
  diff.tail<3>() = (X_C1.translation() - X_C0.translation());

  // Angular.
  AngleAxis<double> rot_err =
      (X_C1.rotation() * X_C0.rotation().transpose()).ToAngleAxis();
  diff.head<3>() = rot_err.axis() * rot_err.angle();

  return diff;
}

DifferentialInverseKinematicsParameters::
    DifferentialInverseKinematicsParameters(int num_positions,
                                            std::optional<int> num_velocities)
    : num_positions_(num_positions),
      num_velocities_(num_velocities.value_or(num_positions)),
      nominal_joint_position_(VectorX<double>::Zero(num_positions)),
      joint_centering_gain_(
          MatrixX<double>::Zero(num_positions_, num_positions)) {
  DRAKE_DEMAND(num_positions > 0);
  DRAKE_DEMAND(num_velocities > 0);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const Eigen::Ref<const VectorX<double>>& V,
    const Eigen::Ref<const MatrixX<double>>& J,
    const DifferentialInverseKinematicsParameters& parameters,
    const std::optional<Eigen::Ref<const SparseMatrix<double>>>& N,
    const std::optional<Eigen::Ref<const SparseMatrix<double>>>& Nplus) {
  const int num_positions = parameters.get_num_positions();
  const int num_velocities = parameters.get_num_velocities();
  const double dt = parameters.get_time_step();
  const int num_cart_constraints = V.size();
  DRAKE_DEMAND(q_current.size() == num_positions);
  DRAKE_DEMAND(v_current.size() == num_velocities);
  DRAKE_DEMAND(J.rows() == num_cart_constraints);
  DRAKE_DEMAND(J.cols() == num_velocities);
  const auto identity_num_positions =
      MatrixX<double>::Identity(num_positions, num_positions);

  solvers::MathematicalProgram prog;
  bool quadratic_cost = false;
  solvers::VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(num_velocities, "v_next");
  solvers::VectorDecisionVariable<1> alpha =
      prog.NewContinuousVariables<1>("alpha");

  // 100*alpha
  const double kPrimaryObjectiveGain = 100;
  prog.AddLinearCost(-Vector1d{kPrimaryObjectiveGain}, 0.0, alpha);

  // |P⋅(v_next - N⁺(q)⋅K⋅(q_nominal - q_current))|²
  const Eigen::FullPivLU<MatrixX<double>> lu(J);
  if (lu.rank() < num_velocities) {
    const Eigen::MatrixXd P = lu.kernel().transpose();
    if (Nplus) {
      DRAKE_DEMAND(Nplus->rows() == num_velocities);
      DRAKE_DEMAND(Nplus->cols() == num_positions);
      prog.Add2NormSquaredCost(
          P,
          P * (*Nplus) * parameters.get_joint_centering_gain() *
              (parameters.get_nominal_joint_position() - q_current),
          v_next);
    } else if (num_positions != num_velocities) {
      throw std::runtime_error(
          "You must pass the Nplus matrix to DoDifferentialInverseKinematics "
          "when J is not full column rank and num_positions != "
          "num_velocities.");
    } else {
      prog.Add2NormSquaredCost(
          P,
          P * parameters.get_joint_centering_gain() *
              (parameters.get_nominal_joint_position() - q_current),
          v_next);
    }
    quadratic_cost = true;
  }

  // J * v_next = alpha * V
  if (V.size() > 0) {
    MatrixX<double> A(num_cart_constraints, num_velocities + 1);
    A.leftCols(num_velocities) = J;
    A.rightCols(1) = -V;
    prog.AddLinearEqualityConstraint(
        A, VectorX<double>::Zero(num_cart_constraints), {v_next, alpha});
  }

  // 0 <= alpha <= 1
  prog.AddBoundingBoxConstraint(0, 1, alpha);

  // joint_lim_min <= q_current + N⋅v_next⋅dt <= joint_lim_max
  if (parameters.get_joint_position_limits()) {
    const Eigen::VectorXd lb =
        (parameters.get_joint_position_limits()->first - q_current) / dt;
    const Eigen::VectorXd ub =
        (parameters.get_joint_position_limits()->second - q_current) / dt;
    if (N) {
      DRAKE_DEMAND(N->rows() == num_positions);
      DRAKE_DEMAND(N->cols() == num_velocities);
      // We cannot currently use prog.AddLinearConstraint with a SparseMatrix.
      prog.AddConstraint(
          std::make_shared<solvers::LinearConstraint>(*N, lb, ub), v_next);
    } else if (num_positions != num_velocities) {
      throw std::runtime_error(
          "You must pass the N matrix to DoDifferentialInverseKinematics "
          "when you have joint position limits and num_positions != "
          "num_velocities.");
    } else {
      prog.AddBoundingBoxConstraint(lb, ub, v_next);
    }
  }

  // joint_vel_lim_min <= v_next <= joint_vel_lim_max
  if (parameters.get_joint_velocity_limits()) {
    prog.AddBoundingBoxConstraint(
        parameters.get_joint_velocity_limits()->first,
        parameters.get_joint_velocity_limits()->second, v_next);
  }

  // joint_accel_lim_min <= (v_next - v_current)/dt <= joint_accel_lim_max
  if (parameters.get_joint_acceleration_limits()) {
    prog.AddLinearConstraint(
        identity_num_positions,
        parameters.get_joint_acceleration_limits()->first * dt + v_current,
        parameters.get_joint_acceleration_limits()->second * dt + v_current,
        v_next);
  }

  // additional linear velocity constraints
  for (const auto& constraint : parameters.get_linear_velocity_constraints()) {
    prog.AddConstraint(
        solvers::Binding<solvers::LinearConstraint>(constraint, v_next));
  }

  // Solve
  solvers::MathematicalProgramResult result;

  if (quadratic_cost) {
    solvers::OsqpSolver solver;
    result = solver.Solve(prog, {}, parameters.get_solver_options());
  } else {
    solvers::ClpSolver solver;
    result = solver.Solve(prog, {}, parameters.get_solver_options());
  }

  if (!result.is_success()) {
    return {std::nullopt,
            DifferentialInverseKinematicsStatus::kNoSolutionFound};
  }

  const double alpha_sol = result.GetSolution(alpha[0]);
  if (alpha_sol < parameters.get_maximum_scaling_to_report_stuck()) {
    // The computed velocity is small compared to the desired.
    return {result.GetSolution(v_next),
            DifferentialInverseKinematicsStatus::kStuck};
  }

  return {result.GetSolution(v_next),
          DifferentialInverseKinematicsStatus::kSolutionFound};
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const Vector6<double>& V_WE_desired, const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const Frame<double>& frame_W = plant.world_frame();
  return DoDifferentialInverseKinematics(plant, context, V_WE_desired, frame_W,
                                         frame_E, parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const Vector6<double>& V_AE_desired, const Frame<double>& frame_A,
    const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const math::RigidTransform<double> X_AE =
      plant.CalcRelativeTransform(context, frame_A, frame_E);
  MatrixX<double> J_AE(6, plant.num_velocities());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kV, frame_E,
                                    Vector3<double>::Zero(), frame_A, frame_A,
                                    &J_AE);

  std::optional<Eigen::SparseMatrix<double>> N = std::nullopt;
  std::optional<Eigen::SparseMatrix<double>> Nplus = std::nullopt;
  if (!plant.IsVelocityEqualToQDot()) {
    if (parameters.get_joint_position_limits()) {
      N = plant.MakeVelocityToQDotMap(context);
    }
    Nplus = plant.MakeQDotToVelocityMap(context);
  }
  return DoDifferentialInverseKinematicsImpl(
      plant.GetPositions(context), plant.GetVelocities(context), X_AE, J_AE,
      SpatialVelocity<double>(V_AE_desired), parameters, N, Nplus);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const math::RigidTransform<double>& X_WE_desired,
    const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const Frame<double>& frame_W = plant.world_frame();
  return DoDifferentialInverseKinematics(plant, context, X_WE_desired, frame_W,
                                         frame_E, parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const math::RigidTransform<double>& X_AE_desired,
    const Frame<double>& frame_A, const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const math::RigidTransform<double> X_AE =
      plant.CalcRelativeTransform(context, frame_A, frame_E);
  Vector6<double> V_AE_desired =
      ComputePoseDiffInCommonFrame(X_AE, X_AE_desired) /
      parameters.get_time_step();
  // Saturate the velocity command at the limits:
  if (V_AE_desired.head<3>().norm() >
      parameters.get_end_effector_angular_speed_limit()) {
    V_AE_desired.head<3>().normalize();
    V_AE_desired.head<3>() *= parameters.get_end_effector_angular_speed_limit();
  }
  if (parameters.get_end_effector_translational_velocity_limits()) {
    V_AE_desired.tail<3>() =
        V_AE_desired.tail<3>()
            .cwiseMax(
                parameters.get_end_effector_translational_velocity_limits()
                    ->first)
            .cwiseMin(
                parameters.get_end_effector_translational_velocity_limits()
                    ->second);
  }
  return DoDifferentialInverseKinematics(plant, context, V_AE_desired, frame_A,
                                         frame_E, parameters);
}

}  // namespace multibody
}  // namespace drake
