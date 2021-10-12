#include "drake/manipulation/planner/differential_inverse_kinematics.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace manipulation {
namespace planner {

namespace internal {
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const math::RigidTransform<double>& X_WE,
    const Eigen::Ref<const Matrix6X<double>>& J_WE_W,
    const multibody::SpatialVelocity<double>& V_WE_desired,
    const DifferentialInverseKinematicsParameters& parameters) {
  const math::RotationMatrix<double> R_EW = X_WE.rotation().transpose();
  const multibody::SpatialVelocity<double> V_WE_E = R_EW * V_WE_desired;

  // Rotate the 6 x n Jacobian from the world frame W to the E frame.
  // TODO(Mitiguy) Switch to direct application of RotationMatrix multiplied by
  // a `6 x n` array if that becomes available.
  const int num_columns = J_WE_W.cols();
  Matrix6X<double> J_WE_E{6, num_columns};
  J_WE_E.topRows<3>() = R_EW * J_WE_W.topRows<3>();
  J_WE_E.bottomRows<3>() = R_EW * J_WE_W.bottomRows<3>();

  Vector6<double> V_WE_E_scaled;
  MatrixX<double> J_WE_E_scaled{6, num_columns};
  int num_cart_constraints = 0;
  for (int i = 0; i < 6; i++) {
    const double gain{parameters.get_end_effector_velocity_gain()(i)};
    if (gain > 0) {
      J_WE_E_scaled.row(num_cart_constraints) = gain * J_WE_E.row(i);
      V_WE_E_scaled(num_cart_constraints) = gain * V_WE_E[i];
      num_cart_constraints++;
    }
  }

  return DoDifferentialInverseKinematics(
      q_current, v_current, V_WE_E_scaled.head(num_cart_constraints),
      J_WE_E_scaled.topRows(num_cart_constraints), parameters);
}
}  // namespace internal

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
      const std::shared_ptr<solvers::LinearConstraint>
          constraint) {
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
                                            int num_velocities)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      nominal_joint_position_(VectorX<double>::Zero(num_positions)) {}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const Eigen::Ref<const VectorX<double>>& V,
    const Eigen::Ref<const MatrixX<double>>& J,
    const DifferentialInverseKinematicsParameters& parameters) {
  const int num_positions = parameters.get_num_positions();
  const int num_velocities = parameters.get_num_velocities();
  const int num_cart_constraints = V.size();
  DRAKE_DEMAND(q_current.size() == num_positions);
  DRAKE_DEMAND(v_current.size() == num_velocities);
  DRAKE_DEMAND(J.rows() == num_cart_constraints);
  DRAKE_DEMAND(J.cols() == num_velocities);

  const auto identity_num_positions =
      MatrixX<double>::Identity(num_positions, num_positions);

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(num_velocities, "v_next");
  solvers::VectorDecisionVariable<1> alpha =
      prog.NewContinuousVariables<1>("alpha");

  const solvers::QuadraticCost* cart_cost = nullptr;

  if (num_cart_constraints > 0) {
    VectorX<double> V_dir = V.normalized();
    double V_mag = V.norm();

    // Constrain the end effector motion to be in the direction of V,
    // and penalize magnitude difference from V.
    MatrixX<double> A(num_cart_constraints, num_velocities + 1);
    A.leftCols(num_velocities) = J;
    A.rightCols(1) = -V_dir;
    prog.AddLinearEqualityConstraint(
        A, VectorX<double>::Zero(num_cart_constraints), {v_next, alpha});
    // TODO(russt): This should not be hard-coded.
    const double kCartesianTrackingWeight = 100;
    cart_cost =
        prog.AddQuadraticErrorCost(Vector1<double>(kCartesianTrackingWeight),
                                   Vector1<double>(V_mag), alpha)
            .evaluator()
            .get();

    // Constrain the unconstrained DoFs velocity to be small, which is used
    // to fulfill the regularization cost.  We use the svd of J = UΣV', in
    // which the columns of V corresponding to the small/zero singular values
    // in Σ are the "unconstrained" degrees of freedom.  Since JacobiSVD
    // always sorts the singular values in decreasing order, we expect these
    // to be the last columns.  We assume that J is full row-rank, so has
    // num_cart_constraints non-zero singular values.
    Eigen::JacobiSVD<MatrixX<double>> svd(J, Eigen::ComputeFullV);
    if (parameters.get_unconstrained_degrees_of_freedom_velocity_limit()) {
      const double uncon_v =
          parameters.get_unconstrained_degrees_of_freedom_velocity_limit()
              .value();
      for (int i = num_cart_constraints; i < num_velocities; i++) {
        prog.AddLinearConstraint(svd.matrixV().col(i).transpose(), -uncon_v,
                                 uncon_v, v_next);
      }
    }
  }

  for (const auto& constraint : parameters.get_linear_velocity_constraints()) {
    prog.AddConstraint(
        solvers::Binding<solvers::LinearConstraint>(constraint, v_next));
  }

  // A bunch of the operations below assume num_positions == num_velocities.
  // TODO(russt): Generalize this
  DRAKE_DEMAND(num_positions == num_velocities);

  // If redundant, add a small regularization term to q_nominal.
  const double dt{parameters.get_timestep()};
  if (num_cart_constraints < num_velocities) {
    prog.AddQuadraticErrorCost(
        identity_num_positions * dt * dt,
        (parameters.get_nominal_joint_position() - q_current) / dt, v_next);
  }

  // Add q upper and lower joint limit.
  if (parameters.get_joint_position_limits()) {
    prog.AddBoundingBoxConstraint(
        (parameters.get_joint_position_limits()->first - q_current) / dt,
        (parameters.get_joint_position_limits()->second - q_current) / dt,
        v_next);
  }

  // Add v_next constraint.
  if (parameters.get_joint_velocity_limits()) {
    prog.AddBoundingBoxConstraint(
        parameters.get_joint_velocity_limits()->first,
        parameters.get_joint_velocity_limits()->second, v_next);
  }

  // Add vd constraint.
  if (parameters.get_joint_acceleration_limits()) {
    prog.AddLinearConstraint(
        // TODO(russt): This should be num_velocities if we generalize the
        // implementation.
        identity_num_positions,
        parameters.get_joint_acceleration_limits()->first * dt + v_current,
        parameters.get_joint_acceleration_limits()->second * dt + v_current,
        v_next);
  }

  // Solve
  solvers::OsqpSolver solver;
  solvers::MathematicalProgramResult result = solver.Solve(prog, {}, {});

  if (!result.is_success()) {
    return {std::nullopt,
            DifferentialInverseKinematicsStatus::kNoSolutionFound};
  }

  if (num_cart_constraints) {
    VectorX<double> cost(1);
    cart_cost->Eval(result.GetSolution(alpha), &cost);
    const double kMaxTrackingError = 5;
    const double kMinEndEffectorVel = 1e-2;
    if (cost(0) > kMaxTrackingError &&
        result.GetSolution(alpha)[0] <= kMinEndEffectorVel) {
      // Not tracking the desired vel norm (large tracking error) and the
      // computed vel is small.
      log()->info("v_next = {}", result.GetSolution(v_next).transpose());
      log()->info("alpha = {}", result.GetSolution(alpha).transpose());
      return {std::nullopt, DifferentialInverseKinematicsStatus::kStuck};
    }
  }

  return {result.GetSolution(v_next),
          DifferentialInverseKinematicsStatus::kSolutionFound};
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const Vector6<double>& V_WE_desired,
    const multibody::Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const math::RigidTransform<double> X_WE =
      plant.CalcRelativeTransform(context, plant.world_frame(), frame_E);
  MatrixX<double> J_WE(6, plant.num_velocities());
  const multibody::Frame<double>& frame_W = plant.world_frame();
  plant.CalcJacobianSpatialVelocity(context,
                                    multibody::JacobianWrtVariable::kV,
                                    frame_E, Vector3<double>::Zero(),
                                    frame_W, frame_W, &J_WE);

  return internal::DoDifferentialInverseKinematics(
      plant.GetPositions(context), plant.GetVelocities(context),
      X_WE, J_WE, multibody::SpatialVelocity<double>(V_WE_desired), parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const math::RigidTransform<double>& X_WE_desired,
    const multibody::Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const math::RigidTransform<double> X_WE =
      plant.EvalBodyPoseInWorld(context, frame_E.body()) *
      frame_E.CalcPoseInBodyFrame(context);
  const Vector6<double> V_WE_desired =
      ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) /
      parameters.get_timestep();
  return DoDifferentialInverseKinematics(plant, context, V_WE_desired, frame_E,
                                         parameters);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
