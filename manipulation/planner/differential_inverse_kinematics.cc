#include "drake/manipulation/planner/differential_inverse_kinematics.h"

#include <memory>
#include <string>

namespace drake {
namespace manipulation {
namespace planner {

std::ostream& operator<<(std::ostream& os,
                         const DifferentialInverseKinematicsStatus value) {
  switch (value) {
    case (DifferentialInverseKinematicsStatus::kSolutionFound):
      return os << "Solution found.";
    case (DifferentialInverseKinematicsStatus::kNoSolutionFound):
      return os << "No solution found.";
    case (DifferentialInverseKinematicsStatus::kStuck):
      return os << "Stuck!";
    default:
      DRAKE_ABORT();
  }
}

Vector6<double> ComputePoseDiffInCommonFrame(const Isometry3<double>& pose0,
                                             const Isometry3<double>& pose1) {
  Vector6<double> diff = Vector6<double>::Zero();

  // Linear.
  diff.tail<3>() = (pose1.translation() - pose0.translation());

  // Angular.
  AngleAxis<double> rot_err(pose1.linear() * pose0.linear().transpose());
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
    const VectorX<double> q_current, const VectorX<double>& v_current,
    const VectorX<double>& V, const MatrixX<double>& J,
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

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(num_velocities, "v_next");
  drake::solvers::VectorXDecisionVariable alpha =
      prog.NewContinuousVariables(1, "alpha");

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
    const double kCartesianTrackingWeight = 100;
    cart_cost =
        prog.AddQuadraticErrorCost(Vector1<double>(kCartesianTrackingWeight),
                                   Vector1<double>(V_mag), alpha)
            .constraint()
            .get();

    Eigen::JacobiSVD<MatrixX<double>> svd(J, Eigen::ComputeFullV);

    // Add constrained the unconstrained dof's velocity to be small, which is
    // used to fulfil the regularization cost.
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
        identity_num_positions,
        parameters.get_joint_acceleration_limits()->first * dt + v_current,
        parameters.get_joint_acceleration_limits()->second * dt + v_current,
        v_next);
  }

  // Solve
  drake::solvers::SolutionResult result = prog.Solve();

  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    return {nullopt, DifferentialInverseKinematicsStatus::kNoSolutionFound};
  }

  if (num_cart_constraints) {
    Eigen::VectorXd cost(1);
    cart_cost->Eval(prog.GetSolution(alpha), cost);
    const double kMaxTrackingError = 5;
    const double kMinEndEffectorVel = 1e-2;
    if (cost(0) > kMaxTrackingError &&
        prog.GetSolution(alpha)[0] <= kMinEndEffectorVel) {
      // Not tracking the desired vel norm (large tracking error) and the
      // computed vel is small.
      drake::log()->info("v_next = {}", prog.GetSolution(v_next).transpose());
      drake::log()->info("alpha = {}", prog.GetSolution(alpha).transpose());
      return {nullopt, DifferentialInverseKinematicsStatus::kStuck};
    }
  }

  return {prog.GetSolution(v_next),
          DifferentialInverseKinematicsStatus::kSolutionFound};
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Isometry3<double>& X_WE_desired,
    const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const Isometry3<double> X_WE =
      robot.CalcFramePoseInWorldFrame(cache, frame_E);
  const Vector6<double> V_WE_desired =
      ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) /
      parameters.get_timestep();
  return DoDifferentialInverseKinematics(robot, cache, V_WE_desired, frame_E,
                                         parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Vector6<double>& V_WE_desired, const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  Eigen::Isometry3d X_WE = robot.CalcFramePoseInWorldFrame(cache, frame_E);

  drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
  R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
  R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

  // Rotate the velocity into E frame.
  Eigen::MatrixXd J_WE_E =
      R_EW * robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);

  Vector6<double> V_WE_E = R_EW * V_WE_desired;

  Vector6<double> V_WE_E_scaled;
  MatrixX<double> J_WE_E_scaled{6, J_WE_E.cols()};
  int num_cart_constraints = 0;
  for (int i = 0; i < 6; i++) {
    const double gain{parameters.get_end_effector_velocity_gain()(i)};
    if (gain > 0) {
      J_WE_E_scaled.row(num_cart_constraints) = gain * J_WE_E.row(i);
      V_WE_E_scaled(num_cart_constraints) = gain * V_WE_E(i);
      num_cart_constraints++;
    }
  }

  MatrixX<double> J = J_WE_E_scaled.topRows(num_cart_constraints);
  VectorX<double> V = V_WE_E_scaled.head(num_cart_constraints);

  return DoDifferentialInverseKinematics(cache.getQ(), cache.getV(), V, J,
                                         parameters);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
