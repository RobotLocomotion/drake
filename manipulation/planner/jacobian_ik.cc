#include "drake/manipulation/planner/jacobian_ik.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

Vector6<double> JacobianIk::CalcPoseDifference(const Isometry3<double>& pose0,
                                               const Isometry3<double>& pose1) {
  Vector6<double> diff = Vector6<double>::Zero();

  // Linear.
  diff.tail<3>() = (pose1.translation() - pose0.translation());

  // Angular.
  AngleAxis<double> rot_err(pose1.linear() * pose0.linear().transpose());
  diff.head<3>() = rot_err.axis() * rot_err.angle();

  return diff;
}

JacobianIk::JacobianIk(const RigidBodyTree<double>* robot)
    : robot_{robot},
      num_joints_{robot_->get_num_positions()},
      identity_{MatrixX<double>::Identity(num_joints_, num_joints_)},
      zero_{VectorX<double>(num_joints_)} {
  if (robot_->get_num_positions() != robot_->get_num_velocities()) {
    throw std::runtime_error(
        "Dimension of genearlized position and velocity has to be the same.");
  }

  q_lower_ = robot_->joint_limit_min;
  q_upper_ = robot_->joint_limit_max;
  v_lower_ = VectorX<double>::Constant(num_joints_, -M_PI);
  v_upper_ = VectorX<double>::Constant(num_joints_, M_PI);
  unconstrained_dof_v_lower_ = VectorX<double>::Constant(1, -0.6);
  unconstrained_dof_v_upper_ = VectorX<double>::Constant(1, 0.6);
}

bool JacobianIk::CalcJointVelocity(const KinematicsCache<double>& cache,
                                   const RigidBodyFrame<double>& frame_E,
                                   const Vector6<double>& V_WE,
                                   const Vector6<double>& gain_E, double dt,
                                   const VectorX<double>& q_nominal,
                                   VectorX<double>* v_out,
                                   bool* is_stuck) const {
  if (q_nominal.size() != num_joints_) {
    throw std::runtime_error("Nominal position has wrong dimensions.");
  }
  if (dt <= 0) {
    throw std::runtime_error("dt must be positive.");
  }
  if ((gain_E.array() < 0).any()) {
    throw std::runtime_error("gain_E must be nonnegative.");
  }

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable v =
      prog.NewContinuousVariables(robot_->get_num_velocities(), "v");
  drake::solvers::VectorXDecisionVariable alpha =
      prog.NewContinuousVariables(1, "alpha");

  Isometry3<double> X_WE = robot_->CalcFramePoseInWorldFrame(cache, frame_E);

  // Rotate the world velocity into E frame.
  drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
  R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
  R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

  MatrixX<double> J_WE_E_6d =
      R_EW *
      robot_->CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);
  Vector6<double> V_WE_E_6d = R_EW * V_WE;

  // Pick the constrained motions.
  int num_cart_constraints = 0;
  for (int i = 0; i < 6; i++) {
    if (gain_E(i) > 0) {
      J_WE_E_6d.row(num_cart_constraints) = gain_E(i) * J_WE_E_6d.row(i);
      V_WE_E_6d(num_cart_constraints) = gain_E(i) * V_WE_E_6d(i);
      num_cart_constraints++;
    }
  }

  const solvers::QuadraticCost* cart_cost = nullptr;

  if (num_cart_constraints > 0) {
    MatrixX<double> J_WE_E = J_WE_E_6d.topRows(num_cart_constraints);
    VectorX<double> V_WE_E = V_WE_E_6d.head(num_cart_constraints);

    VectorX<double> V_WE_E_dir = V_WE_E.normalized();
    double V_WE_E_mag = V_WE_E.norm();

    // Constrain the end effector motion to be in the direction of V_WE_E_dir,
    // and penalize magnitude difference from V_WE_E_mag.
    MatrixX<double> A(num_cart_constraints, J_WE_E.cols() + 1);
    A.topLeftCorner(num_cart_constraints, J_WE_E.cols()) = J_WE_E;
    A.topRightCorner(num_cart_constraints, 1) = -V_WE_E_dir;
    prog.AddLinearEqualityConstraint(
        A, VectorX<double>::Zero(num_cart_constraints), {v, alpha});
    cart_cost =
        prog.AddQuadraticErrorCost(drake::Vector1<double>(100),
                                   drake::Vector1<double>(V_WE_E_mag), alpha)
            .constraint()
            .get();

    Eigen::JacobiSVD<MatrixX<double>> svd(J_WE_E, Eigen::ComputeFullV);

    // Add constrained the unconstrained dof's velocity to be small, which is
    // used
    // to fullfil the regularization cost.
    for (int i = num_cart_constraints; i < num_joints_; i++) {
      prog.AddLinearConstraint(svd.matrixV().col(i).transpose(),
                               unconstrained_dof_v_lower_,
                               unconstrained_dof_v_upper_, v);
    }
  }

  // If redundant, add a small regularization term to q_nominal.
  if (num_cart_constraints < num_joints_) {
    prog.AddQuadraticErrorCost(dt * dt * identity_,
                               (q_nominal - cache.getQ()) / dt, v)
        .constraint()
        .get();
  }

  // Add q upper and lower joint limit.
  prog.AddBoundingBoxConstraint((q_lower_ - cache.getQ()) / dt,
                                (q_upper_ - cache.getQ()) / dt, v);

  // Add v constraint.
  prog.AddBoundingBoxConstraint(v_lower_, v_upper_, v);

  // Solve
  drake::solvers::SolutionResult result = prog.Solve();
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    return false;
  }

  *v_out = prog.GetSolution(v);

  if (num_cart_constraints) {
    VectorX<double> cost(1);
    cart_cost->Eval(prog.GetSolution(alpha), cost);
    // Not tracking the desired vel norm, and computed vel is small.
    *is_stuck = cost(0) > 5 && prog.GetSolution(alpha)[0] <= 1e-2;
  } else {
    *is_stuck = false;
  }

  return true;
}

void JacobianIk::set_unconstrained_dof_velocity_upper_limit(double u) {
  if (u < unconstrained_dof_v_lower_(0)) {
    throw std::runtime_error(
        "Unconstrained dof velocity upper limit is smaller than the lower "
        "limit.");
  }
  unconstrained_dof_v_upper_(0) = u;
}

void JacobianIk::set_unconstrained_dof_velocity_lower_limit(double l) {
  if (l > unconstrained_dof_v_upper_(0)) {
    throw std::runtime_error(
        "Unconstrained dof velocity lower limit is larger than the upper "
        "limit.");
  }
  unconstrained_dof_v_lower_(0) = l;
}

void JacobianIk::set_joint_velocity_upper_limit(const VectorX<double>& u) {
  if (u.size() != num_joints_) {
    throw std::runtime_error(
        "Joint velocity upper limit has incorrect dimension.");
  }
  if ((u.array() > v_lower_.array()).any()) {
    throw std::runtime_error(
        "Joint velocity upper limit is smaller than the lower limit.");
  }
  v_upper_ = u;
}

void JacobianIk::set_joint_velocity_lower_limit(const VectorX<double>& l) {
  if (l.size() != num_joints_) {
    throw std::runtime_error(
        "Joint velocity lower limit has incorrect dimension.");
  }
  if ((l.array() > v_upper_.array()).any()) {
    throw std::runtime_error(
        "Joint velocity lower limit is larger than the upper limit.");
  }
  v_lower_ = l;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
