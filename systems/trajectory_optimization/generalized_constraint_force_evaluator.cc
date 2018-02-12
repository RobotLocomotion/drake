#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
using plants::KinematicsCacheWithVHelper;

// This evaluator computes the generalized constraint force Jᵀλ.
GeneralizedConstraintForceEvaluator::GeneralizedConstraintForceEvaluator(
    const RigidBodyTree<double>& tree, int num_lambda)
    : EvaluatorBase(
          tree.get_num_velocities(),
          tree.get_num_positions() + tree.get_num_velocities() + num_lambda,
          "generalized constraint force"),
      tree_{&tree},
      num_lambda_(num_lambda) {}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  // x contains q, v and λ
  DRAKE_ASSERT(x.rows() == num_vars());
  const auto q = x.head(tree_->get_num_positions());
  const auto v =
      x.segment(tree_->get_num_positions(), tree_->get_num_velocities());
  const auto lambda = x.tail(num_lambda_);

  const auto J = EvalConstraintJacobian(q, v);
  y = J.transpose() * lambda;
}

MatrixX<AutoDiffXd> PositionConstraintForceEvaluator::EvalConstraintJacobian(
    const Eigen::Ref<const AutoDiffVecXd>& q,
    const Eigen::Ref<const AutoDiffVecXd>& v) const {
  auto kinsol = kinematics_cache_helper_->UpdateKinematics(q, v);
  return tree()->positionConstraintsJacobian(kinsol);
}

JointLimitConstraintForceEvaluator::JointLimitConstraintForceEvaluator(
    const RigidBodyTree<double>& tree, int joint_velocity_index)
    : GeneralizedConstraintForceEvaluator(tree, 2),
      joint_velocity_index_(joint_velocity_index) {}

MatrixX<AutoDiffXd> JointLimitConstraintForceEvaluator::EvalConstraintJacobian(
    const Eigen::Ref<const AutoDiffVecXd>& q,
    const Eigen::Ref<const AutoDiffVecXd>& v) const {
  // q and v are unused here, since the Jacobian is a constant, that does
  // not depends on q or v.
  unused(q);
  unused(v);
  Eigen::MatrixXd J(2, tree()->get_num_velocities());
  J.setZero();
  J(LowerLimitForceIndexInLambda(), joint_velocity_index_) = 1;
  J(UpperLimitForceIndexInLambda(), joint_velocity_index_) = -1;
  return J.cast<AutoDiffXd>();
}
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
