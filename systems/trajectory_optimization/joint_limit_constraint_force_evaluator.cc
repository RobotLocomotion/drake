#include "drake/systems/trajectory_optimization/joint_limit_constraint_force_evaluator.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
JointLimitConstraintForceEvaluator::JointLimitConstraintForceEvaluator(
    const RigidBodyTree<double>& tree, int joint_velocity_index)
    : GeneralizedConstraintForceEvaluator(tree, 2, 2),
      joint_velocity_index_(joint_velocity_index) {}

MatrixX<AutoDiffXd> JointLimitConstraintForceEvaluator::EvalConstraintJacobian(
    const Eigen::Ref<const AutoDiffVecXd>& x) const {
  // x is unused here, since the Jacobian is a constant, that does not depends
  // on x.
  unused(x);
  Eigen::Matrix2Xd J(2, tree()->get_num_velocities());
  J.setZero();
  J(LowerLimitForceIndexInLambda(), joint_velocity_index_) = 1;
  J(UpperLimitForceIndexInLambda(), joint_velocity_index_) = -1;
  return J.cast<AutoDiffXd>();
}
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
