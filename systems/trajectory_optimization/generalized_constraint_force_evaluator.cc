#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
using plants::KinematicsCacheWithVHelper;

// This evaluator computes the generalized constraint force Jᵀλ.
GeneralizedConstraintForceEvaluator::GeneralizedConstraintForceEvaluator(
    const RigidBodyTree<double>& tree, int num_vars, int lambda_size)
    : EvaluatorBase(tree.get_num_velocities(), num_vars,
                    "generalized constraint force"),
      tree_{&tree},
      lambda_size_(lambda_size) {}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  // x contains non-λ and λ
  DRAKE_ASSERT(x.rows() == num_vars());
  const auto lambda = GetLambdaFromEvalInputVector(x);

  const auto J = EvalConstraintJacobian(x);
  y = J.transpose() * lambda;
}

PositionConstraintForceEvaluator::PositionConstraintForceEvaluator(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
        kinematics_cache_helper)
    : GeneralizedConstraintForceEvaluator(
          tree, tree.get_num_positions() + tree.getNumPositionConstraints(),
          tree.getNumPositionConstraints()),
      kinematics_cache_helper_(kinematics_cache_helper) {}

MatrixX<AutoDiffXd> PositionConstraintForceEvaluator::EvalConstraintJacobian(
    const Eigen::Ref<const AutoDiffVecXd>& x) const {
  const auto& q = x.head(tree()->get_num_positions());
  auto kinsol = kinematics_cache_helper_->UpdateKinematics(q, tree());
  return tree()->positionConstraintsJacobian(kinsol);
}

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
