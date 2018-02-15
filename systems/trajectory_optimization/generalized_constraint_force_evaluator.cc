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
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
