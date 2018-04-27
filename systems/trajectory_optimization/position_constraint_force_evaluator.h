#pragma once

#include <memory>

#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Evaluates the generalized constraint force from
 * RigidBodyTree::positionConstraint. For example, loop joint constraint is a
 * position constraint.
 */
class PositionConstraintForceEvaluator
    : public GeneralizedConstraintForceEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraintForceEvaluator)

  /** Constructor.
   * @param tree The tree on which the position constraint force is evaluated.
   * @param kinematics_cache_helper. The helper class to update the kinematics
   * cache. The kinematics cache is useful when computing the Jacobian of the
   * position constraint.
   */
  PositionConstraintForceEvaluator(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
          kinematics_cache_helper);

  ~PositionConstraintForceEvaluator() override {}

  /** The size of the generalized position vector to evaluate the position
   * constraint Jacobian. */
  int generalized_positions_size() const { return non_lambda_size(); }

 protected:
  Eigen::MatrixXd EvalConstraintJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& q) const override;

  MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q) const override;

 private:
  mutable std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
      kinematics_cache_helper_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
