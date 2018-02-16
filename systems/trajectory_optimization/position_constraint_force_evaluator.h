#pragma once

#include <memory>

#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Evaluates the generalized constraint force from
 * RigidBodyTree::positionConstraint.
 * Loop joint constraint is a position constraint.
 */
class PositionConstraintForceEvaluator
    : public GeneralizedConstraintForceEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraintForceEvaluator)

  /**
   * @param kinematics_cache_helper. The helper class to update the kinematics
   * cache. The kinematics cache is useful when computing the Jacobian of the
   * position constraint.
   */
  PositionConstraintForceEvaluator(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
          kinematics_cache_helper);

  ~PositionConstraintForceEvaluator() override{};

 protected:
  MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q) const override;

 private:
  mutable std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
      kinematics_cache_helper_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
