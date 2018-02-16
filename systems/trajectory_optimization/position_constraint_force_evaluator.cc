#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
using plants::KinematicsCacheWithVHelper;

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
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
