#pragma once

#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Evaluates the joint limit constraint force.
 * For a single joint (revolute or prismatic), whose index in the velocity
 * vector is i, its joint limit force has the form
 * [0, 0, ..., 0, -λᵤ+λₗ, 0, ... ,0], that only the i'th entry is non-zero.
 * where λᵤ / λₗ are the joint limit force from upper / lower limit
 * respectively. We assume that both λᵤ and λₗ are non-negative.
 */
class JointLimitConstraintForceEvaluator
    : public GeneralizedConstraintForceEvaluator {
 public:
  /** Constructor.
   * @param tree The tree on which the joint limit force is evaluated.
   * @param joint_velocity_index The joint whose velocity index equals to
   * joint_velocity_index has joint limit force.
   */
  JointLimitConstraintForceEvaluator(const RigidBodyTree<double>& tree,
                                     int joint_velocity_index);

  ~JointLimitConstraintForceEvaluator() override {}

  /**
   * The constraint force λ contains both the joint upper limit force λᵤ, and
   * the joint lower limit force λₗ. The following two method returns the
   * indices of λᵤ / λₗ in λ.
   */
  static constexpr int UpperLimitForceIndexInLambda() { return 0; }
  static constexpr int LowerLimitForceIndexInLambda() { return 1; }

 protected:
  Eigen::MatrixXd EvalConstraintJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& x) const override;

  MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& x) const override;

 private:
  const int joint_velocity_index_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
