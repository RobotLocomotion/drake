#pragma once

// This header file exists only to expose some internal implementation to unit
// test. DO NOT INCLUDE THIS HEADER FILE in your program!

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/decision_variable.h"
#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"
namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Implements the constraint for the backward Euler integration
 * <pre>
 * qᵣ - qₗ = q̇ᵣ*h
 * Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
 * </pre>
 * where
 * qᵣ: The generalized position on the right knot.
 * qₗ: The generalized position on the left knot.
 * vᵣ: The generalized velocity on the right knot.
 * vₗ: The generalized velocity on the left knot.
 * uᵣ: The actuator input on the right knot.
 * Mᵣ: The inertia matrix computed from qᵣ.
 * λᵣ: The constraint force (e.g., contact force, joint limit force, etc) on the
 * right knot.
 * c(qᵣ, vᵣ): The Coriolis, gravity and centripedal force on the right knot.
 * h: The duration between the left and right knot.
 * Note that the robot might have many generalized constraint forces, such as
 * loop joint forces, ground contact forces, joint limit forces, etc. These
 * generalized constraint forces are additive, namely if the loop joint forces
 * are Jₗₒₒₚᵀ*λₗₒₒₚ, and the joint constraint forces are Jⱼₒᵢₙₜᵀ*λⱼₒᵢₙₜ, then
 * the total generalized constraint forces from both the loop joint are the
 * joint limits are ther sum Jₗₒₒₚᵀ*λₗₒₒₚ + Jⱼₒᵢₙₜᵀ*λⱼₒᵢₙₜ
 * So we will store a vector of GeneralizedConstraintForceEvaluator in this
 * class, each representing one term of generalized constraint force, and we
 * will evaluate each one of them to obtain the summed constraint forces.
 */
class DirectTranscriptionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTranscriptionConstraint)

  /** Factory method to create a DirectTranscriptionConstraint 
   * @param tree The RigidBodyTree whose trajectory will be optimized.
   * @param kinematics_helper The kinematics helper that stores the kinematics
   * information for the position and velocity on the right knot.
   * @param h The decision variable for the time interval
   * @param q_l The decision variables for the generalized position on the left
   * knot.
   * @param v_l The decision variables for the generalized velocity on the left
   * knot.
   * @param q_r The decision variables for the generalized position on the right
   * knot.
   * @param v_r The decision variables for the generalized velocity on the right
   * knot.
   * @param u_r The decision variables for the actuator input on the right knot.
   * @param constraint_force_evaluator_bindings A vector of
   * GeneralizedConstraintForceEvaluator, together with the bound variables for
   * each evaluator. Note that within the constructor,
   * constraint_force_evaluator_bindings will transfer the ownership of the
   * evaluators to the newly constructed object.
   */
  static solvers::Binding<DirectTranscriptionConstraint> Make(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
          kinematics_helper,
      const symbolic::Variable& h,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_l,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_l,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_r,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_r,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& u_r,
      const std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>&
          constraint_force_evaluator_bindings);

  ~DirectTranscriptionConstraint() override = default;

  const GeneralizedConstraintForceEvaluator*
  generalized_constraint_force_evaluator(int index) const {
    return generalized_constraint_force_evaluator_bindings_[index].first.get();
  }

 protected:
  /** Constructor
   * The input to this constructor is the same as the factory method Create(),
   * except for the additional input map_var_to_index. This mapping records
   * the index of each variable in the input to the Eval function.
   */
  DirectTranscriptionConstraint(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
          kinematics_helper,
      const symbolic::Variable& h,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_l,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_l,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_r,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_r,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& u_r,
      const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
      const std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>&
          constraint_force_evaluator_bindings);

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  const RigidBodyTree<double>* tree_;
  const int num_positions_;
  const int num_velocities_;
  const int num_actuators_;
  // Stores the kinematics cache at the right knot point.
  mutable std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
      kinematics_helper1_;
  // The indices of h, q_l, v_l, q_r, v_r, u_r in the aggregated_variables_.
  int h_index_;
  std::vector<int> q_l_indices_;
  std::vector<int> v_l_indices_;
  std::vector<int> q_r_indices_;
  std::vector<int> v_r_indices_;
  std::vector<int> u_r_indices_;
  // Stores the GeneralizedConstraintForceEvaluator, together with the indices
  // of the variables bound with each evaluator in the aggregated variables.
  // so
  // aggregated_variables_[generalized_constraint_force_evaluator_bindings_[i].second]
  // are the variables bound with the evaluator
  // generalized_constraint_force_evaluator_bindings_[i].first
  std::vector<std::pair<std::shared_ptr<GeneralizedConstraintForceEvaluator>,
                        std::vector<int>>>
      generalized_constraint_force_evaluator_bindings_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
