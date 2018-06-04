#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Implements the trajectory optimization for a RigidBodyTree.
 * Trajectory optimization for RigidBodyTree is special, because the dynamics
 * of the tree has some special structures.
 * 1. Since RigidBodyTree has a second order dynamics, its dynamics can be
 * separated as the time derivative on the generalized position, and the time
 * derivative on the generalized velocities.
 * 2. Its generalized acceleration can be affected by the external force, under
 * the term Jᵀλ. We can optimize over λ as decision variables.
 * 3. The kinematics cache can be reused in each knot of the trajectory, so we
 * will store the kinematics cache for each knot.
 *
 * By default, the generalized constraint force Jᵀλ only includes those from
 * RigidBodyTree::PositionConstraint().
 *
 * @note The user MUST call this Compile function before solving the
 * optimization program, and after all the generalized constraint force Jᵀλ has
 * been added to the program.
 */
class RigidBodyTreeMultipleShooting : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeMultipleShooting)

  /**
   * Constructor.
   * @param tree The RigidBodyTree whose trajectory will be optimized.
   * @param num_time_samples The total number of knots in the trajectory.
   * @param minimum_timestep The minimum of the time step.
   * @param maximum_timestep The maximum of the time step.
   */
  RigidBodyTreeMultipleShooting(const RigidBodyTree<double>& tree,
                                int num_time_samples, double minimum_timestep,
                                double maximum_timestep);

  trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
      const override;

  trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
      const override;

  const solvers::MatrixXDecisionVariable& GeneralizedPositions() const {
    return q_vars_;
  }

  const solvers::MatrixXDecisionVariable& GeneralizedVelocities() const {
    return v_vars_;
  }

  const solvers::MatrixXDecisionVariable& PositionConstraintForces() const {
    return position_constraint_lambda_vars_;
  }

  ~RigidBodyTreeMultipleShooting() override {}

  const RigidBodyTree<double>* tree() const { return tree_; }

  /** Getter for the kinematics cache helper. */
  const std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
  kinematics_cache_with_v_helpers(int index) const {
    return kinematics_cache_with_v_helpers_[index];
  }

  /**
   * Activate the joint limit constraints within an interval for a certain
   * joint. This function makes the following changes to the optimization:
   * 1. It adds two decision variables λᵤ / λₗ to the optimization.
   *    λᵤ / λₗ are the joint limit force from upper bound and lower bound
   *    respectively.
   * 2. It adds the joint limit force to the constraint force Jᵀλ, when
   *    computing the dynamics for transcription.
   * 3. It adds complementarity constraint
   *    (qᵤ - q) * λᵤ = 0
   *    (q - qₗ) * λₗ = 0
   *    where qᵤ is the joint upper bound, and qₗ is the joint lower bound.
   * 4. It adds the constraint
   *    qₗ ≤ q ≤ qᵤ
   *    λᵤ ≥ 0, λₗ ≥ 0
   *    to the optimization.
   */
  solvers::VectorDecisionVariable<2> AddJointLimitImplicitConstraint(
      int interval_index, int joint_position_index, int joint_velocity_index,
      double joint_lower_bound, double joint_upper_bound);

  /**
   * Adds the direct transcription constraint to the optimization program.
   * The user MUST call this Compile function before solving the optimization
   * program, and after all the generalized constraint force Jᵀλ has been
   * added to the program.
   */
  void Compile();

 protected:
  int num_positions() const { return num_positions_; }

  int num_velocities() const { return num_velocities_; }

  void AddGeneralizedConstraintForceEvaluatorToTranscription(
      int interval_index,
      std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&
          evaluator_lambda);

 private:
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Store system-relevant data for e.g. computing the derivatives during
  // trajectory reconstruction.
  const RigidBodyTree<double>* tree_{nullptr};
  const int num_positions_;
  const int num_velocities_;
  const int num_actuators_;
  // constraint_force_evaluator_bindings[i] stores the evaluator that computes
  // one generalized constraint force Jᵀλ at knot i, together with the variables
  // bound with the evaluator
  std::vector<
      std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>>
      constraint_force_evaluator_bindings;
  std::vector<std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>>
      kinematics_cache_helpers_;
  std::vector<std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>>
      kinematics_cache_with_v_helpers_;
  solvers::MatrixXDecisionVariable q_vars_;
  solvers::MatrixXDecisionVariable v_vars_;
  solvers::MatrixXDecisionVariable position_constraint_lambda_vars_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
