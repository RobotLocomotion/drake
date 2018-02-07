#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

/** This evaluator computes the generalized constraint force Jᵀλ.
 */
class GeneralizedConstraintForceEvaluator : public solvers::EvaluatorBase {
 public:
  GeneralizedConstraintForceEvaluator(
      const RigidBodyTree<double>& tree, int num_lambda,
      std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>
          kinematics_helper);

  // Getter for num_lambda.
  int num_lambda() const { return num_lambda_; }

  // Getter for the tree.
  const RigidBodyTree<double>* tree() const { return tree_; }

 protected:
  virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::VectorXd& y) const override;

  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      AutoDiffVecXd& y) const override;
  //
  // Computes the Jacobian J so as to evaluate the generalized constraint force
  // Jᵀλ.
  virtual MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q,
      const Eigen::Ref<const AutoDiffVecXd>& v) const = 0;

 private:
  const RigidBodyTree<double>* tree_;
  const int num_lambda_;
};

/**
 * Evaluates the generalized constraint force from
 * RigidBodyTree::positionConstraint.
 * Loop joint constraint is a position constraint.
 */
class PositionConstraintForceEvaluator
    : public GeneralizedConstraintForceEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraintForceEvaluator)

  PositionConstraintForceEvaluator(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<systems::plants::KinematicsCacheHelper<AutoDiffXd>>
          kinematics_cache_helper)
      : GeneralizedConstraintForceEvaluator(tree,
                                            tree.getNumPositionConstraints()),
        kinematics_cache_helper_(kinematics_cache_helper) {}

 protected:
  virtual MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q,
      const Eigen::Ref<const AutoDiffVecXd>& v) const override;

 private:
  mutable std::shared_ptr<systems::plants::KinematicsCacheHelper<AutoDiffXd>>
      kinematics_cache_helper_;
};

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
 */
// class RigidBodyTreeMultipleShooting : public MultipleShooting {
// public:
//  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeMultipleShooting)
//
//  /**
//   * Constructor.
//   * @param tree The RigidBodyTree whose trajectory will be optimized.
//   * @param num_time_samples The total number of knots in the trajectory.
//   * @param minimum_timestep The minimum of the time step.
//   * @param maximum_timestep The maximum of the time step.
//   */
//  RigidBodyTreeMultipleShooting(const RigidBodyTree<double>& tree,
//                                int num_time_samples, double minimum_timestep,
//                                double maximum_timestep);
//
//  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const override;
//
//  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const override;
//
//  const solvers::MatrixXDecisionVariable& GeneralizedPositions() const {
//    return q_vars_;
//  }
//
//  const solvers::MatrixXDecisionVariable& GeneralizedVelocities() const {
//    return v_vars_;
//  }
//
//  const std::vector<solvers::VectorXDecisionVariable>& ConstraintForces()
//      const {
//    return lambda_vars_;
//  }
//
//  ~RigidBodyTreeMultipleShooting() override {}
//
//  const RigidBodyTree<double>* tree() const { return tree_; }
//
//  const std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>
//  kinematics_cache_with_v_helpers(int index) const {
//    return kinematics_cache_with_v_helpers_[index];
//  }
//
// protected:
//  int num_positions() const { return num_positions_; }
//
//  int num_velocities() const { return num_velocities_; }
//
//  const std::vector<int>& num_lambdas() const { return num_lambdas_; }
//
//  virtual std::unique_ptr<GeneralizedConstraintForceEvaluator>
//  DoConstructGeneralizedConstraintForceEvaluator(int index) const;
//
//  virtual void DoAddCollocationOrTranscriptionConstraint();
//
// private:
//  void DoAddRunningCost(const symbolic::Expression& e) override;
//
//  // Store system-relevant data for e.g. computing the derivatives during
//  // trajectory reconstruction.
//  const RigidBodyTree<double>* tree_{nullptr};
//  const int num_positions_;
//  const int num_velocities_;
//  const std::vector<int> num_lambdas_;
//  std::vector<std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>>
//      kinematics_cache_with_v_helpers_;
//  solvers::MatrixXDecisionVariable q_vars_;
//  solvers::MatrixXDecisionVariable v_vars_;
//  std::vector<solvers::VectorXDecisionVariable> lambda_vars_;
//};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
