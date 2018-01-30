#pragma once

// This header file exists only to expose some internal implementation to unit
// test. DO NOT INCLUDE THIS HEADER FILE in your program!

#include <memory>

#include "drake/math/autodiff.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
// Forward declaration
template <typename Scalar>
class KinematicsCacheWithVHelper;
class GeneralizedConstraintForceEvaluator;

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
 */
class DirectTranscriptionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTranscriptionConstraint)

  /** Constructor
   * @param tree The RigidBodyTree whose trajectory will be optimized.
   * @param num_lambda The number of lambda on the right knot point.
   * @param kinematics_helper The kinematics helper that stores the kinematics
   * information for the position and velocity on the right knot.
   * @param generalized_constraint_force_evaluator The evaluator that computes
   * Jᵣᵀ*λᵣ on the right knot.
   */
  DirectTranscriptionConstraint(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper,
      std::unique_ptr<GeneralizedConstraintForceEvaluator>
          generalized_constraint_force_evaluator);

  ~DirectTranscriptionConstraint() override = default;

  template <typename Scalar, typename DerivedQL, typename DerivedVL,
            typename DerivedQR, typename DerivedVR, typename DerivedUR,
            typename DerivedLambdaR>
  typename std::enable_if<is_eigen_vector_of<DerivedQL, Scalar>::value &&
                              is_eigen_vector_of<DerivedVL, Scalar>::value &&
                              is_eigen_vector_of<DerivedQR, Scalar>::value &&
                              is_eigen_vector_of<DerivedVR, Scalar>::value &&
                              is_eigen_vector_of<DerivedUR, Scalar>::value &&
                              is_eigen_vector_of<DerivedLambdaR, Scalar>::value,
                          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>::type
  CompositeEvalInput(const Scalar& h, const Eigen::MatrixBase<DerivedQL>& q_l,
                     const Eigen::MatrixBase<DerivedVL>& v_l,
                     const Eigen::MatrixBase<DerivedQR>& q_r,
                     const Eigen::MatrixBase<DerivedVR>& v_r,
                     const Eigen::MatrixBase<DerivedUR>& u_r,
                     const Eigen::MatrixBase<DerivedLambdaR>& lambda_r) const {
    DRAKE_ASSERT(q_l.rows() == num_positions_);
    DRAKE_ASSERT(v_l.rows() == num_velocities_);
    DRAKE_ASSERT(q_r.rows() == num_positions_);
    DRAKE_ASSERT(v_r.rows() == num_velocities_);
    DRAKE_ASSERT(u_r.rows() == num_actuators_);
    DRAKE_ASSERT(lambda_r.rows() == num_lambda_);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(num_vars(), 1);
    x << h, q_l, v_l, q_r, v_r, u_r, lambda_r;
    return x;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  const RigidBodyTree<double>* tree_;
  const int num_positions_;
  const int num_velocities_;
  const int num_actuators_;
  const int num_lambda_;
  // Stores the kinematics cache at the right knot point.
  mutable std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>
      kinematics_helper1_;
  std::unique_ptr<GeneralizedConstraintForceEvaluator>
      generalized_constraint_force_evaluator_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
