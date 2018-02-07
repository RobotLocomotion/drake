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
      std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>
          kinematics_helper);

  ~DirectTranscriptionConstraint() override = default;

  /**
   * Adds a GeneralizedConstraintForceEvaluator.
   * Note that by adding a new generalized constraint force evaluator, it also
   * increases DirectTranscriptionConstraint::num_vars() by
   * evaluator.num_lambda(). Namely the last evaluator.num_lambda() variables
   * bounded with this DirectTranscriptionConstraint, is going to be used to
   * compute the generalized constraint force in this @p evaluator.
   * @param evaluator This will evaluate a generalized constraint force. If the
   * user has evaluator1 that computes the loop joint force, and evaluator2 that
   * computes the contact force from the foot toe, then the user can call this
   * function for twice
   * AddGeneralizedConstraintForceEvaluator(evaluator1);
   * AddGeneralizedConstraintForceEvaluator(evaluator2);
   * The user doesn't have to create one evaluator, that computes the summation
   * of all generalized constraint forces.
   */
  void AddGeneralizedConstraintForceEvaluator(
      std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator);

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
  int num_lambda_;
  // Stores the kinematics cache at the right knot point.
  mutable std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>>
      kinematics_helper1_;
  // Stores the GeneralizedConstraintForceEvaluator
  std::vector<std::unique_ptr<GeneralizedConstraintForceEvaluator>>
      generalized_constraint_force_evaluators_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
