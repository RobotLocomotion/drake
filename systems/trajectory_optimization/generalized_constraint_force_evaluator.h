#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/** This evaluator computes the generalized constraint force Jᵀλ.
 * where the Jacobian J can depend on generalized position q
 */
class GeneralizedConstraintForceEvaluator : public solvers::EvaluatorBase {
 public:
  /**
   * Constructor.
   * @param tree Note this object is aliased for the lifetime of @p tree.
   * @param num_lambda λ is a num_lambda x 1 vector.
   */
  GeneralizedConstraintForceEvaluator(const RigidBodyTree<double>& tree,
                                      int num_lambda);

  /** Getter for num_lambda. */
  int num_lambda() const { return num_lambda_; }

  /** Getter for the tree. */
  const RigidBodyTree<double>* tree() const { return tree_; }

  /**
   * Composite the input `x` to the Eval function, given q and λ.
   * This is a helper function, so that the user does not need to remember the
   * order of q and λ in the input `x`.
   */
  template <typename DerivedQ, typename DerivedLambda>
  typename std::enable_if<
      is_eigen_vector_of<DerivedQ, typename DerivedQ::Scalar>::value &&
          is_eigen_vector_of<DerivedLambda, typename DerivedQ::Scalar>::value,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, 1>>::type
  CompositeEvalInputVector(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedLambda>& lambda) const {
    using Scalar = typename DerivedQ::Scalar;
    DRAKE_ASSERT(q.rows() == tree_->get_num_positions());
    DRAKE_ASSERT(lambda.rows() == num_lambda_);
    typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(q.rows() +
                                                        lambda.rows());
    x << q, lambda;
    return x;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const override;
  //
  // Computes the Jacobian J so as to evaluate the generalized constraint force
  // Jᵀλ.
  virtual MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q) const = 0;

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

  /**
   * @param kinematics_cache_helper. The helper class to update the kinematics
   * cache. The kinematics cache is useful when computing the Jacobian of the
   * position constraint.
   */
  PositionConstraintForceEvaluator(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
          kinematics_cache_helper)
      : GeneralizedConstraintForceEvaluator(tree,
                                            tree.getNumPositionConstraints()),
        kinematics_cache_helper_(kinematics_cache_helper) {}

 protected:
  MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q) const override;

 private:
  mutable std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
      kinematics_cache_helper_;
};

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
  JointLimitConstraintForceEvaluator(const RigidBodyTree<double>& tree,
                                     int joint_velocity_index);

  /**
   * The constraint force λ contains both the joint upper limit force λᵤ, and
   * the joint lower limit force λₗ. The following two method returns the
   * indices of λᵤ / λₗ in λ.
   */
  static constexpr int UpperLimitForceIndexInLambda() { return 0; }
  static constexpr int LowerLimitForceIndexInLambda() { return 1; }

 protected:
  MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& q) const override;

 private:
  const int joint_velocity_index_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
