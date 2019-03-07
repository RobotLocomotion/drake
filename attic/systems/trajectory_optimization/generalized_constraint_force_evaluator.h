#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/** This evaluator computes the generalized constraint force Jᵀλ ∈ ℝ ᴺᵛ, where
 * Nᵥ is the size of the generalized velocities.
 * The Jacobian J may or may not depend on generalized position q and/or other
 * variables.
 */
class GeneralizedConstraintForceEvaluator : public solvers::EvaluatorBase {
 public:
  /**
   * Constructor.
   * @param tree Note @p tree is aliased for the lifetime of of this object.
   * @param num_vars Number of variables, including λ.
   * @param lambda_size λ is a lambda_size x 1 vector.
   * @note the Jᵀλ ∈ ℝ ᴺᵛ, where Nᵥ is the size of the generalized velocities.
   * So the size of the output vector is always Nᵥ. To evaluate Jᵀλ, it may or
   * may not depend on variables such as contact force λ, generalized position
   * q, or some additional variables, so the size of the input variable to this
   * evaluator should be specified by the user.
   */
  GeneralizedConstraintForceEvaluator(const RigidBodyTree<double>& tree,
                                      int num_vars, int lambda_size);

  ~GeneralizedConstraintForceEvaluator() override {}

  /** Getter for the size of non_lambda part in the evaluator variables. */
  int non_lambda_size() const { return num_vars() - lambda_size_; }

  /** Getter for lambda_size. */
  int lambda_size() const { return lambda_size_; }

  /** Getter for the tree. */
  const RigidBodyTree<double>* tree() const { return tree_; }

  /**
   * Compose the input `x` to the Eval function, given λ and the part of x
   * that is not in λ.
   * This is a helper function, so that the user does not need to remember which
   * part of x corresponds to λ, and which part corresponds to non-λ.
   */
  template <typename DerivedNonLambda, typename DerivedLambda>
  typename std::enable_if<
      is_eigen_vector_of<DerivedNonLambda,
                         typename DerivedLambda::Scalar>::value &&
          is_eigen_vector<DerivedLambda>::value,
      VectorX<typename DerivedLambda::Scalar>>::type
  ComposeEvalInputVector(const Eigen::MatrixBase<DerivedNonLambda>& non_lambda,
                         const Eigen::MatrixBase<DerivedLambda>& lambda) const {
    using Scalar = typename DerivedLambda::Scalar;
    DRAKE_ASSERT(non_lambda.rows() == num_vars() - lambda_size_);
    DRAKE_ASSERT(lambda.rows() == lambda_size_);
    VectorX<Scalar> x(num_vars());
    x << non_lambda, lambda;
    return x;
  }

  /** Get the non-lambda part from the eval input vector. */
  template <typename Derived>
  auto GetLambdaFromEvalInputVector(const Eigen::MatrixBase<Derived>& x) const {
    return x.tail(lambda_size_);
  }

  /** Get the lambda part from the eval input vector. */
  template <typename Derived>
  auto GetNonLambdaFromEvalInputVector(
      const Eigen::MatrixBase<Derived>& x) const {
    return x.head(num_vars() - lambda_size_);
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "GeneralizedConstraintForceEvaluator does not support symbolic "
        "evaluation.");
  }

  template <typename DerivedX, typename DerivedY>
  typename std::enable_if<std::is_same<typename DerivedX::Scalar,
                                       typename DerivedY::Scalar>::value &&
                          is_eigen_vector<DerivedX>::value &&
                          is_eigen_vector<DerivedY>::value>::type
  DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                Eigen::MatrixBase<DerivedY>* y) const {
    // x contains non-λ and λ
    DRAKE_ASSERT(x.rows() == num_vars());
    const auto lambda = GetLambdaFromEvalInputVector(x);

    const auto J = EvalConstraintJacobian(x);
    *y = J.transpose() * lambda;
  }

  // Derived class implementation should compute the Jacobian J
  // that can be used to evaluate the generalized constraint force
  // Jᵀλ. J must be size Nv x n_λ.
  virtual Eigen::MatrixXd EvalConstraintJacobian(
      const Eigen::Ref<const Eigen::VectorXd>& x) const = 0;

  // Derived class implementation should compute the Jacobian J
  // that can be used to evaluate the generalized constraint force
  // Jᵀλ. J must be size Nv x n_λ.
  virtual MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& x) const = 0;

 private:
  const RigidBodyTree<double>* const tree_;
  const int lambda_size_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
