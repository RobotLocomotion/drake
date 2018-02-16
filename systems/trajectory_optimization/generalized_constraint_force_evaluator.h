#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/** This evaluator computes the generalized constraint force Jᵀλ.
 * where the Jacobian J can depend on generalized position q and/or other
 * variables.
 */
class GeneralizedConstraintForceEvaluator : public solvers::EvaluatorBase {
 public:
  /**
   * Constructor.
   * @param tree Note @p tree is aliased for the lifetime of of this object.
   * @param num_vars Number of variables, including λ.
   * @param lambda_size λ is a lambda_size x 1 vector.
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
      Eigen::Matrix<typename DerivedLambda::Scalar, Eigen::Dynamic, 1>>::type
  ComposeEvalInputVector(const Eigen::MatrixBase<DerivedNonLambda>& non_lambda,
                         const Eigen::MatrixBase<DerivedLambda>& lambda) const {
    using Scalar = typename DerivedLambda::Scalar;
    DRAKE_ASSERT(non_lambda.rows() == num_vars() - lambda_size_);
    DRAKE_ASSERT(lambda.rows() == lambda_size_);
    typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(num_vars());
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
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const override;
  //
  // Computes the Jacobian J so as to evaluate the generalized constraint force
  // Jᵀλ.
  virtual MatrixX<AutoDiffXd> EvalConstraintJacobian(
      const Eigen::Ref<const AutoDiffVecXd>& x) const = 0;

 private:
  const RigidBodyTree<double>* tree_;
  const int lambda_size_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
