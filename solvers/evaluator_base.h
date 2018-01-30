#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {

/**
 * Provides an abstract interface to represent an expression, mapping a fixed
 * or dynamic number of inputs to a fixed number of outputs, that may be
 * evaluated on a scalar type of double or AutoDiffXd.
 *
 * These objects, and its derivatives, are meant to be bound to a given set
 * of variables using the Binding<> class.
 */
class EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EvaluatorBase)

  virtual ~EvaluatorBase() {}

  /**
   * Evaluates the expression with a scalar type of double.
   * @param x A `num_vars` x 1 input vector.
   * @param[out] y A `num_outputs` x 1 output vector.
   */
  // TODO(bradking): consider using a Ref for `y`.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
            Eigen::VectorXd& y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
  }

  /**
   * Evaluates the expression with a scalar type of AutoDiffXd.
   * @param x A `num_vars` x 1 input vector.
   * @param[out] y A `num_outputs` x 1 output vector.
   */
  // TODO(eric.cousineau): Move this to DifferentiableConstraint derived class
  // if/when we need to support non-differentiable functions (at least, if
  // DifferentiableConstraint is ever implemented).
  void Eval(const Eigen::Ref<const AutoDiffVecXd>& x,
            // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
            AutoDiffVecXd& y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
  }

  /**
   * Set a human-friendly description for the evaluator.
   */
  inline void set_description(const std::string& description) {
    description_ = description;
  }

  /**
   * Getter for a human-friendly description for the evaluator.
   */
  inline const std::string& get_description() const { return description_; }

  /**
   * Getter for the number of variables, namely the number of rows in x, as
   * used in Eval(x, y).
   */
  int num_vars() const { return num_vars_; }

  /**
   * Getter for the number of outputs, namely the number of rows in y, as used
   * in Eval(x, y).
   */
  int num_outputs() const { return num_outputs_; }

 protected:
  /**
   * Constructs a evaluator.
   * @param num_outputs. The number of rows in the output.
   * @param num_vars. The number of rows in the input.
   * If the input dimension is not known, then set `num_vars` to Eigen::Dynamic.
   * @param description A human-friendly description.
   * @see Eval(...)
   */
  EvaluatorBase(int num_outputs, int num_vars,
                const std::string& description = "")
      : num_vars_(num_vars),
        num_outputs_(num_outputs),
        description_(description) {}

  /**
   * Implements expression evaluation for scalar type double.
   * @param x Input vector.
   * @param y Output vector.
   * @pre x must be of size `num_vars` x 1.
   * @post y will be of size `num_outputs` x 1.
   */
  virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::VectorXd& y) const = 0;

  /**
   * Implements expression evaluation for scalar type AutoDiffXd.
   * @param x Input vector.
   * @param y Output vector.
   * @pre x must be of size `num_vars` x 1.
   * @post y will be of size `num_outputs` x 1.
   */
  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      AutoDiffVecXd& y) const = 0;

  // Setter for the number of outputs.
  // This method is only meant to be called, if the sub-class structure permits
  // to change the number of outputs. One example is LinearConstraint in
  // solvers/Constraint.h, which can change the number of outputs, if the
  // matrix in the linear constraint is resized.
  void set_num_outputs(int num_outputs) { num_outputs_ = num_outputs; }

 private:
  int num_vars_{};
  int num_outputs_{};
  std::string description_;
};

/**
 * Implements an evaluator of the form P(x, y...) where P is a multivariate
 * polynomial in x, y, ...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the Binding<> (the individual scalar elements of the
 * given VariableList).
 */
class PolynomialEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialEvaluator)

  /**
   * Constructs a polynomial evaluator given a set of polynomials and the
   * corresponding variables.
   * @param polynomials Polynomial vector, a `num_outputs` x 1 vector.
   * @param poly_vars Polynomial variables, a `num_vars` x 1 vector.
   */
  PolynomialEvaluator(const VectorXPoly& polynomials,
                      const std::vector<Polynomiald::VarType>& poly_vars)
      : EvaluatorBase(polynomials.rows(), poly_vars.size()),
        polynomials_(polynomials),
        poly_vars_(poly_vars) {}

  const VectorXPoly& polynomials() const { return polynomials_; }

  const std::vector<Polynomiald::VarType>& poly_vars() const {
    return poly_vars_;
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

  const VectorXPoly polynomials_;
  const std::vector<Polynomiald::VarType> poly_vars_;

  // To avoid repeated allocation, reuse a map for the evaluation point.
  // Do not assume that these values will persist across invocations!
  // TODO(eric.cousineau): Consider removing this for thread safety?
  mutable std::map<Polynomiald::VarType, double> double_evaluation_point_temp_;
  mutable std::map<Polynomiald::VarType, AutoDiffXd>
      taylor_evaluation_point_temp_;
};

/**
 * An evaluator that may be specified using a callable object. Consider
 * constructing these instances using MakeFunctionEvaluator(...).
 * @tparam F The function / functor's type.
 * @see detail::FunctionTraits.
 */
template <typename F>
class FunctionEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionEvaluator)

  /**
   * Constructs an instance by copying from an lvalue or rvalue of `F`.
   * @tparam FF Perfect-forwarding type of `F` (e.g., `const F&`, `F&&`).
   * @param f The callable object. If rvalue, this value will be std::move'd.
   * Otherwise, it will be copied.
   * @param args Arguments to be forwarded to EvaluatorBase constructor.
   */
  template <typename FF, typename... Args>
  FunctionEvaluator(FF&& f, Args&&... args)
      : EvaluatorBase(detail::FunctionTraits<F>::numOutputs(f),
                      detail::FunctionTraits<F>::numInputs(f),
                      std::forward<Args>(args)...),
        f_(std::forward<FF>(f)) {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }

  const F f_;
};

/**
 * Creates a FunctionEvaluator instance bound to a given callable object.
 * @tparam FF Perfect-forwarding type of `F` (e.g., `const F&`, `F&&`).
 * @param f Callable function object.
 * @return An implementation of EvaluatorBase using the callable object.
 * @see detail::FunctionTraits.
 * @relates FunctionEvaluator
 */
template <typename FF>
std::shared_ptr<EvaluatorBase> MakeFunctionEvaluator(FF&& f) {
  using F = std::decay_t<FF>;
  return std::make_shared<FunctionEvaluator<F>>(std::forward<FF>(f));
}

}  // namespace solvers
}  // namespace drake
