#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/math/autodiff.h"
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

  // TODO(bradking): consider using a Ref for `y`.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  /**
   * Evaluates the expression.
   * @param[in] x A `num_vars` x 1 input vector.
   * @param[out] y A `num_outputs` x 1 output vector.
   */
  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd* y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
    DRAKE_ASSERT(y->rows() == num_outputs_);
  }

  // TODO(eric.cousineau): Move this to DifferentiableConstraint derived class
  // if/when we need to support non-differentiable functions (at least, if
  // DifferentiableConstraint is ever implemented).
  /**
   * Evaluates the expression.
   * @param[in] x A `num_vars` x 1 input vector.
   * @param[out] y A `num_outputs` x 1 output vector.
   */
  void Eval(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
    DRAKE_ASSERT(y->rows() == num_outputs_);
  }

  /**
   * Evaluates the expression.
   * @param[in] x A `num_vars` x 1 input vector.
   * @param[out] y A `num_outputs` x 1 output vector.
   */
  void Eval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
            VectorX<symbolic::Expression>* y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
    DRAKE_ASSERT(y->rows() == num_outputs_);
  }

  /**
   * Set a human-friendly description for the evaluator.
   */
  void set_description(const std::string& description) {
    description_ = description;
  }

  /**
   * Getter for a human-friendly description for the evaluator.
   */
  const std::string& get_description() const { return description_; }

  /**
   * Formats this evaluator into the given stream using `vars` for the bound
   * decision variable names.
   *
   * The size of `vars` must match the `num_vars()` declared by this evaluator.
   * (If `num_vars()` is `Eigen::Dynamic`, then `vars` may be any size.)
   */
  std::ostream& Display(std::ostream& os,
                        const VectorX<symbolic::Variable>& vars) const;

  /**
   * Formats this evaluator into the given stream, without displaying the
   * decision variables it is bound to.
   */
  std::ostream& Display(std::ostream& os) const;

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

  /**
   * Set the sparsity pattern of the gradient matrix ∂y/∂x (the gradient of
   * y value in Eval, w.r.t x in Eval) . gradient_sparsity_pattern contains
   * *all* the pairs of (row_index, col_index) for which the corresponding
   * entries could have non-zero value in the gradient matrix ∂y/∂x.
   */
  void SetGradientSparsityPattern(
      const std::vector<std::pair<int, int>>& gradient_sparsity_pattern);

  /**
   * Returns the vector of (row_index, col_index) that contains all the entries
   * in the gradient of Eval function (∂y/∂x) whose value could be non-zero,
   * namely if ∂yᵢ/∂xⱼ could be non-zero, then the pair (i, j) is in
   * gradient_sparsity_pattern.
   * @retval gradient_sparsity_pattern If nullopt, then we regard all entries of
   * the gradient as potentially non-zero.
   */
  const std::optional<std::vector<std::pair<int, int>>>&
      gradient_sparsity_pattern() const {
    return gradient_sparsity_pattern_;
  }

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
                      Eigen::VectorXd* y) const = 0;

  /**
   * Implements expression evaluation for scalar type AutoDiffXd.
   * @param x Input vector.
   * @param y Output vector.
   * @pre x must be of size `num_vars` x 1.
   * @post y will be of size `num_outputs` x 1.
   */
  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      AutoDiffVecXd* y) const = 0;

  /**
   * Implements expression evaluation for scalar type symbolic::Expression.
   * @param[in] x Input vector.
   * @param[out] y Output vector.
   * @pre x must be of size `num_vars` x 1.
   * @post y will be of size `num_outputs` x 1.
   */
  virtual void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                      VectorX<symbolic::Expression>* y) const = 0;

  /**
   * NVI implementation of Display. The default implementation will report
   * the NiceTypeName, get_description, and list the bound variables.
   * Subclasses may override to customize the message.
   * @pre vars size is consistent with num_vars".
   */
  virtual std::ostream& DoDisplay(
      std::ostream& os, const VectorX<symbolic::Variable>& vars) const;

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
  // gradient_sparsity_pattern_ records the pair (row_index, col_index) that
  // contains the non-zero entries in the gradient of the Eval
  // function. Note that if the entry (row_index, col_index) *can* be non-zero
  // for certain value of x, then it should be included in
  // gradient_sparsity_patten_. When gradient_sparsity_pattern_.has_value() =
  // false, the gradient matrix is regarded as non-sparse, i.e., every entry of
  // the gradient matrix can be non-zero.
  std::optional<std::vector<std::pair<int, int>>> gradient_sparsity_pattern_;
};

/**
 * Print out the evaluator.
 */
std::ostream& operator<<(std::ostream& os, const EvaluatorBase& e);

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
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PolynomialEvaluator does not support symbolic evaluation.");
  }

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
      : EvaluatorBase(internal::FunctionTraits<F>::numOutputs(f),
                      internal::FunctionTraits<F>::numInputs(f),
                      std::forward<Args>(args)...),
        f_(std::forward<FF>(f)) {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    y->resize(internal::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 internal::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y->rows()) ==
                 internal::FunctionTraits<F>::numOutputs(f_));
    internal::FunctionTraits<F>::eval(f_, x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    y->resize(internal::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 internal::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y->rows()) ==
                 internal::FunctionTraits<F>::numOutputs(f_));
    internal::FunctionTraits<F>::eval(f_, x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "FunctionEvaluator does not support symbolic evaluation.");
  }

  const F f_;
};

/**
 * Creates a FunctionEvaluator instance bound to a given callable object.
 * @tparam FF Perfect-forwarding type of `F` (e.g., `const F&`, `F&&`).
 * @param f Callable function object.
 * @return An implementation of EvaluatorBase using the callable object.
 * @relates FunctionEvaluator
 */
template <typename FF>
std::shared_ptr<EvaluatorBase> MakeFunctionEvaluator(FF&& f) {
  using F = std::decay_t<FF>;
  return std::make_shared<FunctionEvaluator<F>>(std::forward<FF>(f));
}

/**
 * Defines a simple evaluator with no outputs that takes a callback function
 * pointer.  This is intended for debugging / visualization of intermediate
 * results during an optimization (for solvers that support it).
 */
class VisualizationCallback : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VisualizationCallback)

  typedef std::function<void(const Eigen::Ref<const Eigen::VectorXd>&)>
      CallbackFunction;

  VisualizationCallback(int num_inputs, const CallbackFunction& callback,
                        const std::string& description = "")
      : EvaluatorBase(0, num_inputs, description), callback_(callback) {}

  void EvalCallback(const Eigen::Ref<const Eigen::VectorXd>& x) const {
    DRAKE_ASSERT(x.size() == num_vars());
    callback_(x);
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DRAKE_ASSERT(x.size() == num_vars());
    y->resize(0);
    callback_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DRAKE_ASSERT(x.size() == num_vars());
    y->resize(0);
    callback_(math::autoDiffToValueMatrix(x));
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "VisualizationCallback does not support symbolic evaluation.");
  }

  const CallbackFunction callback_;
};

}  // namespace solvers
}  // namespace drake
