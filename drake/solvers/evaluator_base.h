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
 * evaluated on a scalar type of double or AutoDiff.
 * These objects, and its derivatives, are meant to be bound to a given set
 * of variables using the Binding<> class.
 *
 * EvaluateBase is not copyable, nor movable.
 */
class EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EvaluatorBase)

  virtual ~EvaluatorBase() {}

  // TODO(bradking): consider using a Ref for `y`.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
            Eigen::VectorXd& y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
  }
  // Move this to DifferentiableConstraint derived class if/when we
  // need to support non-differentiable functions (at least, if
  // DifferentiableConstraint is ever implemented).
  void Eval(const Eigen::Ref<const AutoDiffVecXd>& x,
            // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
            AutoDiffVecXd& y) const {
    DRAKE_ASSERT(x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic);
    DoEval(x, y);
  }

  inline void set_description(const std::string& description) {
    description_ = description;
  }
  inline const std::string& get_description() const { return description_; }

  /// Getter for the number of variables, namely the number of rows in x, as
  /// used in Eval(x, y).
  int num_vars() const { return num_vars_; }

  /// Getter for the number of outputs, namely the number of rows in y, as used
  /// in Eval(x, y).
  int num_outputs() const { return num_outputs_; }

 protected:
  /// Constructs a evaluator which has \p num_constraints rows, with input
  /// variables to Eval a \p num_vars x 1 vector.
  /// @param num_outputs. The number of rows in the output, namely
  /// in Constraint::Eval(x, y), y should be a \p num_outputs x 1 vector.
  /// @param num_vars. The number of rows in the input, namely in
  /// Constraint::Eval(x, y), x should be a \p num_vars x 1 vector.
  /// If the input dimension is not known, then set \p num_vars to
  /// Eigen::Dynamic.
  EvaluatorBase(int num_outputs, int num_vars,
                const std::string& description = "")
      : num_vars_(num_vars),
        num_outputs_(num_outputs),
        description_(description) {}

  virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::VectorXd& y) const = 0;

  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      AutoDiffVecXd& y) const = 0;

 private:
  int num_vars_{};
  int num_outputs_{};
  std::string description_;
};
/*
 * Implements an evaluator of the form P(x, y...) where P is a multivariate
 * polynomial in x, y, ...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the MathematicalProgram::Binding (the individual scalar
 * elements of the given VariableList).
 */
class PolynomialEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialEvaluator)

  PolynomialEvaluator(const VectorXPoly& polynomials,
                      const std::vector<Polynomiald::VarType>& poly_vars)
      : EvaluatorBase(polynomials.rows(), poly_vars.size()),
        polynomials_(polynomials),
        poly_vars_(poly_vars) {}

  const VectorXPoly& polynomials() const { return polynomials_; }

  const std::vector<Polynomiald::VarType>& poly_vars() const {
    return poly_vars_;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  const VectorXPoly polynomials_;
  const std::vector<Polynomiald::VarType> poly_vars_;

  // To avoid repeated allocation, reuse a map for the evaluation point.
  // TODO(eric.cousineau): Consider removing this for thread safety?
  mutable std::map<Polynomiald::VarType, double> double_evaluation_point_;
  mutable std::map<Polynomiald::VarType, AutoDiffXd> taylor_evaluation_point_;
};

/**
 * An evaluator that may be specified using a callable object.
 * @tparam F The function / functor's type.
 * @see detail::FunctionTraits.
 */
template <typename F>
class FunctionEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionEvaluator)

  // Construct by copying from an lvalue or rvalue.
  template <typename TF, typename... Args>
  FunctionEvaluator(TF&& f, Args&&... args)
      : EvaluatorBase(detail::FunctionTraits<F>::numOutputs(f),
                      detail::FunctionTraits<F>::numInputs(f),
                      std::forward<Args>(args)...),
        f_(std::forward<TF>(f)) {}

 protected:
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

 private:
  const F f_;
};

template <typename F>
std::shared_ptr<EvaluatorBase> MakeFunctionEvaluator(F&& f) {
  using FC = FunctionEvaluator<std::decay_t<F>>;
  return std::make_shared<FC>(std::forward<F>(f));
}

}  // namespace solvers
}  // namespace drake
