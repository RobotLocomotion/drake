#pragma once

#include <memory>
#include <utility>

#include "drake/solvers/constraint.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {

// TODO(eric.cousineau): Remove stopgap, and actually have Constraint and
// Cost be different classes. Consider using some common evaluation base.

/**
 * Stopgap definition that permits Cost to use functionality in Constraint.
 * Using an internal implementation permits child costs to inherit directly
 * from cost, thus be convertible to a cost.
 */
class Cost : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Cost)

  explicit Cost(const std::shared_ptr<Constraint>& impl)
      : Constraint(impl->num_constraints(), impl->num_vars()), impl_(impl) {
    // Costs may only be scalar.
    DRAKE_DEMAND(impl->num_constraints() == 1);
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const override;

 private:
  std::shared_ptr<Constraint> impl_;
};

/**
 * Stopgap class to provide functionality as constraint, but allow templates to
 * detect a difference from results from CreateConstraint and CreateCost.
 * @tparam C Constraint type to inherit from.
 */
template <typename C>
class CostShim : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CostShim)

  template <typename... Args>
  explicit CostShim(Args&&... args)
      : Cost(std::make_shared<C>(std::forward<Args>(args)...)) {}
};

/**
 * Implements a cost of the form @f Ax @f
 */
class LinearCost : public CostShim<LinearConstraint> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearCost)

  // Inherit forwarding constructor.
  using CostShim::CostShim;
};

/**
 * Implements a cost of the form @f .5 x'Qx + b'x @f
 */
class QuadraticCost : public CostShim<QuadraticConstraint> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticCost)

  // Inherit forwarding constructor.
  using CostShim::CostShim;
};

/**
 *  Implements a cost of the form P(x, y...) where P is a multivariate
 *  polynomial in x, y...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the MathematicalProgram::Binding (the individual scalar
 * elements of the given VariableList).
 */
class PolynomialCost : public CostShim<PolynomialConstraint> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialCost)

  // Inherit forwarding constructor.
  using CostShim::CostShim;
};

/**
 * A constraint that may be specified using a callable object.
 * @tparam F The function / functor's type.
 * @see detail::FunctionTraits.
 * @note This is presently in Cost as it is the only place used. Once Cost is
 * its own proper class, this name will be transitioned to FunctionCost.
 */
template <typename F>
class FunctionConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionConstraint)

  // Construct by copying from an lvalue.
  template <typename... Args>
  FunctionConstraint(const F& f, Args&&... args)
      : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(f) {}

  // Construct by moving from an rvalue.
  template <typename... Args>
  FunctionConstraint(F&& f, Args&&... args)
      : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(std::forward<F>(f)) {}

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

/**
 * A cost that may be specified using a callable object.
 * @tparam F The function / functor's type.
 * @see detail::FunctionTraits.
 */
template <typename F>
class FunctionCost : public CostShim<FunctionConstraint<F>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionCost)

  // Inherit forwarding constructor.
  // Must explicitly qualify type due to class-level template.
  using Base = CostShim<FunctionConstraint<F>>;
  using Base::Base;
};

}  // namespace solvers
}  // namespace drake
