#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

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
class Cost : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Cost)

  explicit Cost(size_t num_vars) : EvaluatorBase(1, num_vars) {}
};

/**
 * Stopgap definition that permits CostShim to pass a constructed Constraint
 * instance, and propogate the information to the Cost-wrapped Constraint.
 */
class CostShimBase : public Cost {
 protected:
  explicit CostShimBase(const std::shared_ptr<Constraint>& impl)
      : Cost(impl->num_vars()), impl_(impl) {
    // Costs may only be scalar.
    DRAKE_DEMAND(impl->num_constraints() == 1);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const override;

  const std::shared_ptr<Constraint>& impl() const { return impl_; }

 private:
  const std::shared_ptr<Constraint> impl_;
};

/**
 * Stopgap class to provide functionality as constraint, but allow templates to
 * detect a difference from results from CreateConstraint and CreateCost.
 * @tparam C Constraint type to inherit from.
 */
template <typename C>
class CostShim : public CostShimBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CostShim)

  // By construction, this will be the correct instance, so we will use a
  // static cast rather than dynamic cast.
  template <typename... Args>
  explicit CostShim(Args&&... args)
      : CostShimBase(std::make_shared<C>(std::forward<Args>(args)...)),
        constraint_(std::static_pointer_cast<C>(impl())) {}

 protected:
  const std::shared_ptr<C>& constraint() const { return constraint_; }

 private:
  const std::shared_ptr<C> constraint_;
};

/**
 * Implements a cost of the form @f a'x + b @f.
 */
class LinearCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearCost)

  /**
   * Construct a linear cost of the form @f a'x + b @f.
   * @param a Linear term.
   * @param b (optional) Constant term.
   */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  LinearCost(const Eigen::Ref<const Eigen::VectorXd>& a, double b = 0.)
      : Cost(a.rows()), a_(a), b_(b) {}

  ~LinearCost() override {}

  Eigen::SparseMatrix<double> GetSparseMatrix() const {
    // TODO(eric.cousineau): Consider storing or caching sparse matrix, such
    // that we can return a const lvalue reference.
    return a_.sparseView();
  }

  const Eigen::VectorXd& a() const { return a_; }

  double b() const { return b_; }

  /**
   * Updates the linear term, upper and lower bounds in the linear constraint.
   * The updated constraint is @f a_new' x + b_new @f.
   * Note that the number of variables (number of cols) cannot change.
   * @param new_a New linear term.
   * @param new_b (optional) New constant term.
   */
  void UpdateCoefficients(const Eigen::Ref<const Eigen::VectorXd>& new_a,
                          double new_b = 0.) {
    if (new_a.rows() != a_.rows()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    a_ = new_a;
    b_ = new_b;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  Eigen::VectorXd a_;
  double b_{};
};

/**
 * Implements a cost of the form @f .5 x'Qx + b'x + c @f.
 */
class QuadraticCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticCost)

  /**
   * Constructs a cost of the form @f .5 x'Qx + b'x + c @f.
   * @param Q Quadratic term.
   * @param b Linear term.
   * @param c (optional) Constant term.
   */
  template <typename DerivedQ, typename Derivedb>
  QuadraticCost(const Eigen::MatrixBase<DerivedQ>& Q,
                const Eigen::MatrixBase<Derivedb>& b, double c = 0.)
      : Cost(Q.rows()), Q_(Q), b_(b), c_(c) {
    DRAKE_ASSERT(Q_.rows() == Q_.cols());
    DRAKE_ASSERT(Q_.cols() == b_.rows());
  }

  ~QuadraticCost() override {}

  const Eigen::MatrixXd& Q() const { return Q_; }

  const Eigen::VectorXd& b() const { return b_; }

  double c() const { return c_; }

  /**
   * Updates the quadratic and linear term of the constraint. The new
   * matrices need to have the same dimension as before.
   * @param new_Q New quadratic term.
   * @param new_b New linear term.
   * @param new_c (optional) New constant term.
   */
  template <typename DerivedQ, typename DerivedB>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedQ>& new_Q,
                          const Eigen::MatrixBase<DerivedB>& new_b,
                          double new_c = 0.) {
    if (new_Q.rows() != new_Q.cols() || new_Q.rows() != new_b.rows() ||
        new_b.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions");
    }

    if (new_b.rows() != b_.rows()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    Q_ = new_Q;
    b_ = new_b;
    c_ = new_c;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
  double c_{};
};

/**
 * Creates a cost term of the form (x-x_desired)'*Q*(x-x_desired).
 */
std::shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired);

/**
 * Creates a cost term of the form | Ax - b |^2.
 */
std::shared_ptr<QuadraticCost> MakeL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/**
 * Implements a cost of the form P(x, y...) where P is a multivariate
 * polynomial in x, y, ...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the MathematicalProgram::Binding (the individual scalar
 * elements of the given VariableList).
 */
class PolynomialCost : public CostShim<PolynomialConstraint> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialCost)

  PolynomialCost(const VectorXPoly& polynomials,
                 const std::vector<Polynomiald::VarType>& poly_vars)
      : CostShim(
            polynomials, poly_vars,
            Vector1<double>::Constant(-std::numeric_limits<double>::infinity()),
            Vector1<double>::Constant(
                std::numeric_limits<double>::infinity())) {}

  const VectorXPoly& polynomials() const { return constraint()->polynomials(); }

  const std::vector<Polynomiald::VarType>& poly_vars() const {
    return constraint()->poly_vars();
  }
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

/**
 * Converts an input of type @p F to a FunctionCost object.
 * @tparam F This class should have functions numInputs(), numOutputs and
 * eval(x, y).
 * @see detail::FunctionTraits
 */
template <typename F>
std::shared_ptr<Cost> MakeFunctionCost(F&& f) {
  using FC = FunctionCost<std::decay_t<F>>;
  return std::make_shared<FC>(std::forward<F>(f));
}

}  // namespace solvers
}  // namespace drake
