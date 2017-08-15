#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace solvers {

/**
 * Provides an abstract base for all costs.
 */
class Cost : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Cost)

 protected:
  /**
   * Constructs a cost evaluator.
   * @param num_vars Number of input variables.
   * @param description Human-friendly description.
   */
  explicit Cost(int num_vars, const std::string& description = "")
      : EvaluatorBase(1, num_vars, description) {}
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

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

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
 * A cost that may be specified using another (potentially nonlinear)
 * evaluator.
 * @tparam EvaluatorType The nested evaluator.
 */
template <typename EvaluatorType = EvaluatorBase>
class EvaluatorCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EvaluatorCost)

  explicit EvaluatorCost(const std::shared_ptr<EvaluatorType>& evaluator)
      : Cost(evaluator->num_vars()), evaluator_(evaluator) {
    DRAKE_DEMAND(evaluator->num_outputs() == 1);
  }

 protected:
  const EvaluatorType& evaluator() const { return *evaluator_; }
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    evaluator_->Eval(x, y);
  }
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    evaluator_->Eval(x, y);
  }

 private:
  std::shared_ptr<EvaluatorType> evaluator_;
};

/**
 * Implements a cost of the form P(x, y...) where P is a multivariate
 * polynomial in x, y, ...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the Binding<> (the individual scalar elements of the
 * given VariableList).
 */
class PolynomialCost : public EvaluatorCost<PolynomialEvaluator> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialCost)

  /**
   * Constructs a polynomial cost
   * @param polynomials Polynomial vector, a 1 x 1 vector.
   * @param poly_vars Polynomial variables, a `num_vars` x 1 vector.
   */
  PolynomialCost(const VectorXPoly& polynomials,
                 const std::vector<Polynomiald::VarType>& poly_vars)
      : EvaluatorCost(
            std::make_shared<PolynomialEvaluator>(polynomials, poly_vars)) {}

  const VectorXPoly& polynomials() const { return evaluator().polynomials(); }

  const std::vector<Polynomiald::VarType>& poly_vars() const {
    return evaluator().poly_vars();
  }
};

/**
 * Converts an input of type @p F to a nonlinear cost.
 * @tparam FF The forwarded function type (e.g., `const F&, `F&&`, ...).
 * The class `F` should have functions numInputs(), numOutputs(), and
 * eval(x, y).
 * @see detail::FunctionTraits.
 */
template <typename FF>
std::shared_ptr<Cost> MakeFunctionCost(FF&& f) {
  return std::make_shared<EvaluatorCost<>>(
      MakeFunctionEvaluator(std::forward<FF>(f)));
}

}  // namespace solvers
}  // namespace drake
