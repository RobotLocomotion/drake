#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace solvers {

/**
 * Provides an abstract base for all costs.
 *
 * @ingroup solver_evaluators
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
 * Implements a cost of the form @f[ a'x + b @f].
 *
 * @ingroup solver_evaluators
 */
class LinearCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearCost)

  /**
   * Construct a linear cost of the form @f[ a'x + b @f].
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
   * Updates the coefficients of the cost.
   * Note that the number of variables (size of a) cannot change.
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
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

 private:
  template <typename DerivedX, typename U>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x, VectorX<U>* y) const;

  Eigen::VectorXd a_;
  double b_{};
};

/**
 * Implements a cost of the form @f[ .5 x'Qx + b'x + c @f].
 *
 * @ingroup solver_evaluators
 */
class QuadraticCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticCost)

  /**
   * Constructs a cost of the form @f[ .5 x'Qx + b'x + c @f].
   * @param Q Quadratic term.
   * @param b Linear term.
   * @param c (optional) Constant term.
   * @param is_hessian_psd (optional) Indicates if the Hessian matrix Q is
   * positive semidefinite (psd) or not. If set to true, then the user
   * guarantees that Q is psd; if set to false, then the user guarantees that Q
   * is not psd. If set to std::nullopt, then the constructor will check if Q is
   * psd or not. The default is std::nullopt. To speed up the constructor, set
   * is_hessian_psd to either true or false.
   */
  template <typename DerivedQ, typename Derivedb>
  QuadraticCost(const Eigen::MatrixBase<DerivedQ>& Q,
                const Eigen::MatrixBase<Derivedb>& b, double c = 0.,
                std::optional<bool> is_hessian_psd = std::nullopt)
      : Cost(Q.rows()), Q_((Q + Q.transpose()) / 2), b_(b), c_(c) {
    DRAKE_ASSERT(Q_.rows() == Q_.cols());
    DRAKE_ASSERT(Q_.cols() == b_.rows());
    if (is_hessian_psd.has_value()) {
      is_convex_ = is_hessian_psd.value();
    } else {
      is_convex_ = CheckHessianPsd();
    }
  }

  ~QuadraticCost() override {}

  /// Returns the symmetric matrix Q, as the Hessian of the cost.
  const Eigen::MatrixXd& Q() const { return Q_; }

  const Eigen::VectorXd& b() const { return b_; }

  double c() const { return c_; }

  /**
   * Returns true if this cost is convex. A quadratic cost if convex if and only
   * if its Hessian matrix Q is positive semidefinite.
   */
  bool is_convex() const { return is_convex_; }

  /**
   * Updates the quadratic and linear term of the constraint. The new
   * matrices need to have the same dimension as before.
   * @param new_Q New quadratic term.
   * @param new_b New linear term.
   * @param new_c (optional) New constant term.
   * @param is_hessian_psd (optional) Indicates if the Hessian matrix Q is
   * positive semidefinite (psd) or not. If set to true, then the user
   * guarantees that Q is psd; if set to false, then the user guarantees that Q
   * is not psd. If set to std::nullopt, then this function will check if Q is
   * psd or not. The default is std::nullopt. To speed up the computation, set
   * is_hessian_psd to either true or false.
   */
  template <typename DerivedQ, typename DerivedB>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedQ>& new_Q,
                          const Eigen::MatrixBase<DerivedB>& new_b,
                          double new_c = 0.,
                          std::optional<bool> is_hessian_psd = std::nullopt) {
    if (new_Q.rows() != new_Q.cols() || new_Q.rows() != new_b.rows() ||
        new_b.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions");
    }

    if (new_b.rows() != b_.rows()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    Q_ = (new_Q + new_Q.transpose()) / 2;
    b_ = new_b;
    c_ = new_c;
    if (is_hessian_psd.has_value()) {
      is_convex_ = is_hessian_psd.value();
    } else {
      is_convex_ = CheckHessianPsd();
    }
  }

 private:
  template <typename DerivedX, typename U>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x, VectorX<U>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

  bool CheckHessianPsd();

  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
  double c_{};
  bool is_convex_{};
};

/**
 * Creates a cost term of the form (x-x_desired)'*Q*(x-x_desired).
 *
 * @ingroup solver_evaluators
 */
std::shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired);

/**
 * Creates a quadratic cost of the form |Ax-b|²=(Ax-b)ᵀ(Ax-b)
 *
 * @ingroup solver_evaluators
 */
std::shared_ptr<QuadraticCost> Make2NormSquaredCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/**
 * Implements a cost of the form @f[ |Ax + b|₂ @f].
 *
 * @ingroup solver_evaluators
 */
class L2NormCost : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(L2NormCost)

  // TODO(russt): Add an option to select an implementation that smooths the
  // gradient discontinuity at the origin.
  /**
   * Construct a cost of the form @f[ |Ax + b|₂ @f].
   * @param A Linear term.
   * @param b Constant term.
   */
  L2NormCost(const Eigen::Ref<const Eigen::MatrixXd>& A,
             const Eigen::Ref<const Eigen::VectorXd>& b);

  ~L2NormCost() override {}

  const Eigen::MatrixXd& A() const { return A_; }

  const Eigen::VectorXd& b() const { return b_; }

  /**
   * Updates the coefficients of the cost.
   * Note that the number of variables (columns of A) cannot change.
   * @param new_A New linear term.
   * @param new_b New constant term.
   */
  void UpdateCoefficients(const Eigen::Ref<const Eigen::MatrixXd>& new_A,
                          const Eigen::Ref<const Eigen::VectorXd>& new_b);

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

 private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};


/**
 * A cost that may be specified using another (potentially nonlinear)
 * evaluator.
 * @tparam EvaluatorType The nested evaluator.
 *
 * @ingroup solver_evaluators
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
              Eigen::VectorXd* y) const override {
    evaluator_->Eval(x, y);
  }
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    evaluator_->Eval(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
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
 *
 * @ingroup solver_evaluators
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
 *
 * @ingroup solver_evaluators
 */
template <typename FF>
std::shared_ptr<Cost> MakeFunctionCost(FF&& f) {
  return std::make_shared<EvaluatorCost<>>(
      MakeFunctionEvaluator(std::forward<FF>(f)));
}

}  // namespace solvers
}  // namespace drake
