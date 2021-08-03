#pragma once

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/evaluator_base.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {

// TODO(eric.cousineau): Consider enabling the constraint class directly to
// specify new slack variables.
// TODO(eric.cousineau): Consider parameterized constraints:  e.g. the
// acceleration constraints in the rigid body dynamics are constraints
// on vdot and f, but are "parameterized" by q and v.
/**
 * A constraint is a function + lower and upper bounds.
 *
 * Solver interfaces must acknowledge that these constraints are mutable.
 * Parameters can change after the constraint is constructed and before the
 * call to Solve().
 *
 * It should support evaluating the constraint, and adding it to an optimization
 * problem.
 *
 * @ingroup solver_evaluators
 */
class Constraint : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Constraint)

  /**
   * Constructs a constraint which has `num_constraints` rows, with an input
   * `num_vars` x 1 vector.
   * @param num_constraints. The number of rows in the constraint output.
   * @param num_vars. The number of rows in the input.
   * If the input dimension is unknown, then set `num_vars` to Eigen::Dynamic.
   * @param lb Lower bound, which must be a `num_constraints` x 1 vector.
   * @param ub Upper bound, which must be a `num_constraints` x 1 vector.
   * @see Eval(...)
   */
  template <typename DerivedLB, typename DerivedUB>
  Constraint(int num_constraints, int num_vars,
             const Eigen::MatrixBase<DerivedLB>& lb,
             const Eigen::MatrixBase<DerivedUB>& ub,
             const std::string& description = "")
      : EvaluatorBase(num_constraints, num_vars, description),
        lower_bound_(lb),
        upper_bound_(ub) {
    check(num_constraints);
    DRAKE_DEMAND(!lower_bound_.array().isNaN().any());
    DRAKE_DEMAND(!upper_bound_.array().isNaN().any());
  }

  /**
   * Constructs a constraint which has `num_constraints` rows, with an input
   * `num_vars` x 1 vector, with no bounds.
   * @param num_constraints. The number of rows in the constraint output.
   * @param num_vars. The number of rows in the input.
   * If the input dimension is unknown, then set `num_vars` to Eigen::Dynamic.
   * @see Eval(...)
   */
  Constraint(int num_constraints, int num_vars)
      : Constraint(
            num_constraints, num_vars,
            Eigen::VectorXd::Constant(num_constraints,
                                      -std::numeric_limits<double>::infinity()),
            Eigen::VectorXd::Constant(
                num_constraints, std::numeric_limits<double>::infinity())) {}

  /**
   * Return whether this constraint is satisfied by the given value, `x`.
   * @param x A `num_vars` x 1 vector.
   * @param tol A tolerance for bound checking.
   */
  bool CheckSatisfied(const Eigen::Ref<const Eigen::VectorXd>& x,
                      double tol = 1E-6) const {
    DRAKE_ASSERT(x.rows() == num_vars() || num_vars() == Eigen::Dynamic);
    return DoCheckSatisfied(x, tol);
  }

  bool CheckSatisfied(const Eigen::Ref<const AutoDiffVecXd>& x,
                      double tol = 1E-6) const {
    DRAKE_ASSERT(x.rows() == num_vars() || num_vars() == Eigen::Dynamic);
    return DoCheckSatisfied(x, tol);
  }

  symbolic::Formula CheckSatisfied(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x) const {
    DRAKE_ASSERT(x.rows() == num_vars() || num_vars() == Eigen::Dynamic);
    return DoCheckSatisfied(x);
  }

  const Eigen::VectorXd& lower_bound() const { return lower_bound_; }
  const Eigen::VectorXd& upper_bound() const { return upper_bound_; }

  /** Number of rows in the output constraint. */
  int num_constraints() const { return num_outputs(); }


 protected:
  /** Updates the lower bound.
   * @note if the users want to expose this method in a sub-class, do
   * using Constraint::UpdateLowerBound, as in LinearConstraint.
   */
  void UpdateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& new_lb) {
    if (new_lb.rows() != num_constraints()) {
      throw std::logic_error("Lower bound has invalid dimension.");
    }
    lower_bound_ = new_lb;
  }

  /** Updates the upper bound.
   * @note if the users want to expose this method in a sub-class, do
   * using Constraint::UpdateUpperBound, as in LinearConstraint.
   */
  void UpdateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& new_ub) {
    if (new_ub.rows() != num_constraints()) {
      throw std::logic_error("Upper bound has invalid dimension.");
    }
    upper_bound_ = new_ub;
  }

  /**
   * Set the upper and lower bounds of the constraint.
   * @param lower_bound. A `num_constraints` x 1 vector.
   * @param upper_bound. A `num_constraints` x 1 vector.
   * @note If the users want to expose this method in a sub-class, do
   * using Constraint::set_bounds, as in LinearConstraint.
   */
  void set_bounds(const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                  const Eigen::Ref<const Eigen::VectorXd>& upper_bound) {
    UpdateLowerBound(lower_bound);
    UpdateUpperBound(upper_bound);
  }

  virtual bool DoCheckSatisfied(const Eigen::Ref<const Eigen::VectorXd>& x,
                                const double tol) const {
    Eigen::VectorXd y(num_constraints());
    DoEval(x, &y);
    return (y.array() >= lower_bound_.array() - tol).all() &&
           (y.array() <= upper_bound_.array() + tol).all();
  }

  virtual bool DoCheckSatisfied(const Eigen::Ref<const AutoDiffVecXd>& x,
                                const double tol) const {
    AutoDiffVecXd y(num_constraints());
    DoEval(x, &y);
    auto get_value = [](const AutoDiffXd& v) { return v.value(); };
    return
        (y.array().unaryExpr(get_value) >= lower_bound_.array() - tol).all() &&
        (y.array().unaryExpr(get_value) <= upper_bound_.array() + tol).all();
  }

  virtual symbolic::Formula DoCheckSatisfied(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x) const;

 private:
  void check(int num_constraints) const;

  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
};

/**
 * lb ≤ .5 xᵀQx + bᵀx ≤ ub
 * Without loss of generality, the class stores a symmetric matrix Q.
 * For a non-symmetric matrix Q₀, we can define Q = (Q₀ + Q₀ᵀ) / 2, since
 * xᵀQ₀x = xᵀQ₀ᵀx = xᵀ*(Q₀+Q₀ᵀ)/2 *x. The first equality holds because the
 * transpose of a scalar is the scalar itself. Hence we can always convert
 * a non-symmetric matrix Q₀ to a symmetric matrix Q.
 *
 * @ingroup solver_evaluators
 */
class QuadraticConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticConstraint)

  static const int kNumConstraints = 1;

  /**
   * Construct a quadratic constraint.
   * @tparam DerivedQ The type for Q.
   * @tparam Derivedb The type for b.
   * @param Q0 The square matrix. Notice that Q₀ does not have to be symmetric.
   * @param b The linear coefficient.
   * @param lb The lower bound.
   * @param ub The upper bound.
   */
  template <typename DerivedQ, typename Derivedb>
  QuadraticConstraint(const Eigen::MatrixBase<DerivedQ>& Q0,
                      const Eigen::MatrixBase<Derivedb>& b, double lb,
                      double ub)
      : Constraint(kNumConstraints, Q0.rows(), drake::Vector1d::Constant(lb),
                   drake::Vector1d::Constant(ub)),
        Q_((Q0 + Q0.transpose()) / 2),
        b_(b) {
    DRAKE_ASSERT(Q_.rows() == Q_.cols());
    DRAKE_ASSERT(Q_.cols() == b_.rows());
  }

  ~QuadraticConstraint() override {}

  /** The symmetric matrix Q, being the Hessian of this constraint.
   */
  virtual const Eigen::MatrixXd& Q() const { return Q_; }

  virtual const Eigen::VectorXd& b() const { return b_; }

  /**
   * Updates the quadratic and linear term of the constraint. The new
   * matrices need to have the same dimension as before.
   * @param new_Q new quadratic term
   * @param new_b new linear term
   */
  template <typename DerivedQ, typename DerivedB>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedQ>& new_Q,
                          const Eigen::MatrixBase<DerivedB>& new_b) {
    if (new_Q.rows() != new_Q.cols() || new_Q.rows() != new_b.rows() ||
        new_b.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions");
    }

    if (new_b.rows() != b_.rows()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    Q_ = (new_Q + new_Q.transpose()) / 2;
    b_ = new_b;
  }

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
};

/**
 Constraining the linear expression \f$ z=Ax+b \f$ lies within the Lorentz cone.
 A vector z ∈ ℝ ⁿ lies within Lorentz cone if
 @f[
 z_0 \ge \sqrt{z_1^2+...+z_{n-1}^2}
 @f]
 <!-->
 z₀ ≥ sqrt(z₁² + ... + zₙ₋₁²)
 <-->
 where A ∈ ℝ ⁿˣᵐ, b ∈ ℝ ⁿ are given matrices.
 Ideally this constraint should be handled by a second-order cone solver.
 In case the user wants to enforce this constraint through general nonlinear
 optimization, we provide three different formulations on the Lorentz cone
 constraint
 1. [kConvex] g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0
    This formulation is not differentiable at z₁=...=zₙ₋₁=0
 2. [kConvexSmooth] g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0
    but the gradient of g(z) is approximated as
    ∂g(z)/∂z = [1, -z₁/sqrt(z₁² + ... zₙ₋₁² + ε), ...,
 -zₙ₋₁/sqrt(z₁²+...+zₙ₋₁²+ε)] where ε is a small positive number.
 3. [kNonconvex] z₀²-(z₁²+...+zₙ₋₁²) ≥ 0
    z₀ ≥ 0
    This constraint is differentiable everywhere, but z₀²-(z₁²+...+zₙ₋₁²) ≥ 0 is
 non-convex. For more information and visualization, please refer to
 https://www.epfl.ch/labs/disopt/wp-content/uploads/2018/09/7.pdf
 and https://docs.mosek.com/modeling-cookbook/cqo.html (Fig 3.1)

 @ingroup solver_evaluators
 */
class LorentzConeConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LorentzConeConstraint)

  /**
   * We provide three possible Eval functions to represent the Lorentz cone
   * constraint z₀ ≥ sqrt(z₁² + ... + zₙ₋₁²). For more explanation on the three
   * formulations, refer to LorentzConeConstraint documentation.
   */
  enum class EvalType {
    kConvex,  ///< The constraint is g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0.
              ///< Note this formulation is non-differentiable at z₁= ...=
              ///< zₙ₋₁=0
    kConvexSmooth,  ///< Same as kConvex, but with approximated gradient that
                    ///< exists everywhere..
    kNonconvex  ///< Nonconvex constraint z₀²-(z₁²+...+zₙ₋₁²) ≥ 0 and z₀ ≥ 0.
                ///< Note this formulation is differentiable, but at z₁= ...=
                ///< zₙ₋₁=0 the gradient is also 0, so a gradient-based
                ///< nonlinear solver can get stuck.
  };

  LorentzConeConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::VectorXd>& b,
                        EvalType eval_type = EvalType::kConvexSmooth);

  ~LorentzConeConstraint() override {}

  /** Getter for A. */
  const Eigen::SparseMatrix<double>& A() const { return A_; }

  /** Getter for dense version of A. */
  const Eigen::MatrixXd& A_dense() const { return A_dense_; }

  /** Getter for b. */
  const Eigen::VectorXd& b() const { return b_; }

  /** Getter for eval type. */
  EvalType eval_type() const { return eval_type_; }

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

  const Eigen::SparseMatrix<double> A_;
  // We need to store a dense matrix of A_, so that we can compute the gradient
  // using AutoDiffXd, and return the gradient as a dense matrix.
  const Eigen::MatrixXd A_dense_;
  const Eigen::VectorXd b_;
  const EvalType eval_type_;
};

/**
 * Constraining that the linear expression \f$ z=Ax+b \f$ lies within rotated
 * Lorentz cone.
 * A vector z ∈ ℝ ⁿ lies within rotated Lorentz cone, if
 * @f[
 * z_0 \ge 0\\
 * z_1 \ge 0\\
 * z_0  z_1 \ge z_2^2 + z_3^2 + ... + z_{n-1}^2
 * @f]
 * where A ∈ ℝ ⁿˣᵐ, b ∈ ℝ ⁿ are given matrices.
 * <!-->
 * z₀ ≥ 0
 * z₁ ≥ 0
 * z₀ * z₁ ≥ z₂² + z₃² + ... zₙ₋₁²
 * <-->
 * For more information and visualization, please refer to
 * https://docs.mosek.com/modeling-cookbook/cqo.html (Fig 3.1)
 *
 * @ingroup solver_evaluators
 */
class RotatedLorentzConeConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RotatedLorentzConeConstraint)

  RotatedLorentzConeConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                               const Eigen::Ref<const Eigen::VectorXd>& b)
      : Constraint(
            3, A.cols(), Eigen::Vector3d::Constant(0.0),
            Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())),
        A_(A.sparseView()),
        A_dense_(A),
        b_(b) {
    DRAKE_DEMAND(A_.rows() >= 3);
    DRAKE_ASSERT(A_.rows() == b_.rows());
  }

  /** Getter for A. */
  const Eigen::SparseMatrix<double>& A() const { return A_; }

  /** Getter for dense version of A. */
  const Eigen::MatrixXd& A_dense() const { return A_dense_; }

  /** Getter for b. */
  const Eigen::VectorXd& b() const { return b_; }

  ~RotatedLorentzConeConstraint() override {}

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

  const Eigen::SparseMatrix<double> A_;
  // We need to store a dense matrix of A_, so that we can compute the gradient
  // using AutoDiffXd, and return the gradient as a dense matrix.
  const Eigen::MatrixXd A_dense_;
  const Eigen::VectorXd b_;
};

/**
 * A constraint that may be specified using another (potentially nonlinear)
 * evaluator.
 * @tparam EvaluatorType The nested evaluator.
 *
 * @ingroup solver_evaluators
 */
template <typename EvaluatorType = EvaluatorBase>
class EvaluatorConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EvaluatorConstraint)

  /**
   * Constructs an evaluator constraint, given the EvaluatorType instance
   * (which will specify the number of constraints and variables), and will
   * forward the remaining arguments to the Constraint constructor.
   * @param evaluator EvaluatorType instance.
   * @param args Arguments to be forwarded to the constraint constructor.
   */
  template <typename... Args>
  EvaluatorConstraint(const std::shared_ptr<EvaluatorType>& evaluator,
                      Args&&... args)
      : Constraint(evaluator->num_outputs(), evaluator->num_vars(),
                   std::forward<Args>(args)...),
        evaluator_(evaluator) {}

  using Constraint::set_bounds;
  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;

 protected:
  /** Reference to the nested evaluator. */
  const EvaluatorType& evaluator() const { return *evaluator_; }

 private:
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

  std::shared_ptr<EvaluatorType> evaluator_;
};

/**
 * A constraint on the values of multivariate polynomials.
 *
 *  lb[i] <= P[i](x, y...) <= ub[i], where each P[i] is a multivariate
 *  polynomial in x, y...
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the MathematicalProgram::Binding (the individual scalar
 * elements of the given VariableList).
 *
 * @ingroup solver_evaluators
 */
class PolynomialConstraint : public EvaluatorConstraint<PolynomialEvaluator> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolynomialConstraint)

  /**
   * Constructs a polynomial constraint
   * @param polynomials Polynomial vector, a `num_constraints` x 1 vector.
   * @param poly_vars Polynomial variables, a `num_vars` x 1 vector.
   * @param lb Lower bounds, a `num_constraints` x 1 vector.
   * @param ub Upper bounds, a `num_constraints` x 1 vector.
   */
  PolynomialConstraint(const VectorXPoly& polynomials,
                       const std::vector<Polynomiald::VarType>& poly_vars,
                       const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
      : EvaluatorConstraint(
            std::make_shared<PolynomialEvaluator>(polynomials, poly_vars), lb,
            ub) {}

  ~PolynomialConstraint() override {}

  const VectorXPoly& polynomials() const { return evaluator().polynomials(); }

  const std::vector<Polynomiald::VarType>& poly_vars() const {
    return evaluator().poly_vars();
  }
};

// TODO(bradking): consider implementing DifferentiableConstraint,
// TwiceDifferentiableConstraint, ComplementarityConstraint,
// IntegerConstraint, ...

/**
 * Implements a constraint of the form @f$ lb <= Ax <= ub @f$
 *
 * @ingroup solver_evaluators
 */
class LinearConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearConstraint)

  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  LinearConstraint(const Eigen::MatrixBase<DerivedA>& a,
                   const Eigen::MatrixBase<DerivedLB>& lb,
                   const Eigen::MatrixBase<DerivedUB>& ub)
      : Constraint(a.rows(), a.cols(), lb, ub), A_(a) {
    DRAKE_DEMAND(a.rows() == lb.rows());
    DRAKE_DEMAND(A_.array().isFinite().all());
  }

  ~LinearConstraint() override {}

  virtual Eigen::SparseMatrix<double> GetSparseMatrix() const {
    // TODO(eric.cousineau): Consider storing or caching sparse matrix, such
    // that we can return a const lvalue reference.
    return A_.sparseView();
  }
  virtual const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& A()
      const {
    return A_;
  }

  /**
   * Updates the linear term, upper and lower bounds in the linear constraint.
   * The updated constraint is:
   * new_lb <= new_A * x <= new_ub
   * Note that the size of constraints (number of rows) can change, but the
   * number of variables (number of cols) cannot.
   * @param new_A new linear term
   * @param new_lb new lower bound
   * @param new_up new upper bound
   */
  template <typename DerivedA, typename DerivedL, typename DerivedU>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedA>& new_A,
                          const Eigen::MatrixBase<DerivedL>& new_lb,
                          const Eigen::MatrixBase<DerivedU>& new_ub) {
    if (new_A.rows() != new_lb.rows() || new_lb.rows() != new_ub.rows() ||
        new_lb.cols() != 1 || new_ub.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions");
    }

    if (new_A.cols() != A_.cols()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    A_ = new_A;
    set_num_outputs(A_.rows());
    set_bounds(new_lb, new_ub);
  }

  using Constraint::set_bounds;
  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;
};

/**
 * Implements a constraint of the form @f$ Ax = b @f$
 *
 * @ingroup solver_evaluators
 */
class LinearEqualityConstraint : public LinearConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearEqualityConstraint)

  /**
   * Constructs the linear equality constraint Aeq * x = beq
   */
  template <typename DerivedA, typename DerivedB>
  LinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                           const Eigen::MatrixBase<DerivedB>& beq)
      : LinearConstraint(Aeq, beq, beq) {}

  /**
   * Constructs the linear equality constraint a.dot(x) = beq
   */
  LinearEqualityConstraint(const Eigen::Ref<const Eigen::RowVectorXd>& a,
                           double beq)
      : LinearEqualityConstraint(a, Vector1d(beq)) {}

  ~LinearEqualityConstraint() override {}

  /*
   * @brief change the parameters of the constraint (A and b), but not the
   *variable associations
   *
   * note that A and b can change size in the rows only (representing a
   *different number of linear constraints, but on the same decision variables)
   */
  template <typename DerivedA, typename DerivedB>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedA>& Aeq,
                          const Eigen::MatrixBase<DerivedB>& beq) {
    LinearConstraint::UpdateCoefficients(Aeq, beq, beq);
  }

 private:
  /**
   * The user should not call this function. Call UpdateCoefficients(Aeq, beq)
   * instead.
   */
  template <typename DerivedA, typename DerivedL, typename DerivedU>
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedA>&,
                          const Eigen::MatrixBase<DerivedL>&,
                          const Eigen::MatrixBase<DerivedU>&) {
    static_assert(
        !std::is_same_v<DerivedA, DerivedA>,
        "This method should not be called form `LinearEqualityConstraint`");
  }

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;
};

/**
 * Implements a constraint of the form @f$ lb <= x <= ub @f$
 *
 * Note: the base Constraint class (as implemented at the moment) could
 * play this role.  But this class enforces that it is ONLY a bounding
 * box constraint, and not something more general.  Some solvers use
 * this information to handle bounding box constraints differently than
 * general constraints, so use of this form is encouraged.
 *
 * @ingroup solver_evaluators
 */
class BoundingBoxConstraint : public LinearConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BoundingBoxConstraint)

  template <typename DerivedLB, typename DerivedUB>
  BoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb,
                        const Eigen::MatrixBase<DerivedUB>& ub)
      : LinearConstraint(Eigen::MatrixXd::Identity(lb.rows(), lb.rows()), lb,
                         ub) {}

  ~BoundingBoxConstraint() override {}

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  std::ostream& DoDisplay(std::ostream&,
                          const VectorX<symbolic::Variable>&) const override;
};

/**
 * Implements a constraint of the form:
 *
 * <pre>
 *   Mx + q >= 0
 *   x >= 0
 *   x'(Mx + q) == 0
 * </pre>
 *
 * An implied slack variable complements any 0 component of x.  To get
 * the slack values at a given solution x, use Eval(x).
 *
 * @ingroup solver_evaluators
 */
class LinearComplementarityConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearComplementarityConstraint)

  template <typename DerivedM, typename Derivedq>
  LinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                  const Eigen::MatrixBase<Derivedq>& q)
      : Constraint(q.rows(), M.cols()), M_(M), q_(q) {}

  ~LinearComplementarityConstraint() override {}

  const Eigen::MatrixXd& M() const { return M_; }
  const Eigen::VectorXd& q() const { return q_; }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  bool DoCheckSatisfied(const Eigen::Ref<const Eigen::VectorXd>& x,
                        const double tol) const override;

  bool DoCheckSatisfied(const Eigen::Ref<const AutoDiffVecXd>& x,
                        const double tol) const override;

  symbolic::Formula DoCheckSatisfied(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x) const override;

 private:
  // Return Mx + q (the value of the slack variable).
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  // TODO(ggould-tri) We are storing what are likely statically sized matrices
  // in dynamically allocated containers.  This probably isn't optimal.
  Eigen::MatrixXd M_;
  Eigen::VectorXd q_;
};

/**
 * Implements a positive semidefinite constraint on a symmetric matrix S
 * @f[\text{
 *     S is p.s.d
 * }@f]
 * namely, all eigen values of S are non-negative.
 *
 * @ingroup solver_evaluators
 */
class PositiveSemidefiniteConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositiveSemidefiniteConstraint)

  /**
   * Impose the constraint that a symmetric matrix with size @p rows x @p rows
   * is positive semidefinite.
   * @see MathematicalProgram::AddPositiveSemidefiniteConstraint() for how
   * to use this constraint on some decision variables. We currently use this
   * constraint as a place holder in MathematicalProgram, to indicate the
   * positive semidefiniteness of some decision variables.
   * @param rows The number of rows (and columns) of the symmetric matrix.
   *
   * Example:
   * @code{.cc}
   * // Create a MathematicalProgram object.
   * auto prog = MathematicalProgram();
   *
   * // Add a 2 x 2 symmetric matrix S to optimization program as new decision
   * // variables.
   * auto S = prog.NewSymmetricContinuousVariables<2>("S");
   *
   * // Impose a positive semidefinite constraint on S.
   * std::shared_ptr<PositiveSemidefiniteConstraint> psd_constraint =
   *     prog.AddPositiveSemidefiniteConstraint(S);
   *
   * /////////////////////////////////////////////////////////////
   * // Add more constraints to make the program more interesting,
   * // but this is not needed.
   *
   * // Add the constraint that S(1, 0) = 1.
   * prog.AddBoundingBoxConstraint(1, 1, S(1, 0));
   *
   * // Minimize S(0, 0) + S(1, 1).
   * prog.AddLinearCost(Eigen::RowVector2d(1, 1), {S.diagonal()});
   *
   * /////////////////////////////////////////////////////////////
   *
   * // Now solve the program.
   * auto result = Solve(prog);
   *
   * // Retrieve the solution of matrix S.
   * auto S_value = GetSolution(S, result);
   *
   * // Compute the eigen values of the solution, to see if they are
   * // all non-negative.
   * Eigen::Vector4d S_stacked;
   * S_stacked << S_value.col(0), S_value.col(1);
   *
   * Eigen::VectorXd S_eigen_values;
   * psd_constraint->Eval(S_stacked, S_eigen_values);
   *
   * std::cout<<"S solution is: " << S << std::endl;
   * std::cout<<"The eigen value of S is " << S_eigen_values << std::endl;
   * @endcode
   */
  explicit PositiveSemidefiniteConstraint(int rows)
      : Constraint(rows, rows * rows, Eigen::VectorXd::Zero(rows),
                   Eigen::VectorXd::Constant(
                       rows, std::numeric_limits<double>::infinity())),
        matrix_rows_(rows) {}

  ~PositiveSemidefiniteConstraint() override {}

  int matrix_rows() const { return matrix_rows_; }

 protected:
  /**
   * Evaluate the eigen values of the symmetric matrix.
   * @param x The stacked columns of the symmetric matrix.
   */
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  /**
   * @param x The stacked columns of the symmetric matrix. This function is not
   * supported yet, since Eigen's eigen value solver does not accept
   * AutoDiffScalar.
   */
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  /**
   * @param x The stacked columns of the symmetric matrix. This function is not
   * supported, since Eigen's eigen value solver does not accept
   * symbolic::Expression.
   */
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

 private:
  int matrix_rows_;  // Number of rows in the symmetric matrix being positive
                     // semi-definite.
};

/**
 * Impose the matrix inequality constraint on variable x
 * <!-->
 * F₀ + x₁ * F₁ + ... xₙ * Fₙ is p.s.d
 * <-->
 * @f[
 * F_0 + x_1  F_1 + ... + x_n  F_n \text{ is p.s.d}
 * @f]
 * where p.s.d stands for positive semidefinite.
 * @f$ F_0, F_1, ..., F_n @f$ are all given symmetric matrices of the same size.
 *
 * @ingroup solver_evaluators
 */
class LinearMatrixInequalityConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearMatrixInequalityConstraint)

  /**
   * @param F Each symmetric matrix F[i] should be of the same size.
   * @param symmetry_tolerance  The precision to determine if the input matrices
   * Fi are all symmetric. @see math::IsSymmetric().
   */
  LinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      double symmetry_tolerance = 1E-10);

  ~LinearMatrixInequalityConstraint() override {}

  /* Getter for all given matrices F */
  const std::vector<Eigen::MatrixXd>& F() const { return F_; }

  /// Gets the number of rows in the matrix inequality constraint. Namely
  /// Fi are all matrix_rows() x matrix_rows() matrices.
  int matrix_rows() const { return matrix_rows_; }

 protected:
  /**
   * Evaluate the eigen values of the linear matrix.
   */
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  /**
   * This function is not supported, since Eigen's eigen value solver does not
   * accept AutoDiffScalar type.
   */
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  /**
   * This function is not supported, since Eigen's eigen value solver does not
   * accept symbolic::Expression type.
   */
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

 private:
  std::vector<Eigen::MatrixXd> F_;
  const int matrix_rows_{};
};

/**
 * Impose a generic (potentially nonlinear) constraint represented as a
 * vector of symbolic Expression.  Expression::Evaluate is called on every
 * constraint evaluation.
 *
 * Uses symbolic::Jacobian to provide the gradients to the AutoDiff method.
 *
 * @ingroup solver_evaluators
 */
class ExpressionConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExpressionConstraint)

  ExpressionConstraint(const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
                       const Eigen::Ref<const Eigen::VectorXd>& lb,
                       const Eigen::Ref<const Eigen::VectorXd>& ub);

  /**
   * @return the list of the variables involved in the vector of expressions,
   * in the order that they are expected to be received during DoEval.
   * Any Binding that connects this constraint to decision variables should
   * pass this list of variables to the Binding.
   */
  const VectorXDecisionVariable& vars() const { return vars_; }

  /** @return the symbolic expressions. */
  const VectorX<symbolic::Expression>& expressions() const {
    return expressions_;
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
  VectorX<symbolic::Expression> expressions_{0};
  MatrixX<symbolic::Expression> derivatives_{0, 0};

  // map_var_to_index_[vars_(i).get_id()] = i.
  VectorXDecisionVariable vars_{0};
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index_;

  // Only for caching, does not carrying hidden state.
  mutable symbolic::Environment environment_;
};

/**
 * An exponential cone constraint is a special type of convex cone constraint.
 * We constrain A * x + b to be in the exponential cone, where A has 3 rows, and
 * b is in ℝ³, x is the decision variable.
 * A vector z in ℝ³ is in the exponential cone, if
 * {z₀, z₁, z₂ | z₀ ≥ z₁ * exp(z₂ / z₁), z₁ > 0}.
 * Equivalently, this constraint can be refomulated with logarithm function
 * {z₀, z₁, z₂ | z₂ ≤ z₁ * log(z₀ / z₁), z₀ > 0, z₁ > 0}
 *
 * The Eval function implemented in this class is
 * z₀ - z₁ * exp(z₂ / z₁) >= 0,
 * z₁ > 0
 * where z = A * x + b.
 * It is not recommended to solve an exponential cone constraint through
 * generic nonlinear optimization. It is possible that the nonlinear solver
 * can accidentally set z₁ = 0, where the constraint is not well defined.
 * Instead, the user should consider to solve the program through conic solvers
 * that can exploit exponential cone, such as Mosek and SCS.
 *
 * @ingroup solver_evaluators
 */
class ExponentialConeConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExponentialConeConstraint)

  /**
   * Constructor for exponential cone.
   * Constrains A * x + b to be in the exponential cone.
   * @pre A has 3 rows.
   */
  ExponentialConeConstraint(
      const Eigen::Ref<const Eigen::SparseMatrix<double>>& A,
      const Eigen::Ref<const Eigen::Vector3d>& b);

  ~ExponentialConeConstraint() override{};

  /** Getter for matrix A. */
  const Eigen::SparseMatrix<double>& A() const { return A_; }

  /** Getter for vector b. */
  const Eigen::Vector3d& b() const { return b_; }

 protected:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

 private:
  Eigen::SparseMatrix<double> A_;
  Eigen::Vector3d b_;
};

}  // namespace solvers
}  // namespace drake
