#pragma once

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/solvers/evaluator_base.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {

/**
 * A constraint is a function + lower and upper bounds.
 *
 * Solver interfaces must acknowledge that these constraints are mutable.
 * Parameters can change after the constraint is constructed and before the
 * call to Solve().
 *
 * It should support evaluating the constraint, and adding it to an optimization
 * problem.
 */
// TODO(eric.cousineau): Consider enabling the constraint class directly to
// specify new slack variables.
// TODO(eric.cousineau): Consider parameterized constraints:  e.g. the
// acceleration constraints in the rigid body dynamics are constraints
// on vdot and f, but are "parameterized" by q and v.
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

  const Eigen::VectorXd& lower_bound() const { return lower_bound_; }
  const Eigen::VectorXd& upper_bound() const { return upper_bound_; }

  /** Number of rows in the output constraint. */
  int num_constraints() const { return num_outputs(); }

 protected:
  /** Updates the lower bound.
   * @note if the users want to expose this method in a sub-class, do
   * using Constraint::set_bounds, as in LinearConstraint.
   */
  template <typename Derived>
  void UpdateLowerBound(const Eigen::MatrixBase<Derived>& new_lb) {
    set_bounds(new_lb, upper_bound_);
  }

  /** Updates the upper bound.
   * @note if the users want to expose this method in a sub-class, do
   * using Constraint::set_bounds, as in LinearConstraint.
   */
  template <typename Derived>
  void UpdateUpperBound(const Eigen::MatrixBase<Derived>& new_ub) {
    set_bounds(lower_bound_, new_ub);
  }

  /**
   * Set the upper and lower bounds of the constraint.
   * @param lower_bound. A `num_constraints` x 1 vector.
   * @param upper_bound. A `num_constraints` x 1 vector.
   * @note If the users want to expose this method in a sub-class, do
   * using Constraint::set_bounds, as in LinearConstraint.
   */
  template <typename DerivedL, typename DerivedU>
  void set_bounds(const Eigen::MatrixBase<DerivedL>& lower_bound,
                  const Eigen::MatrixBase<DerivedU>& upper_bound) {
    if (lower_bound.rows() != num_constraints() ||
        upper_bound.rows() != num_constraints() || lower_bound.cols() != 1 ||
        upper_bound.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions.");
    }

    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  virtual bool DoCheckSatisfied(const Eigen::Ref<const Eigen::VectorXd>& x,
                                const double tol) const {
    Eigen::VectorXd y(num_constraints());
    DoEval(x, y);
    return (y.array() >= lower_bound_.array() - tol).all() &&
           (y.array() <= upper_bound_.array() + tol).all();
  }

  virtual bool DoCheckSatisfied(const Eigen::Ref<const AutoDiffVecXd>& x,
                                const double tol) const {
    AutoDiffVecXd y(num_constraints());
    DoEval(x, y);
    return (y.array() >= lower_bound_.cast<AutoDiffXd>().array() - tol).all() &&
           (y.array() <= upper_bound_.cast<AutoDiffXd>().array() + tol).all();
  }

 private:
  void check(int num_constraints) {
    static_cast<void>(num_constraints);
    DRAKE_ASSERT(lower_bound_.size() == num_constraints &&
                 "Size of lower bound must match number of constraints.");
    DRAKE_ASSERT(upper_bound_.size() == num_constraints &&
                 "Size of upper bound must match number of constraints.");
  }

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
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

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
 optimization, with smooth gradient, we alternatively impose the following
 constraint, with smooth gradient everywhere
 @f[
 a_0^Tx+b_0\ge 0\\
 (a_0^Tx+b_0)^2-(a_1^Tx+b_1)^2-...-(a_{n-1}^Tx+b_{n-1})^2 \ge 0
 @f]
 where @f$ a_i^T@f$ is the i'th row of matrix @f$ A@f$. @f$ b_i @f$ is the i'th
 entry of vector @f$ b @f$.

 For more information and visualization, please refer to
 https://inst.eecs.berkeley.edu/~ee127a/book/login/l_socp_soc.html
 */
class LorentzConeConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LorentzConeConstraint)

  LorentzConeConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::VectorXd>& b)
      : Constraint(
            2, A.cols(), Eigen::Vector2d::Constant(0.0),
            Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
        A_(A),
        b_(b) {
    DRAKE_DEMAND(A_.rows() >= 2);
    DRAKE_ASSERT(A_.rows() == b_.rows());
  }

  ~LorentzConeConstraint() override {}

  /** Getter for A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Getter for b. */
  const Eigen::VectorXd& b() const { return b_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

  const Eigen::MatrixXd A_;
  const Eigen::VectorXd b_;
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
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/l_socp_soc.html
 */
class RotatedLorentzConeConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RotatedLorentzConeConstraint)

  RotatedLorentzConeConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                               const Eigen::Ref<const Eigen::VectorXd>& b)
      : Constraint(
            3, A.cols(), Eigen::Vector3d::Constant(0.0),
            Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())),
        A_(A),
        b_(b) {
    DRAKE_DEMAND(A_.rows() >= 3);
    DRAKE_ASSERT(A_.rows() == b_.rows());
  }

  /** Getter for A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Getter for b. */
  const Eigen::VectorXd& b() const { return b_; }

  ~RotatedLorentzConeConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

  const Eigen::MatrixXd A_;
  const Eigen::VectorXd b_;
};

/**
 * A constraint that may be specified using another (potentially nonlinear)
 * evaluator.
 * @tparam EvaluatorType The nested evaluator.
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

  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;
  using Constraint::set_bounds;

 protected:
  /** Reference to the nested evaluator. */
  const EvaluatorType& evaluator() const { return *evaluator_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    evaluator_->Eval(x, y);
  }
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    evaluator_->Eval(x, y);
  }

  std::shared_ptr<EvaluatorType> evaluator_;
};

/**
 *  lb[i] <= P[i](x, y...) <= ub[i], where each P[i] is a multivariate
 *  polynomial in x, y...
 *
 * A constraint on the values of multivariate polynomials.
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the MathematicalProgram::Binding (the individual scalar
 * elements of the given VariableList).
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
 * Implements a constraint of the form @f lb <= Ax <= ub @f
 */
class LinearConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearConstraint)

  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  LinearConstraint(const Eigen::MatrixBase<DerivedA>& a,
                   const Eigen::MatrixBase<DerivedLB>& lb,
                   const Eigen::MatrixBase<DerivedUB>& ub)
      : Constraint(a.rows(), a.cols(), lb, ub), A_(a) {
    DRAKE_ASSERT(a.rows() == lb.rows());
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

  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;
  using Constraint::set_bounds;

 protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;
};

/**
 * Implements a constraint of the form @f Ax = b @f
 */
class LinearEqualityConstraint : public LinearConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearEqualityConstraint)

  template <typename DerivedA, typename DerivedB>
  LinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                           const Eigen::MatrixBase<DerivedB>& beq)
      : LinearConstraint(Aeq, beq, beq) {}

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
  void UpdateCoefficients(const Eigen::MatrixBase<DerivedA>& new_A,
                          const Eigen::MatrixBase<DerivedL>& new_lb,
                          const Eigen::MatrixBase<DerivedU>& new_ub) {
    static_assert(
        !std::is_same<DerivedA, DerivedA>::value,
        "This method should not be called form `LinearEqualityConstraint`");
  }
};

/**
 * Implements a constraint of the form @f lb <= x <= ub @f
 *
 * Note: the base Constraint class (as implemented at the moment) could
 * play this role.  But this class enforces that it is ONLY a bounding
 * box constraint, and not something more general.  Some solvers use
 * this information to handle bounding box constraints differently than
 * general constraints, so use of this form is encouraged.
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

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;
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
  /** Return Mx + q (the value of the slack variable). */
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

  bool DoCheckSatisfied(const Eigen::Ref<const Eigen::VectorXd>& x,
                        const double tol) const override;

  bool DoCheckSatisfied(const Eigen::Ref<const AutoDiffVecXd>& x,
                        const double tol) const override;

 private:
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
   * prog.Solve();
   *
   * // Retrieve the solution of matrix S.
   * auto S_value = GetSolution(S);
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
              Eigen::VectorXd& y) const override;

  /**
   * @param x The stacked columns of the symmetric matrix. This function is not
   * supported yet, since Eigen's eigen value solver does not accept
   * AutoDiffScalar.
   */
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

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
              Eigen::VectorXd& y) const override;

  /**
   * This function is not supported, since Eigen's eigen value solver does not
   * accept AutoDiffScalar type.
   */
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  std::vector<Eigen::MatrixXd> F_;
  const int matrix_rows_{};
};

}  // namespace solvers
}  // namespace drake
