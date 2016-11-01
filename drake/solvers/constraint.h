#pragma once

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"

namespace drake {
namespace solvers {

/**
 * A constraint is a function + lower and upper bounds.
 *
 * Solver interfaces must acknowledge that these constraints are mutable.
 * Parameters can change after the constraint is constructed and before the
 * call to Solve().
 *
 * Some thoughts:
 * It should support evaluating the constraint, adding it to an optimization
 * problem, and have support for constraints that require slack variables
 * (adding additional decision variables to the problem).  There
 * should also be some notion of parameterized constraints:  e.g. the
 * acceleration constraints in the rigid body dynamics are constraints
 * on vdot and f, but are "parameterized" by q and v.
 */
class Constraint {
  void check(size_t num_constraints) {
    static_cast<void>(num_constraints);
    DRAKE_ASSERT(static_cast<size_t>(lower_bound_.size()) == num_constraints &&
                 "Size of lower bound must match number of constraints.");
    DRAKE_ASSERT(static_cast<size_t>(upper_bound_.size()) == num_constraints &&
                 "Size of upper bound must match number of constraints.");
  }

 public:
  explicit Constraint(size_t num_constraints)
      : lower_bound_(num_constraints), upper_bound_(num_constraints) {
    check(num_constraints);
    lower_bound_.setConstant(-std::numeric_limits<double>::infinity());
    upper_bound_.setConstant(std::numeric_limits<double>::infinity());
  }

  template <typename DerivedLB, typename DerivedUB>
  Constraint(size_t num_constraints, Eigen::MatrixBase<DerivedLB> const& lb,
             Eigen::MatrixBase<DerivedUB> const& ub)
      : lower_bound_(lb), upper_bound_(ub) {
    check(num_constraints);
  }

  virtual ~Constraint() {}

  // TODO(bradking): consider using a Ref for `y`.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  virtual void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                    Eigen::VectorXd& y) const = 0;
  // Move this to DifferentiableConstraint derived class if/when we
  // need to support non-differentiable functions (at least, if
  // DifferentiableConstraint is ever implemented).
  virtual void Eval(const Eigen::Ref<const TaylorVecXd>& x,
                    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                    TaylorVecXd& y) const = 0;

  Eigen::VectorXd const& lower_bound() const { return lower_bound_; }
  Eigen::VectorXd const& upper_bound() const { return upper_bound_; }
  size_t num_constraints() const { return lower_bound_.size(); }

  template <typename Derived>
  void UpdateLowerBound(const Eigen::MatrixBase<Derived>& new_lb) {
    set_bounds(new_lb, upper_bound_);
  }

  template <typename Derived>
  void UpdateUpperBound(const Eigen::MatrixBase<Derived>& new_ub) {
    set_bounds(lower_bound_, new_ub);
  }

  inline void set_description(const std::string& description) {
    description_ = description;
  }
  inline const std::string& get_description() const { return description_; }

  /**
   * Set the upper and lower bounds of the constraint.
   * @param lower_bound. A num_constraint() x 1 vector.
   * @param upper_bound. A num_constraint() x 1 vector.
   */
  template <typename DerivedL, typename DerivedU>
  void set_bounds(const Eigen::MatrixBase<DerivedL>& lower_bound,
                  const Eigen::MatrixBase<DerivedU>& upper_bound) {
    if (lower_bound.rows() != upper_bound.rows() || lower_bound.cols() != 1 ||
        upper_bound.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions.");
    }

    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

 private:
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  std::string description_;
};

/**
 * lb <= .5 x'Qx + b'x <= ub
 */
class QuadraticConstraint : public Constraint {
 public:
  static const int kNumConstraints = 1;

  template <typename DerivedQ, typename Derivedb>
  QuadraticConstraint(const Eigen::MatrixBase<DerivedQ>& Q,
                      const Eigen::MatrixBase<Derivedb>& b, double lb,
                      double ub)
      : Constraint(kNumConstraints, drake::Vector1d::Constant(lb),
                   drake::Vector1d::Constant(ub)),
        Q_(Q),
        b_(b) {
    DRAKE_ASSERT(Q_.rows() == Q_.cols());
    DRAKE_ASSERT(Q_.cols() == b_.rows());
  }

  ~QuadraticConstraint() override {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  }
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = .5 * x.transpose() * Q_.cast<TaylorVarXd>() * x +
        b_.cast<TaylorVarXd>().transpose() * x;
  };

  virtual const Eigen::MatrixXd& Q() const { return Q_; }

  virtual const Eigen::VectorXd& b() const { return b_; }

  /**
   * Updates the quadratic and linear term of the constraint. The new
   * matrices need to have the same dimension as before.
   * @param new_Q new quadratic term
   * @param new_b new linear term
   */
  template <typename DerivedQ, typename DerivedB>
  void UpdateQuadraticAndLinearTerms(const Eigen::MatrixBase<DerivedQ>& new_Q,
                                     const Eigen::MatrixBase<DerivedB>& new_b) {
    if (new_Q.rows() != new_Q.cols() || new_Q.rows() != new_b.rows() ||
        new_b.cols() != 1) {
      throw std::runtime_error("New constraints have invalid dimensions");
    }

    if (new_b.rows() != b_.rows()) {
      throw std::runtime_error("Can't change the number of decision variables");
    }

    Q_ = new_Q;
    b_ = new_b;
  }

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
};

/**
 A LorentzConeConstraint that takes a n x 1 vector x, and imposes constraint
 \f[
 x_1 >= \sqrt{x_2^2+...+x_n^2}
 \f]
 Ideally this constraint should be handled by a second-order cone solver.
 In case the user wants to enforce this constraint through general nonlinear
 optimization, with smooth gradient, we alternatively impose the following
 constraint, with smooth gradient everywhere
 \f[
 x_1 >= 0 \\
 x_1^2-x_2^2-...-x_n^2 >= 0
 \f]
 For more information and visualization, please refer to
 https://inst.eecs.berkeley.edu/~ee127a/book/login/l_socp_soc.html
 */
class LorentzConeConstraint : public Constraint {
 public:
  LorentzConeConstraint()
      : Constraint(2, Eigen::Vector2d::Constant(0.0),
                   Eigen::Vector2d::Constant(
                       std::numeric_limits<double>::infinity())) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y(0) = x(0);
    y(1) = pow(x(0), 2) - x.tail(x.size() - 1).squaredNorm();
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y(0) = x(0);
    y(1) = pow(x(0), 2) - x.tail(x.size() - 1).squaredNorm();
  }
};

/**
 * A rotated Lorentz cone constraint that taks a n x 1 vector x, and imposes
 * constraint
 * \f[
 * x_1 >= 0
 * x_2 >= 0
 * x_1 * x_2 >= x_3^2 + x_4^2 + ... + x_n^2
 * \f]
 * For more information and visualization, please refer to
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/l_socp_soc.html
 */
class RotatedLorentzConeConstraint : public Constraint {
 public:
  RotatedLorentzConeConstraint()
      : Constraint(3, Eigen::Vector3d::Constant(0.0),
                   Eigen::Vector3d::Constant(
                       std::numeric_limits<double>::infinity())) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y(0) = x(0);
    y(1) = x(1);
    y(2) = x(0) * x(1) - x.tail(x.size() - 2).squaredNorm();
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y(0) = x(0);
    y(1) = x(1);
    y(2) = x(0) * x(1) - x.tail(x.size() - 2).squaredNorm();
  }
};
/** A semidefinite constraint  that takes a symmetric matrix as
 well as a linear component.
 <pre>
 lb <= b'*x + Trace(G'*X) <= ub
 </pre>
 */
class SemidefiniteConstraint : public Constraint {
 public:
  static const int kNumConstraints = 1;
  // TODO(naveenoid) : ASSERT check on dimensions of G and b.
  // TODO(alexdunyak) : Implement Eval().
  template <typename DerivedQ, typename Derivedb>
  SemidefiniteConstraint(const Eigen::MatrixBase<DerivedQ>& G,
                         const Eigen::MatrixBase<Derivedb>& b, double lb,
                         double ub)
      : Constraint(kNumConstraints, drake::Vector1d::Constant(lb),
                   drake::Vector1d::Constant(ub)),
        G_(G),
        b_(b) {}

  ~SemidefiniteConstraint() override {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    throw std::runtime_error(
        "Eval is not implemented in SemidefiniteConstraint.");
  };
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    throw std::runtime_error(
        "Eval is not implemented in SemidefiniteConstraint.");
  };

  virtual const Eigen::MatrixXd& G() const { return G_; }

  virtual const Eigen::VectorXd& b() const { return b_; }

 private:
  Eigen::MatrixXd G_;
  Eigen::VectorXd b_;
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
class PolynomialConstraint : public Constraint {
 public:
  PolynomialConstraint(const VectorXPoly& polynomials,
                       const std::vector<Polynomiald::VarType>& poly_vars,
                       const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
      : Constraint(polynomials.rows(), lb, ub),
        polynomials_(polynomials),
        poly_vars_(poly_vars) {}

  ~PolynomialConstraint() override {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    double_evaluation_point_.clear();
    for (size_t i = 0; i < poly_vars_.size(); i++) {
      double_evaluation_point_[poly_vars_[i]] = x[i];
    }
    y.resize(num_constraints());
    for (size_t i = 0; i < num_constraints(); i++) {
      y[i] = polynomials_[i].EvaluateMultivariate(double_evaluation_point_);
    }
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    taylor_evaluation_point_.clear();
    for (size_t i = 0; i < poly_vars_.size(); i++) {
      taylor_evaluation_point_[poly_vars_[i]] = x[i];
    }
    y.resize(num_constraints());
    for (size_t i = 0; i < num_constraints(); i++) {
      y[i] = polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_);
    }
  }

 private:
  const VectorXPoly polynomials_;
  const std::vector<Polynomiald::VarType> poly_vars_;

  /// To avoid repeated allocation, reuse a map for the evaluation point.
  mutable std::map<Polynomiald::VarType, double> double_evaluation_point_;
  mutable std::map<Polynomiald::VarType, TaylorVarXd> taylor_evaluation_point_;
};

// todo: consider implementing DifferentiableConstraint,
// TwiceDifferentiableConstraint, ComplementarityConstraint,
// IntegerConstraint, ...
/**
 * Implements a constraint of the form @f lb <= Ax <= ub @f
 */
class LinearConstraint : public Constraint {
 public:
  explicit LinearConstraint(size_t num_constraints)
      : Constraint(num_constraints) {}
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  LinearConstraint(const Eigen::MatrixBase<DerivedA>& a,
                   const Eigen::MatrixBase<DerivedLB>& lb,
                   const Eigen::MatrixBase<DerivedUB>& ub)
      : Constraint(a.rows(), lb, ub), A_(a) {
    DRAKE_ASSERT(a.rows() == lb.rows());
  }

  ~LinearConstraint() override {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = A_ * x;
  }
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = A_.cast<TaylorVarXd>() * x;
  };

  virtual Eigen::SparseMatrix<double> GetSparseMatrix() const {
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
  void UpdateConstraint(const Eigen::MatrixBase<DerivedA>& new_A,
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
    set_bounds(new_lb, new_ub);
  }

 protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;
};

/**
 * Implements a constraint of the form @f Ax = b @f
 */
class LinearEqualityConstraint : public LinearConstraint {
 public:
  template <typename DerivedA, typename DerivedB>
  LinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                           const Eigen::MatrixBase<DerivedB>& beq)
      : LinearConstraint(Aeq, beq, beq) {}

  ~LinearEqualityConstraint() override {}

  /*
   * @brief change the parameters of the constraint (A and b), but not the
   *variable associations
   *
   * note that A and b can change size in the rows only (representing a
   *different number of linear constraints, but on the same decision variables)
   */
  template <typename DerivedA, typename DerivedB>
  void UpdateConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                        const Eigen::MatrixBase<DerivedB>& beq) {
    LinearConstraint::UpdateConstraint(Aeq, beq, beq);
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
  template <typename DerivedLB, typename DerivedUB>
  BoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb,
                        const Eigen::MatrixBase<DerivedUB>& ub)
      : LinearConstraint(Eigen::MatrixXd::Identity(lb.rows(), lb.rows()), lb,
                         ub) {}

  ~BoundingBoxConstraint() override {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = x;
  }
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = x;
  }
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
  template <typename DerivedM, typename Derivedq>
  LinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                  const Eigen::MatrixBase<Derivedq>& q)
      : Constraint(q.rows()), M_(M), q_(q) {}

  ~LinearComplementarityConstraint() override {}

  /** Return Mx + q (the value of the slack variable). */
  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = (M_ * x) + q_;
  }
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = (M_.cast<TaylorVarXd>() * x) + q_.cast<TaylorVarXd>();
  };

  const Eigen::MatrixXd& M() const { return M_; }
  const Eigen::VectorXd& q() const { return q_; }

 private:
  // TODO(ggould-tri) We are storing what are likely statically sized matrices
  // in dynamically allocated containers.  This probably isn't optimal.
  Eigen::MatrixXd M_;
  Eigen::VectorXd q_;
};

}  // namespace solvers
}  // namespace drake
