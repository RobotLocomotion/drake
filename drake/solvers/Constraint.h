#ifndef DRAKE_CONSTRAINT_H
#define DRAKE_CONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace Drake {

/* Constraint
 * @brief A constraint is a function + lower and upper bounds.
 *
 * Some thoughts:
 * It should support evaluating the constraint, adding it to an optimization
 *problem,
 * and have support for constraints that require slack variables (adding
 *additional decision variables to the problem).  There
 * should also be some notion of parameterized constraints:  e.g. the
 *acceleration constraints in the rigid body dynamics are constraints
 * on vdot and f, but are "parameterized" by q and v.
 */
class Constraint {
  void check(size_t num_constraints) {
    static_cast<void>(num_constraints);
    assert(lower_bound.size() == num_constraints &&
           "Size of lower bound must match number of constraints.");
    assert(upper_bound.size() == num_constraints &&
           "Size of upper bound must match number of constraints.");
  }

 public:
  Constraint(size_t num_constraints)
      : lower_bound(num_constraints), upper_bound(num_constraints) {
    check(num_constraints);
    lower_bound.setConstant(-std::numeric_limits<double>::infinity());
    upper_bound.setConstant(std::numeric_limits<double>::infinity());
  }

  template <typename DerivedLB, typename DerivedUB>
  Constraint(size_t num_constraints, Eigen::MatrixBase<DerivedLB> const& lb,
             Eigen::MatrixBase<DerivedUB> const& ub)
      : lower_bound(lb), upper_bound(ub) {
    check(num_constraints);
  }
  virtual ~Constraint() {}

  // TODO: consider using a Ref for `y` too.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const = 0;
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const = 0;  // move this to
                                                // DifferentiableConstraint
                                                // derived class if/when we need
                                                // to support non-differentiable
                                                // functions

  Eigen::VectorXd const& getLowerBound() const { return lower_bound; }
  Eigen::VectorXd const& getUpperBound() const { return upper_bound; }
  size_t getNumConstraints() const { return lower_bound.size(); }

 protected:
  Eigen::VectorXd lower_bound, upper_bound;
};

/** QuadraticConstraint
 * @brief  lb <= .5 x'Qx + b'x <= ub
 */
class QuadraticConstraint : public Constraint {
 public:
  template <typename DerivedQ, typename Derivedb>
  QuadraticConstraint(const Eigen::MatrixBase<DerivedQ>& Q,
                      const Eigen::MatrixBase<Derivedb>& b, double lb,
                      double ub)
      : Constraint(1, Vector1d::Constant(lb), Vector1d::Constant(ub)),
        Q(Q),
        b(b) {}
  virtual ~QuadraticConstraint() {}

  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(getNumConstraints());
    y = .5 * x.transpose() * Q * x + b.transpose() * x;
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(getNumConstraints());
    y = .5 * x.transpose() * Q.cast<TaylorVarXd>() * x +
        b.cast<TaylorVarXd>().transpose() * x;
  };

 private:
  Eigen::MatrixXd Q;
  Eigen::VectorXd b;
};

// todo: consider implementing DifferentiableConstraint,
// TwiceDifferentiableConstraint, PolynomialConstraint, QuadraticConstraint,
// ComplementarityConstraint, IntegerConstraint, ...
/** LinearConstraint
 * @brief Implements a constraint of the form @f lb <= Ax <= ub @f
 */
class LinearConstraint : public Constraint {
 public:
  LinearConstraint(size_t num_constraints) : Constraint(num_constraints) {}
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  LinearConstraint(const Eigen::MatrixBase<DerivedA>& a,
                   const Eigen::MatrixBase<DerivedLB>& lb,
                   const Eigen::MatrixBase<DerivedUB>& ub)
      : Constraint(a.rows(), lb, ub), A(a) {
    assert(A.rows() == lb.rows());
  }
  virtual ~LinearConstraint() {}

  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(getNumConstraints());
    y = getMatrix() * x;
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(getNumConstraints());
    y = getMatrix().cast<TaylorVarXd>() * x;
  };

  virtual Eigen::SparseMatrix<double> getSparseMatrix() const {
    return getMatrix().sparseView();
  };
  virtual const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&
  getMatrix() const {
    return A;
  }

 protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
};

/** LinearEqualityConstraint
 * @brief Implements a constraint of the form @f Ax = b @f
 */
class LinearEqualityConstraint : public LinearConstraint {
 public:
  template <typename DerivedA, typename DerivedB>
  LinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                           const Eigen::MatrixBase<DerivedB>& beq)
      : LinearConstraint(Aeq, beq, beq) {}
  virtual ~LinearEqualityConstraint() {}

  /* updateConstraint
   * @brief change the parameters of the constraint (A and b), but not the
   *variable associations
   *
   * note that A and b can change size in the rows only (representing a
   *different number of linear constraints, but on the same decision variables)
   */
  template <typename DerivedA, typename DerivedB>
  void updateConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                        const Eigen::MatrixBase<DerivedB>& beq) {
    assert(Aeq.rows() == beq.rows());
    if (Aeq.cols() != A.cols())
      throw std::runtime_error("Can't change the number of decision variables");
    A.resize(Aeq.rows(), Eigen::NoChange);
    A = Aeq;
    lower_bound.conservativeResize(beq.rows());
    lower_bound = beq;
    upper_bound.conservativeResize(beq.rows());
    upper_bound = beq;
  };
};

/** BoundingBoxConstraint
*@brief Implements a constraint of the form @f lb <= x <= ub @f
*Note: the base Constraint class (as implemented at the moment) could play this
* role.  But this class enforces
*that it is ONLY a bounding box constraint, and not something more general.
*/
class BoundingBoxConstraint : public LinearConstraint {
 public:
  template <typename DerivedLB, typename DerivedUB>
  BoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb,
                        const Eigen::MatrixBase<DerivedUB>& ub)
      : LinearConstraint(Eigen::MatrixXd::Identity(lb.rows(), lb.rows()), lb,
                         ub) {}
  virtual ~BoundingBoxConstraint() {}

  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(getNumConstraints());
    y = x;
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(getNumConstraints());
    y = x;
  }
};
}

#endif
