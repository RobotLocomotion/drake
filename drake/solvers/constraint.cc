#include "drake/solvers/constraint.h"

#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
Constraint::Constraint(Constraint&& rhs)
    : Constraint(rhs.num_constraints(), std::move(rhs.lower_bound()),
                 std::move(rhs.upper_bound()),
                 std::move(rhs.get_description())) {}

Constraint& Constraint::operator=(Constraint&& rhs) {
  lower_bound_ = std::move(rhs.lower_bound());
  upper_bound_ = std::move(rhs.upper_bound());
  description_ = std::move(rhs.get_description());
  return *this;
}

void QuadraticConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                               Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
}

void QuadraticConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                               TaylorVecXd& y) const {
  y.resize(num_constraints());
  y = .5 * x.transpose() * Q_.cast<TaylorVarXd>() * x +
      b_.cast<TaylorVarXd>().transpose() * x;
};

QuadraticConstraint& QuadraticConstraint::operator=(QuadraticConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  Q_ = std::move(rhs.Q());
  b_ = std::move(rhs.b());
  return *this;
}

void LorentzConeConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y(0) = x(0);
  y(1) = pow(x(0), 2) - x.tail(x.size() - 1).squaredNorm();
}

void LorentzConeConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                                 TaylorVecXd& y) const {
  y.resize(num_constraints());
  y(0) = x(0);
  y(1) = pow(x(0), 2) - x.tail(x.size() - 1).squaredNorm();
}

LorentzConeConstraint& LorentzConeConstraint::operator=(
    LorentzConeConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  return *this;
}

RotatedLorentzConeConstraint& RotatedLorentzConeConstraint::operator=(
    RotatedLorentzConeConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  return *this;
}

void RotatedLorentzConeConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y(0) = x(0);
  y(1) = x(1);
  y(2) = x(0) * x(1) - x.tail(x.size() - 2).squaredNorm();
}

void RotatedLorentzConeConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                                        TaylorVecXd& y) const {
  y.resize(num_constraints());
  y(0) = x(0);
  y(1) = x(1);
  y(2) = x(0) * x(1) - x.tail(x.size() - 2).squaredNorm();
}

void PolynomialConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd& y) const {
  double_evaluation_point_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    double_evaluation_point_[poly_vars_[i]] = x[i];
  }
  y.resize(num_constraints());
  for (size_t i = 0; i < num_constraints(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(double_evaluation_point_);
  }
}

void PolynomialConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                                TaylorVecXd& y) const {
  taylor_evaluation_point_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    taylor_evaluation_point_[poly_vars_[i]] = x[i];
  }
  y.resize(num_constraints());
  for (size_t i = 0; i < num_constraints(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_);
  }
}

LinearConstraint& LinearConstraint::operator=(LinearConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  A_ = std::move(rhs.A_);
  return *this;
}

void LinearConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                            Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y = A_ * x;
}
void LinearConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                            TaylorVecXd& y) const {
  y.resize(num_constraints());
  y = A_.cast<TaylorVarXd>() * x;
};

LinearEqualityConstraint& LinearEqualityConstraint::operator=(LinearEqualityConstraint&& rhs) {
  LinearConstraint::operator=(std::move(rhs));
  return *this;
}

BoundingBoxConstraint& BoundingBoxConstraint::operator=(BoundingBoxConstraint&& rhs) {
  LinearConstraint::operator=(std::move(rhs));
  return *this;
}

void BoundingBoxConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y = x;
}
void BoundingBoxConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
                                 TaylorVecXd& y) const {
  y.resize(num_constraints());
  y = x;
}

LinearComplementarityConstraint& LinearComplementarityConstraint::operator=(LinearComplementarityConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  M_ = std::move(rhs.M_);
  q_ = std::move(rhs.q_);
  return *this;
}

void LinearComplementarityConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  y.resize(num_constraints());
  y = (M_ * x) + q_;
}

void LinearComplementarityConstraint::Eval(
    const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const {
  y.resize(num_constraints());
  y = (M_.cast<TaylorVarXd>() * x) + q_.cast<TaylorVarXd>();
};

PositiveSemidefiniteConstraint& PositiveSemidefiniteConstraint::operator=(PositiveSemidefiniteConstraint&& rhs) {
  Constraint::operator=(std::move(rhs));
  return *this;
}

void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
               num_constraints() * num_constraints());
  Eigen::MatrixXd S(num_constraints(), num_constraints());

  for (int i = 0; i < static_cast<int>(num_constraints()); ++i) {
    S.col(i) = x.segment(i * num_constraints(), num_constraints());
  }

  DRAKE_ASSERT(S.rows() == static_cast<int>(num_constraints()));

  // This uses the lower diagonal part of S to compute the eigen values.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  y = eigen_solver.eigenvalues();
}

void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

void LinearMatrixInequalityConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  DRAKE_ASSERT(x.rows() == static_cast<int>(F_.size()) - 1);
  Eigen::MatrixXd S = F_[0];
  for (int i = 1; i < static_cast<int>(F_.size()); ++i) {
    S += x(i - 1) * F_[i];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  y = eigen_solver.eigenvalues();
}

void LinearMatrixInequalityConstraint::Eval(
    const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    double symmetry_tolerance)
    : Constraint(F.empty() ? 0 : F.front().rows()),
      F_(F.begin(), F.end()),
      matrix_rows_(F.empty() ? 0 : F.front().rows()) {
  DRAKE_DEMAND(!F.empty());
  set_bounds(Eigen::VectorXd::Zero(matrix_rows_),
             Eigen::VectorXd::Constant(
                 matrix_rows_, std::numeric_limits<double>::infinity()));
  for (const auto& Fi : F) {
    DRAKE_ASSERT(Fi.rows() == matrix_rows_);
    DRAKE_ASSERT(math::IsSymmetric(Fi, symmetry_tolerance));
  }
}
}  // namespace solvers
}  // namespace drake
