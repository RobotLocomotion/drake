#include "drake/solvers/constraint.h"

#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
               num_constraints() * num_constraints());
  Eigen::MatrixXd S(num_constraints(), num_constraints());

  int count = 0;
  for (size_t j = 0; j < num_constraints(); ++j) {
    for (size_t i = 0; i < num_constraints(); ++i) {
      S(i, j) = x(count);
      ++count;
    }
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
