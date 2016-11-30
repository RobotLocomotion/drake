#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
namespace {
bool IsMatrixSymmetric(const Eigen::Ref<const Eigen::MatrixXd> X, double precision = std::numeric_limits<double>::epsilon()) {
  if (X.rows() != X.cols()) { return false;}
  for (int i = 0; i < static_cast<int>(X.rows()); ++i) {
    for (int j = i; j < static_cast<int>(X.cols()); ++j) {
      if (std::abs(X(i, j) - X(j, i)) > precision) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace
void PositiveSemidefiniteConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
          Eigen::VectorXd& y) const {
throw std::runtime_error(
"The Eval function for positive semidefinite constraint is not defined.");
}

void PositiveSemidefiniteConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
          TaylorVecXd& y) const {
throw std::runtime_error(
"The Eval function for positive semidefinite constraint is not defined.");
}

void LinearMatrixInequalityConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
          Eigen::VectorXd& y) const {
throw std::runtime_error(
"The Eval function for linear matrix inequality constraint is not defined.");
}

void LinearMatrixInequalityConstraint::Eval(const Eigen::Ref<const TaylorVecXd>& x,
          TaylorVecXd& y) const {
throw std::runtime_error(
"The Eval function for linear matrix inequality constraint is not defined.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(const std::list<
    Eigen::Ref<const Eigen::MatrixXd>> &F) : Constraint(0), matrix_rows_(F.front().rows()){
  F_.resize(F.size());
  auto F_it = F_.begin();
  for (const auto& Fi : F) {
    DRAKE_ASSERT(Fi.rows() == F.front().rows());
    DRAKE_ASSERT(IsMatrixSymmetric(Fi, 1E-10));
    *F_it = Fi;
    ++F_it;
  }
}
}
}