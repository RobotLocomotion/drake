#include "drake/solvers/constraint.h"

#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined.");
}

void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const {
  throw std::runtime_error(
      "The Eval function for positive semidefinite constraint is not defined.");
}

void LinearMatrixInequalityConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  throw std::runtime_error(
      "The Eval function for linear matrix inequality constraint is not "
      "defined.");
}

void LinearMatrixInequalityConstraint::Eval(
    const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const {
  throw std::runtime_error(
      "The Eval function for linear matrix inequality constraint is not "
      "defined.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(
    const std::list<Eigen::Ref<const Eigen::MatrixXd>>& F)
    : Constraint(0), matrix_rows_(F.front().rows()) {
  F_.resize(F.size());
  auto F_it = F_.begin();
  for (const auto& Fi : F) {
    DRAKE_ASSERT(Fi.rows() == F.front().rows());
    DRAKE_ASSERT(math::IsSymmetric(Fi, 1E-10));
    *F_it = Fi;
    ++F_it;
  }
}
}  // namespace solvers
}  // namespace drake
