#include "drake/solvers/constraint.h"

#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
void PositiveSemidefiniteConstraint::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
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
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F, double symmetry_tolerance)
    : Constraint(0), F_(F.begin(), F.end()), matrix_rows_(F.empty() ? 0 : F.front().rows()) {
  DRAKE_DEMAND(!F.empty());
}
}  // namespace solvers
}  // namespace drake
