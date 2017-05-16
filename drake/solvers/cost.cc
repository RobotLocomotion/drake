#include "drake/solvers/cost.h"

#include <memory>

using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

void CostShimBase::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::VectorXd& y) const {
  impl_->Eval(x, y);
}
void CostShimBase::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          AutoDiffVecXd& y) const {
  impl_->Eval(x, y);
}

shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const MatrixXd>& Q,
    const Eigen::Ref<const VectorXd>& x_desired) {
  return make_shared<QuadraticCost>(2 * Q, -2 * Q * x_desired);
}

shared_ptr<QuadraticCost> MakeL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  return make_shared<QuadraticCost>(2 * A.transpose() * A,
                                    -2 * A.transpose() * b);
}

}  // namespace solvers
}  // namespace drake
