#include "drake/solvers/cost.h"

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

}  // namespace solvers
}  // namespace drake
