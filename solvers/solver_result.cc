#include "drake/solvers/solver_result.h"

namespace drake {
namespace solvers {
namespace internal {
void SolverResult::set_decision_variable_values(
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  decision_variable_values_.reset();
  decision_variable_values_.emplace(values);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
