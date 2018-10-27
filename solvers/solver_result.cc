#include "drake/solvers/solver_result.h"

namespace drake {
namespace solvers {
void SolverResult::set_decision_variable_values(
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  decision_variable_values_.reset();
  decision_variable_values_.emplace(values);
}
}  // namespace solvers
}  // namespace drake
