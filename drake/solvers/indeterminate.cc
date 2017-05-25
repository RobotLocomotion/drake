#include "drake/solvers/indeterminate.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
VectorXIndeterminate ConcatenateIndeterminatesRefList(
    const IndeterminatesRefList& var_list) {
  // TODO(fischergundlach) : Move the function
  // drake::solvers::decision_variables::ConcatenateVariableRefList to
  // drake::symbolic::variable.
  return ConcatenateVariableRefList(var_list);
}
}  // namespace solvers
}  // namespace drake
