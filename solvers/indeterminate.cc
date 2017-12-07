#include "drake/solvers/indeterminate.h"

#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
VectorXIndeterminate ConcatenateIndeterminatesRefList(
    const IndeterminatesRefList& var_list) {
  // TODO(fischergundlach): Consolidate DecisionVariable and Indeterminate in
  // variable.{h,cc}.
  return ConcatenateVariableRefList(var_list);
}
}  // namespace solvers
}  // namespace drake
