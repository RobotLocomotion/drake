#include "drake/solvers/indeterminate_variable.h"

namespace drake {
namespace solvers {
VectorXIndeterminateVariable ConcatenateIndeterminateVariableRefList(
    const IndeterminateVariableRefList& var_list) {
  int dim = 0;
  for (const auto& var : var_list) {
    dim += var.size();
  }
  VectorXIndeterminateVariable stacked_var(dim);
  int var_count = 0;
  for (const auto& var : var_list) {
    stacked_var.segment(var_count, var.rows()) = var;
    var_count += var.rows();
  }
  return stacked_var;
}
}  // namespace solvers
}  // namespace drake
