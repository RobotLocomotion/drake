#include "drake/solvers/indeterminate.h"

namespace drake {
namespace solvers {
VectorXIndeterminate ConcatenateIndeterminatesRefList(
    const IndeterminatesRefList& var_list) {
  int dim = 0;
  for (const auto& var : var_list) {
    dim += var.size();
  }
  VectorXIndeterminate stacked_var(dim);
  int var_count = 0;
  for (const auto& var : var_list) {
    stacked_var.segment(var_count, var.rows()) = var;
    var_count += var.rows();
  }
  return stacked_var;
}
}  // namespace solvers
}  // namespace drake
