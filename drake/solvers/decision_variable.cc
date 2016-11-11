#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
int GetVariableVectorSize(const VariableVector& vars) {
  int var_dim = 0;
  for (const auto& var : vars) {
    DRAKE_ASSERT(var.cols() == 1);
    var_dim += var.rows();
  }
  return var_dim;
}

bool VariableVectorContainsColumnVectorsOnly(const VariableVector& vars) {
  for (const auto& var : vars) {
    if (var.cols() != 1) {
      return false;
    }
  }
  return true;
}

}  // namespace solvers
}  // namespace drake
