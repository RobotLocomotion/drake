#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
template <typename T>
int GetVariableVectorSize_impl(const T& vars) {
  int var_dim = 0;
  for (const auto& var : vars) {
    DRAKE_ASSERT(var.cols() == 1);
    var_dim += var.rows();
  }
  return var_dim;
}
int GetVariableVectorSize(const VariableVector& vars) {
  return GetVariableVectorSize_impl(vars);
}

int GetVariableVectorRefSize(const VariableVectorRef& vars) {
  return GetVariableVectorSize_impl(vars);
}

template <typename T>
bool VariableVectorContainsColumnVectorsOnly_impl(const T& vars) {
  for (const auto& var : vars) {
    if (var.cols() != 1) {
      return false;
    }
  }
  return true;
}

bool VariableVectorContainsColumnVectorsOnly(const VariableVector& vars) {
  return VariableVectorContainsColumnVectorsOnly_impl(vars);
}

bool VariableVectorRefContainsColumnVectorsOnly(const VariableVectorRef& vars) {
  return VariableVectorContainsColumnVectorsOnly_impl(vars);
}

}  // namespace solvers
}  // namespace drake
