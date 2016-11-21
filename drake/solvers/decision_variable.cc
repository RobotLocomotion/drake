#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
namespace {
template<typename T>
int GetVariableVectorSize_impl(const T &vars) {
  int var_dim = 0;
  for (const auto &var : vars) {
    DRAKE_ASSERT(var.cols() == 1);
    var_dim += var.rows();
  }
  return var_dim;
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
}  // namespace

int size(const VariableList &vars) {
  return GetVariableVectorSize_impl(vars);
}

int size(const VariableListRef &vars) {
  return GetVariableVectorSize_impl(vars);
}


bool VariableListContainsColumnVectorsOnly(const VariableList &vars) {
  return VariableVectorContainsColumnVectorsOnly_impl(vars);
}

bool VariableListRefContainsColumnVectorsOnly(const VariableListRef &vars) {
  return VariableVectorContainsColumnVectorsOnly_impl(vars);
}

}  // namespace solvers
}  // namespace drake
