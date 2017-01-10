#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
VariableList::VariableList(const VariableListRef& variable_list) {
  variables_.resize(variable_list.size());
  auto variable_list_it = variable_list.begin();
  for (auto& var : variables_) {
    var = *variable_list_it;
    ++variable_list_it;
    size_ += var.size();
    column_vectors_only_ &= (var.cols() == 1);

    for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
      for (int j = 0; j < static_cast<int>(var.cols()); ++j) {
        unique_variable_indices_.insert(var(i, j));
      }
    }
  }
}
}  // namespace solvers
}  // namespace drake
