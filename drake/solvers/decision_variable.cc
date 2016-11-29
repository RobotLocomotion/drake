#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
size_t DecisionVariableScalarHash::operator()(
    const DecisionVariableScalar& var) const {
  return var.index();
}

bool IsDecisionVariableMatrixSymmetric(
    const Eigen::Ref<const DecisionVariableMatrixX> matrix) {
  if (matrix.rows() != matrix.cols()) {
    return false;
  }
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = i+1; j < matrix.cols(); ++j) {
      if (matrix(i, j) != matrix(j, i)) {
        return false;
      }
    }
  }
  return true;
}

VariableList::VariableList(
    const VariableListRef& variable_list) {
  variables_.resize(variable_list.size());
  size_ = 0;
  column_vectors_only_ = true;
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
