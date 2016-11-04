#include "drake/solvers/decision_variable.h"


namespace drake {
namespace solvers {
Eigen::MatrixXd DecisionVariableMatrix::value() const {
  Eigen::MatrixXd mat(rows_, cols_);
  for (int i = 0; i < static_cast<int>(rows_); ++i) {
    for (int j = 0; j < static_cast<int>(cols_); ++j) {
      size_t vector_index = MatrixIndicesToVectorIndex(i, j);
      DRAKE_ASSERT(!vars_[vector_index].expired());
      mat(i, j) = vars_[vector_index].lock()->value();
    }
  }
  return mat;
}

Eigen::VectorXd DecisionVariableMatrix::VariableValue() const {
  Eigen::VectorXd vec(NumberOfVariables());
  for (int i = 0; i < vec.size(); ++i) {
    DRAKE_ASSERT(!vars_[i].expired());
    vec(i) = vars_[i].lock()->value();
  }
  return vec;
}

DecisionVariableMatrix DecisionVariableMatrix::block(size_t row_start,
                                                     size_t col_start,
                                                     size_t rows,
                                                     size_t cols) const {
  DRAKE_ASSERT(row_start + rows <= rows_ && col_start + cols <= cols_);
  std::vector<std::weak_ptr<const DecisionVariableScalar>> vars;
  if (IsBlockSymmetric(row_start, col_start, rows, cols)) {
    vars.reserve(rows * (rows + 1) / 2);
    for (int j = 0; j < static_cast<int>(cols); ++j) {
      for (int i = j; i < static_cast<int>(rows); ++i) {
        vars.push_back(
            vars_[MatrixIndicesToVectorIndex(row_start + i, col_start + j)]);
      }
    }
    return DecisionVariableMatrix(rows, rows, vars, true);
  } else {
    vars.reserve(rows * cols);
    for (int j = 0; j < static_cast<int>(cols); ++j) {
      for (int i = 0; i < static_cast<int>(rows); ++i) {
        vars.push_back(
            vars_[MatrixIndicesToVectorIndex(row_start + i, col_start + j)]);
      }
    }
    return DecisionVariableMatrix(rows, cols, vars, false);
  }
}

bool DecisionVariableMatrix::covers(size_t index) const {
  for (int i = 0; i < static_cast<int>(vars_.size()); ++i) {
    DRAKE_ASSERT(!vars_[i].expired());
    if (vars_[i].lock()->index() == index) {
      return true;
    }
  }
  return false;
}
}  // namespace solvers
}  // namespace drake
