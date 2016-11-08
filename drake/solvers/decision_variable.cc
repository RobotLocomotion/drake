#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
Eigen::MatrixXd DecisionVariableMatrix::value() const {
  Eigen::MatrixXd mat(rows_, cols_);
  for (int i = 0; i < static_cast<int>(rows_); ++i) {
    for (int j = 0; j < static_cast<int>(cols_); ++j) {
      size_t vector_index = MatrixIndicesToVectorIndex(i, j);
      mat(i, j) = vars_[vector_index]->value();
    }
  }
  return mat;
}

Eigen::VectorXd DecisionVariableMatrix::VariableValue() const {
  Eigen::VectorXd vec(NumberOfVariables());
  for (int i = 0; i < vec.size(); ++i) {
    vec(i) = vars_[i]->value();
  }
  return vec;
}

DecisionVariableMatrix DecisionVariableMatrix::block(size_t row_start,
                                                     size_t col_start,
                                                     size_t rows,
                                                     size_t cols) const {
  DRAKE_ASSERT(row_start + rows <= rows_ && col_start + cols <= cols_);
  std::vector<std::shared_ptr<const DecisionVariableScalar>> vars;
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
    if (vars_[i]->index() == index) {
      return true;
    }
  }
  return false;
}

int GetVariableVectorSize(const VariableVector& vars) {
  int var_dim = 0;
  for (const auto& var : vars) {
    var_dim += var.NumberOfVariables();
  }
  return var_dim;
}

bool VariableVectorContainsColumnVectorsOnly(const VariableVector& vars) {
  for (const DecisionVariableMatrix& var : vars) {
    if (var.cols() != 1) {
      return false;
    }
  }
  return true;
}

}  // namespace solvers
}  // namespace drake
