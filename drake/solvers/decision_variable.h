#pragma once

#include <cstddef>
#include <list>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

class DecisionVariableScalar {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };
  DecisionVariableScalar(VarType type, const std::string& name, size_t index)
      : type_(type), name_(name), value_(0), index_(index) {}

  VarType type() const { return type_; }

  const std::string& name() const { return name_; }
  double value() const { return value_; }
  size_t index() const { return index_; }

  void set_value(double new_value) { value_ = new_value; }

 private:
  VarType type_;
  std::string name_;
  double value_;
  size_t index_;
};

/**
 * A matrix of decision variables. This class stores the decision variables.
 * If the matrix is not symmetric, namely X(i,j) does not necessarily equals to
 * X(j, i), then the decision variables are the stacked column of the matrix.
 * If the matrix is symmetric, then the decision variables are the stacked
 * columns of the lower triangular part of the matrix.
 */
class MatrixDecisionVariable {
 public:
  /**
   * Construct a matrix of decision variables of size rows x cols.
   * @param rows The number of rows in the matrix.
   * @param cols The number of columns in the matrix.
   * @param vars The decision variables stored in the matrix. If the matrix is
   * not symmetric, then @p vars contains the stacked column of the matrix;
   * otherwise @p vars contains the stacked columns of the lower triangular part
   * of the matrix.
   * For example, for a non-symmetric matrix M with 2 rows and 3 columns,
   * the decision variables are
   * [M(0,0) M(1,0) M(0,1) M(1,1) M(0,2) M(1,2)];
   * for a symmetric matrix S with 3 rows and 3 columns,
   * the decision variables are
   * [S(0,0) S(1,0) S(2,0) S(1,1) S(2,1) S(2,2)];
   * @param is_symmetric Whether the matrix is symmetric or not.
   */
  MatrixDecisionVariable(
      size_t rows, size_t cols,
      const std::vector<std::reference_wrapper<const DecisionVariableScalar>>&
          vars,
      bool is_symmetric = false)
      : rows_(rows),
        cols_(cols),
        is_symmetric_(is_symmetric),
        vars_(vars.begin(), vars.end()) {
    if (is_symmetric) {
      DRAKE_ASSERT(rows == cols && rows * (rows + 1) / 2 == vars.size());
    } else {
      DRAKE_ASSERT(rows * cols == vars.size());
    }
  }

  /**
   * Returns a new MatrixDecisionVariable, at i'th row and j'th column of
   * the original matrix.
   */
  MatrixDecisionVariable operator()(int i, int j) const {
    return MatrixDecisionVariable(1, 1,
                                  {vars_[MatrixIndicesToVectorIndex(i, j)]});
  }

  /**
   * Returns the value of the decision variable at i'th row and j'th column
   * of the original matrix.
   */
  double value(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().value();
  }

  /**
   * Return the type of the variable at i'th row and j'th column of the matrix.
   */
  DecisionVariableScalar::VarType type(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().type();
  }

  /**
   * Return the name of the variable at i'th row and j'th column of the matrix.
   */
  const std::string& name(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().name();
  }

  /**
   * Return the index of the variable at i'th row and j'th column of the matrix.
   */
  size_t index(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().index();
  }

  bool is_symmetric() const { return is_symmetric_; }

  /**
   * Return the number of decision variables stored in the matrix. If the matrix
   * is not symmetric, there are rows_ * cols_ decision variables; otherwise,
   * the number of decision variables is the number of entries in the lower
   * triangular part of the matrix.
   * @return
   */
  int NumberOfVariables() const {
    if (!is_symmetric_) {
      return rows_ * cols_;
    } else {
      return rows_ * (rows_ + 1) / 2;
    }
  }
  /**
   * Return the value of all the decision variables stored inside the class
   */
  Eigen::MatrixXd value() const {
    Eigen::MatrixXd mat(rows_, cols_);
    for (int i = 0; i < static_cast<int>(rows_); ++i) {
      for (int j = 0; j < static_cast<int>(cols_); ++j) {
        mat(i, j) = vars_[MatrixIndicesToVectorIndex(i, j)].get().value();
      }
    }
    return mat;
  }

  /**
   * Construct a new MatrixDecisionVariable by taking a sub-block from the
   * original matrix.
   * @param row_start The starting row of the sub-block.
   * @param col_start The starting column of the sub-block.
   * @param rows The number of rows in the sub-block.
   * @param cols The number of columns in the sub-block.
   */
  MatrixDecisionVariable block(size_t row_start, size_t col_start, size_t rows,
                               size_t cols) const {
    DRAKE_ASSERT(row_start + rows <= rows_ && col_start + cols <= cols_);
    std::vector<std::reference_wrapper<const DecisionVariableScalar>> vars;
    if (IsBlockSymmetric(row_start, col_start, rows, cols)) {
      vars.reserve(rows * (rows + 1) / 2);
      for (int j = 0; j < static_cast<int>(cols); ++j) {
        for (int i = j; i < static_cast<int>(rows); ++i) {
          vars.push_back(
              vars_[MatrixIndicesToVectorIndex(row_start + i, col_start + j)]);
        }
      }
      return MatrixDecisionVariable(rows, rows, vars, true);
    } else {
      vars.reserve(rows * cols);
      for (int j = 0; j < static_cast<int>(cols); ++j) {
        for (int i = 0; i < static_cast<int>(rows); ++i) {
          vars.push_back(
              vars_[MatrixIndicesToVectorIndex(row_start + i, col_start + j)]);
        }
      }
      return MatrixDecisionVariable(rows, cols, vars, false);
    }
  }

  /**
   * Construct a new MatrixDecisionVariable from a row of the original matrix.
   * @param i The index of the row.
   */
  MatrixDecisionVariable row(size_t i) const { return block(i, 0, 1, cols_); }

  /**
   * Construct a new MatrixDecisionVariable from a column of the original
   * matrix.
   * @param j The index of the column.
   */
  MatrixDecisionVariable col(size_t j) const { return block(0, j, rows_, 1); }

 private:
  size_t rows_;
  size_t cols_;
  bool is_symmetric_;
  std::vector<std::reference_wrapper<const DecisionVariableScalar>> vars_;

  /**
   * Return the index in vars_ given matrix index (i, j).
   */
  int MatrixIndicesToVectorIndex(size_t i, size_t j) const {
    DRAKE_ASSERT(i < rows_ && j < cols_);
    if (!is_symmetric_) {
      return j * rows_ + i;
    } else {
      if (i < j) {
        return MatrixIndicesToVectorIndex(j, i);
      }
      return (2 * rows_ - j + 1) * j / 2 + i - j;
    }
  }

  /**
   * Determines if a sub-block of the matrix is symmetric or not. This sub-block
   * starts from row @p row_start and column @p col_start; the dimension of the
   * sub-block is @p rows x @p cols.
   */
  bool IsBlockSymmetric(size_t row_start, size_t col_start, size_t rows,
                        size_t cols) const {
    if (!is_symmetric_) {
      return false;
    }
    if (row_start == col_start && rows == cols) {
      return true;
    }
    return false;
  }
};

/**
 * DecisionVariable
 * @brief Provides storage for a decision variable inside an
 * MathematicalProgram.
 */
class DecisionVariable {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  DecisionVariable(VarType type, const std::string& name, size_t num_vars,
                   size_t start_index)
      : type_(type),
        name_(name),
        data_(Eigen::VectorXd::Zero(num_vars)),
        start_index_(start_index) {}

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return start_index_; }
  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return data_.size(); }
  /** name()
   * @return the name of the DecisionVariable
   */
  const std::string& name() const { return name_; }
  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  const Eigen::VectorXd& value() const { return data_; }
  void set_value(const Eigen::VectorXd& new_data) { data_ = new_data; }

  VarType type() const { return type_; }

 private:
  VarType type_;
  std::string name_;
  Eigen::VectorXd data_;
  size_t start_index_;
};

class DecisionVariableView {  // enables users to access pieces of the decision
                              // variables like they would any other eigen
                              // vector
 public:
  /// Create a view which covers an entire DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  explicit DecisionVariableView(const DecisionVariable& var)
      : var_(var), start_index_(0), size_(var_.size()) {}

  /// Create a view covering part of a DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  DecisionVariableView(const DecisionVariable& var, size_t start, size_t n)
      : var_(var), start_index_(start), size_(n) {
    DRAKE_ASSERT((start + n) <= static_cast<size_t>(var.size()));
  }

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return var_.index() + start_index_; }

  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return size_; }

  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  Eigen::VectorBlock<const Eigen::VectorXd, Eigen::Dynamic> value() const {
    return var_.value().segment(start_index_, size_);
  }

  std::string name() const {
    if ((start_index_ == 0) &&
        (size_ == static_cast<size_t>(var_.value().size()))) {
      return var_.name();
    } else {
      return var_.name() + "(" + std::to_string(start_index_) + ":" +
             std::to_string(start_index_ + size_) + ")";
    }
  }

  /** covers()
   * @brief returns true iff the given @p index of the enclosing
   * MathematicalProgram is included in this VariableView.*/
  bool covers(size_t var_index) const {
    return (var_index >= index()) && (var_index < (index() + size_));
  }

  const DecisionVariableView operator()(size_t i) const {
    DRAKE_ASSERT(i <= size_);
    return DecisionVariableView(var_, start_index_ + i, 1);
  }
  const DecisionVariableView row(size_t i) const {
    DRAKE_ASSERT(i <= size_);
    return DecisionVariableView(var_, start_index_ + i, 1);
  }
  const DecisionVariableView head(size_t n) const {
    DRAKE_ASSERT(n <= size_);
    return DecisionVariableView(var_, start_index_, n);
  }
  const DecisionVariableView tail(size_t n) const {
    DRAKE_ASSERT(n <= size_);
    return DecisionVariableView(var_, start_index_ + size_ - n, n);
  }
  const DecisionVariableView segment(size_t start, size_t n) const {
    DRAKE_ASSERT(start + n <= size_);
    return DecisionVariableView(var_, start_index_ + start, n);
  }

 private:
  const DecisionVariable& var_;
  size_t start_index_, size_;
};

typedef std::list<DecisionVariableView> VariableList;
inline int GetVariableListSize(const VariableList& vars) {
  int var_dim = 0;
  for (const auto& var : vars) {
    var_dim += var.size();
  }
  return var_dim;
}

}  // end namespace solvers
}  // end namespace drake
