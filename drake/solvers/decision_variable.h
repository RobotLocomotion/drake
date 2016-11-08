#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
/**
 * This class stores the type, the name, the value, and the index of a
 * decision variable in an optimization program.
 */
class DecisionVariableScalar {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  /**
   * @return The type of the variable.
   */
  VarType type() const { return type_; }

  /**
   * @return The name of the variable.
   */
  std::string name() const { return name_; }

  /**
   * @return The value of the variable. This method is only meaningful after
   * calling Solve() in MathematicalProgram.
   */
  double value() const { return value_; }

  /**
   * @return The index of the variable in the optimization program.
   */
  size_t index() const { return index_; }

  void set_value(double new_value) { value_ = new_value; }

  friend class MathematicalProgram;

 private:
  /**
   * Construct a decision variable. We make this constructor private so that
   * only the friend class (aka MathematicalProgram) can construct a decision
   * variable.
   * @param type Support CONTINUOUS, INTEGER or BINARY.
   * @param name The name of the variable.
   * @param index The index of the variable in the optimization program.
   */
  DecisionVariableScalar(VarType type, const std::string& name, size_t index)
      : type_(type), name_(name), value_(0), index_(index) {}
  const VarType type_;
  const std::string name_;
  double value_;
  const size_t index_;
};

/**
 * This class stores a matrix of weak pointers, pointing to decision variables.
 * If the matrix is not symmetric, namely X(i,j) does not necessarily equals to
 * X(j, i), then the decision variables are the stacked column of the matrix.
 * If the matrix is symmetric, then the decision variables are the stacked
 * columns of the lower triangular part of the matrix.
 */
class DecisionVariableMatrix {
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
  DecisionVariableMatrix(
      size_t rows, size_t cols,
      const std::vector<std::shared_ptr<const DecisionVariableScalar>>& vars,
      bool is_symmetric = false)
      : rows_(rows),
        cols_(cols),
        is_symmetric_(is_symmetric),
        vars_(vars.begin(), vars.end()) {
    DRAKE_ASSERT(!is_symmetric || rows == cols);
    DRAKE_ASSERT(static_cast<int>(vars.size()) == NumberOfVariables());
  }

  /**
   * @return The number of rows in the matrix.
   */
  size_t rows() const { return rows_; }

  /**
   * @return The number of columns in the matrix.
   */
  size_t cols() const { return cols_; }

  /**
   * @return A new DecisionVariableMatrix containing only 1 decision variable,
   * at the i'th row and the j'th column of the original matrix.
   */
  DecisionVariableMatrix operator()(int i, int j) const {
    return DecisionVariableMatrix(1, 1,
                                  {vars_[MatrixIndicesToVectorIndex(i, j)]});
  }

  /**
   * Returns a new DecisionVariableMatrix, containing the i'th decision
   * variable stored in the original matrix.
   * For a non-symmetric matrix, the decision variables are stored in the column
   * major, as the stacked columns of the matrix; for a symmetric matrix, the
   * decision variables are stored as the stacked columns of the lower
   * triangular
   * part of the matrix.
   * For example, for a 3 x 3 non-symmetric matrix M, the 5'th decision variable
   * (0-indexed) is at M(2, 1);
   * for a 3 x 3 symmetric matrix M, the 5'th decision variable (0-indexed) is
   * at M(2, 2).
   */
  DecisionVariableMatrix operator()(int i) const {
    return DecisionVariableMatrix(1, 1, {vars_[i]});
  }

  /**
   * @return The value of the decision variable at the i'th row and the
   * j'th column of the original matrix.
   */
  double value(size_t i, size_t j) const {
    size_t vector_index = MatrixIndicesToVectorIndex(i, j);
    return vars_[vector_index]->value();
  }

  /**
   * Returns the value of the i'th decision variable stored in the matrix.
   * For a non-symmetric matrix, the decision variables are stored in the column
   * major, as the stacked columns of the matrix; for a symmetric matrix, the
   * decision variables are stored as the stacked columns of the lower
   * triangular
   * part of the matrix.
   * For example, for a 3 x 3 non-symmetric matrix M, the 5'th decision variable
   * (0-indexed) is at M(2, 1);
   * for a 3 x 3 symmetric matrix M, the 5'th decision variable (0-indexed) is
   * at M(2, 2).
   * @return The value of the decision variable.
   */
  double value(size_t i) const {
    DRAKE_ASSERT(static_cast<int>(i) < NumberOfVariables());
    return vars_[i]->value();
  }

  /**
   * @return the type of the variable at i'th row and j'th column of the matrix.
   */
  DecisionVariableScalar::VarType type(size_t i, size_t j) const {
    size_t vector_index = MatrixIndicesToVectorIndex(i, j);
    return vars_[vector_index]->type();
  }

  /**
   * Returns the type of the i'th decision variable stored in the matrix.
   * For a non-symmetric matrix, the decision variables are stored in the column
   * major, as the stacked columns of the matrix; for a symmetric matrix, the
   * decision variables are stored as the stacked columns of the lower
   * triangular
   * part of the matrix.
   * For example, for a 3 x 3 non-symmetric matrix M, the 5'th decision variable
   * (0-indexed) is at M(2, 1);
   * for a 3 x 3 symmetric matrix M, the 5'th decision variable (0-indexed) is
   * at M(2, 2).
   * @return The type of the decision variable.
   */
  DecisionVariableScalar::VarType type(size_t i) const {
    DRAKE_ASSERT(static_cast<int>(i) < NumberOfVariables());
    return vars_[i]->type();
  }

  /**
   * @return The name of the variable at i'th row and j'th column of the matrix.
   */
  std::string name(size_t i, size_t j) const {
    size_t vector_index = MatrixIndicesToVectorIndex(i, j);
    return vars_[vector_index]->name();
  }

  /**
   * Returns the name of the i'th decision variable stored in the matrix.
   * For a non-symmetric matrix, the decision variables are stored in the column
   * major, as the stacked columns of the matrix; for a symmetric matrix, the
   * decision variables are stored as the stacked columns of the lower
   * triangular
   * part of the matrix.
   * For example, for a 3 x 3 non-symmetric matrix M, the 5'th decision variable
   * (0-indexed) is at M(2, 1);
   * for a 3 x 3 symmetric matrix M, the 5'th decision variable (0-indexed) is
   * at M(2, 2).
   * @return The name of the decision variable.
   */
  std::string name(size_t i) const {
    DRAKE_ASSERT(static_cast<int>(i) < NumberOfVariables());
    return vars_[i]->name();
  }

  /**
   * @return The index of the variable at the i'th row and the j'th column
   * of the matrix.
   */
  size_t index(size_t i, size_t j) const {
    size_t vector_index = MatrixIndicesToVectorIndex(i, j);
    return vars_[vector_index]->index();
  }

  /**
   * Returns the index of the i'th decision variable stored in the matrix.
   * For a non-symmetric matrix, the decision variables are stored in the column
   * major, as the stacked columns of the matrix; for a symmetric matrix, the
   * decision variables are stored as the stacked columns of the lower
   * triangular
   * part of the matrix.
   * For example, for a 3 x 3 non-symmetric matrix M, the 5'th decision variable
   * (0-indexed) is at M(2, 1);
   * for a 3 x 3 symmetric matrix M, the 5'th decision variable (0-indexed) is
   * at M(2, 2).
   * @return The index of the decision variable.
   */
  size_t index(size_t i) const {
    DRAKE_ASSERT(static_cast<int>(i) < NumberOfVariables());
    return vars_[i]->index();
  }

  bool is_symmetric() const { return is_symmetric_; }

  /**
   * Return the number of decision variables stored in the matrix. If the matrix
   * is not symmetric, there are rows_ * cols_ decision variables; otherwise,
   * the number of decision variables is the number of entries in the lower
   * triangular part of the matrix.
   */
  int NumberOfVariables() const {
    if (!is_symmetric_) {
      return rows_ * cols_;
    } else {
      return rows_ * (rows_ + 1) / 2;
    }
  }

  /**
   * Return the value of matrix.
   * @return A matrix of the size rows_ * cols_;
   */
  Eigen::MatrixXd value() const;

  /** Return the value of the decision variables
   * @return A vector of the size NumberOfVariables().
   */
  Eigen::VectorXd VariableValue() const;

  /**
   * Construct a new DecisionVariableMatrix by taking a sub-block from the
   * original matrix.
   * @param row_start The starting row of the sub-block.
   * @param col_start The starting column of the sub-block.
   * @param rows The number of rows in the sub-block.
   * @param cols The number of columns in the sub-block.
   */
  DecisionVariableMatrix block(size_t row_start, size_t col_start, size_t rows,
                               size_t cols) const;

  /**
   * Construct a new DecisionVariableMatrix from a row of the original matrix.
   * @param i The index of the row.
   */
  DecisionVariableMatrix row(size_t i) const { return block(i, 0, 1, cols_); }

  /**
   * Construct a new DecisionVariableMatrix from a column of the original
   * matrix.
   * @param j The index of the column.
   */
  DecisionVariableMatrix col(size_t j) const { return block(0, j, rows_, 1); }

  /**
   * Construct a new DecisionVariableMatrix containing the first @p i decision
   * variables of the original matrix. Only works if the original matrix is
   * a column vector (has only one column).
   */
  DecisionVariableMatrix head(size_t i) const {
    DRAKE_ASSERT(cols_ == 1);
    return block(0, 0, i, 1);
  }

  /**
   * Construct a new DecisionVariableMatrix containing the last @p i decision
   * variables of the original matrix. Only works if the original matrix is
   * a column vector (has only one column).
   */
  DecisionVariableMatrix tail(size_t i) const {
    DRAKE_ASSERT(cols_ == 1);
    return block(rows_ - i, 0, i, 1);
  }

  /**
   * Construct a new DecisionVariableMatrix containing a chunk of the decision
   * variables of the original matrix, starting from the @p start_row,
   * with @p i elements. Only works if the original matrix is
   * a column vector (has only one column).
   */
  DecisionVariableMatrix segment(size_t start_row, size_t rows) const {
    DRAKE_ASSERT(cols_ == 1);
    return block(start_row, 0, rows, 1);
  }

  /**
   * Construct a new DecisionVariableMatrix containing a chunk of the decision
   * variables of the original matrix, starting from the @p start_row,
   * with @p rows rows.
   */
  DecisionVariableMatrix middleRows(size_t start_row, size_t rows) const {
    return block(start_row, 0, rows, cols_);
  }

  /**
   * Construct a new DecisionVariableMatrix containing a chunk of the decision
   * variables of the original matrix, starting from the @p start_col,
   * with @p cols columns.
   */
  DecisionVariableMatrix middleCols(size_t start_col, size_t cols) const {
    return block(0, start_col, rows_, cols);
  }
  /**
   * Determine if the variable with given index is included in this matrix.
   * @param index The index of the variable in the mathematical program.
   * @return True if the variable is included in the matrix; false otherwise.
   */
  bool covers(size_t index) const;

 private:
  const size_t rows_;
  const size_t cols_;
  const bool is_symmetric_;
  const std::vector<std::shared_ptr<const DecisionVariableScalar>> vars_;

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

typedef std::vector<DecisionVariableMatrix> VariableVector;

/**
 * Given a vector of DecisionVariableMatrix @p vars, computes the TOTAL number
 * of scalar decision variables stored in @p vars, including duplication.
 * So if vars[0] contains decision variable x0, x1, x2, vars[1] contains
 * variable x1, x3, then GetVariableVectorSize(vars) will return 5, and count
 * x1 for twice.
 */
int GetVariableVectorSize(const VariableVector& vars);

/**
 * Given a std::vector of DecisionVariableMatrix @p vars, returns true if all
 * DecisionVariableMatrix objects have only 1 column (thus a column vector or a
 * scalar).
 */
bool VariableVectorContainsColumnVectorsOnly(const VariableVector& vars);
}  // end namespace solvers
}  // end namespace drake
