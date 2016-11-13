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

  /** Construct an @ref undefined form with no variables. */
  DecisionVariableScalar() {}
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
  double value() const { return *value_; }

  /**
   * @return The index of the variable in the optimization program.
   */
  size_t index() const { return index_; }

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
      : type_(type), name_(name), value_(new double(0)), index_(index) {}

  void set_value(double new_value) { *value_ = new_value; }

  VarType type_;
  std::string name_;
  double* value_;
  size_t index_;
};

template<Eigen::Index rows, Eigen::Index cols>
using DecisionVariableMatrix = Eigen::Matrix<const DecisionVariableScalar, rows, cols>;
template<Eigen::Index rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX = DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using DecisionVariableMatrixXRef = Eigen::Ref<const DecisionVariableMatrixX>;

typedef std::vector<Eigen::Ref<const DecisionVariableMatrixX>> VariableVector;

template<typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
DecisionVariableMatrixToDoubleMatrix(const Eigen::MatrixBase<Derived>& decision_variable_matrix) {
  Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> double_matrix(decision_variable_matrix.rows(), decision_variable_matrix.cols());
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      double_matrix(i, j) = decision_variable_matrix(i,j).value();
    }
  }
  return double_matrix;
}

/**
 * Determine if a DecisionVariableMatrix object covers a variable with
 * given index.
 */
template<typename Derived>
bool DecisionVariableMatrixCoversIndex(const Eigen::MatrixBase<Derived>& decision_variable_matrix, size_t index) {
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      if (decision_variable_matrix(i,j).index() == index) {
        return true;
      }
    }
  }
  return false;
}

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
