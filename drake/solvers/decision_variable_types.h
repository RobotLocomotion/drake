#pragma once

#include <cstddef>        // for size_t
#include <list>           // for list
#include <type_traits>    // for is_same
#include <unordered_set>  // for unordered_set

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_decision_variable.h"

namespace drake {
namespace solvers {
using DecisionVariableScalar = drake::symbolic::DecisionVariableScalar;
template <int rows, int cols>
using DecisionVariableMatrix =
    Eigen::Matrix<drake::solvers::DecisionVariableScalar, rows, cols>;
template <int rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX =
    DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using VariableListRef = std::list<Eigen::Ref<const DecisionVariableMatrixX>>;

struct DecisionVariableScalarHash {
  size_t operator()(const DecisionVariableScalar& var) const;
};

/**
 * This class stores a list of DecisionVariableMatrix objects. An instance
 * of this class is going to be bound to a constraint, indicating that a
 * constraint is imposed on one or several DecisionVariableMatrix objects.
 */
class VariableList {
 public:
  explicit VariableList(const VariableListRef& variable_list);

  /**
   * Returns all the stored DecisionVariableMatrix.
   */
  const std::list<DecisionVariableMatrixX>& variables() const {
    return variables_;
  }

  /**
   * Given a list of DecisionVariableMatrix @p vars, computes the TOTAL number
   * of unique scalar decision variables stored in @p vars.
   * Example
   * @code{.cc}
   * // Create a mathematical program with no decision variables.
   * MathematicalProgram prog;
   *
   * // Add a vector containing 4 decision variables.
   * auto x = prog.AddContinuousVariables<4>();
   *
   * // x1 contains x(0), x(1), x(2).
   * DecisionVariableVector<3> x1 = x.head<3>();
   *
   * // x2 contains x(2), x(3).
   * DecisionVariableVector<2> x2 = x.tail<2>();
   *
   * // Construct a VariableList containing both x1 and x2.
   * VariableList var_list({x1, x2});
   *
   * std::cout << "The number of unique variables is " <<
   *     var_list.num_unique_variables() << std::endl;
   *
   * std::cout << "The size of variable list (including duplication) is " <<
   *     var_list.size() << std::endl;
   * @endcode
   *
   * The output is
   * <pre>
   * The number of unique variables is 4
   * The size of variable list (including duplication) is 5
   * </pre>
   */
  size_t num_unique_variables() const {
    return unique_variable_indices_.size();
  }

  /**
   * Given a list of DecisionVariableMatrix @p vars, computes the TOTAL number
   * of scalar decision variables stored in @p vars, including duplication. The
   * duplicated variables will be counted for more than once.
   * @see num_unique_variables() for an example.
   */
  size_t size() const { return size_; }

  /**
   * Determines if the stored DecisionVariableMatrix objects are
   * all column vectors.
   */
  bool column_vectors_only() const { return column_vectors_only_; }

  /**
   * @return The all unique variables stored in the class.
   */
  const std::unordered_set<DecisionVariableScalar, DecisionVariableScalarHash>&
  unique_variables() const {
    return unique_variable_indices_;
  }

 private:
  std::list<DecisionVariableMatrixX> variables_;
  size_t size_;
  bool column_vectors_only_;
  std::unordered_set<DecisionVariableScalar, DecisionVariableScalarHash>
      unique_variable_indices_;
};
}  // end namespace solvers

/**
 * Given a DecisionVariableMatrix object, returns the Eigen::Matrix that
 * stores the values of each decision variable.
 * @tparam Derived A DecisionVariableMatrix class.
 * @param decision_variable_matrix A DecisionVariableMatrix object.
 */
template <typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
GetSolution(const Eigen::MatrixBase<Derived>& decision_variable_matrix) {
  static_assert(std::is_same<typename Derived::Scalar,
                             solvers::DecisionVariableScalar>::value,
                "The input should be a DecisionVariableMatrix object");
  Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
      double_matrix(decision_variable_matrix.rows(),
                    decision_variable_matrix.cols());
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      double_matrix(i, j) = decision_variable_matrix(i, j).value();
    }
  }
  return double_matrix;
}

/**
 * Determines if a DecisionVariableMatrix object contains a variable with
 * given @p index.
 */
template <typename Derived>
bool DecisionVariableMatrixContainsIndex(
    const Eigen::MatrixBase<Derived>& decision_variable_matrix, size_t index) {
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      if (decision_variable_matrix(i, j).index() == index) {
        return true;
      }
    }
  }
  return false;
}
}  // end namespace drake
