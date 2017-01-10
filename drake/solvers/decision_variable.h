#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace solvers {
template <int rows, int cols>
using DecisionVariableMatrix =
    Eigen::Matrix<drake::symbolic::Variable, rows, cols>;
template <int rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX =
    DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using VariableListRef = std::list<Eigen::Ref<const DecisionVariableMatrixX>>;

/**
 * This class stores a list of DecisionVariableMatrix objects. An instance
 * of this class is going to be bound to a constraint, indicating that a
 * constraint is imposed on one or several DecisionVariableMatrix objects.
 * TODO(hongkai.dai) : replace this class with symbolic::List, or rename it as
 * using Arguments = std::vector<DecisionVariableMatrixX>;
 *
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
   * auto x = prog.NewContinuousVariables<4>();
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
  const std::unordered_set<drake::symbolic::Variable,
                           drake::hash_value<symbolic::Variable>>&
  unique_variables() const {
    return unique_variable_indices_;
  }

 private:
  std::list<DecisionVariableMatrixX> variables_{};
  size_t size_{0};
  bool column_vectors_only_{true};
  std::unordered_set<drake::symbolic::Variable,
                     drake::hash_value<symbolic::Variable>>
      unique_variable_indices_{};
};
}  // end namespace solvers
}  // end namespace drake
