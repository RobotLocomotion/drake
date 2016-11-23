#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"

namespace drake {
namespace solvers {
/**
 * This class stores the type, the name, the value, and the index of a
 * decision variable in an optimization program.
 * The DecisionVariableScalar created by MathematicalProgram should not outlive
 * its creator MathematicalProgram object.
 */
class DecisionVariableScalar {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  /**
   * This constructor creates a dummy placeholder, the value_ pointer
   * is initialized to nullptr. The usage of this function is
   *
   * @code{.cc}
   * // Creates an optimization program object
   * // with no decision variables.
   * MathematicalProgram prog;
   *
   * // Add a 2 x 1 vector containing two
   * // decision variables to the optimization program.
   * // This calls the private constructor
   * // DecisionVariableScalar(VarType type, const std::string &name, double*
   * value, size_t index)
   * DecisionVariableVector<2> x1 = prog.AddContinuousVariables<2>();
   *
   * // // Add 2 x 1 vector containing two
   * // decision variables to the optimization program.
   * // This calls the private constructor
   * // DecisionVariableScalar(VarType type, const std::string &name, double*
   * value, size_t index)
   * DecisionVariableVector<2> x2 = prog.AddContinuousVariables<2>();
   *
   * // This calls the default constructor DecisionVariableScalar(),
   * // X is not related to the optimization program prog yet.
   * DecisionVariableMatrix<2, 2> X;
   *
   * // Now X contains the decision variables from the optimization program
   * object prog.
   * // The first column of X is x1, the second column of X is x2.
   * X << x1, x2;
   * @endcode
   */
  DecisionVariableScalar()
      : type_(VarType::CONTINUOUS), name_(""), value_(nullptr), index_(0) {}
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
  DecisionVariableScalar(VarType type, const std::string& name, double* value,
                         size_t index)
      : type_(type), name_(name), value_(value), index_(index) {}

  void set_value(double new_value) { *value_ = new_value; }

  VarType type_;
  std::string name_;
  double* value_;
  size_t index_;
};
}  // namespace solvers
}  // namespace drake

namespace Eigen {

/// Eigen scalar type traits for Matrix<DecisionVariableScalar>.
template <>
struct NumTraits<drake::solvers::DecisionVariableScalar> {
  enum {
    // Our set of allowed values is discrete, and no epsilon is allowed during
    // equality comparison, so treat this as an unsigned integer type.
    IsInteger = 0,
    IsSigned = 0,
    IsComplex = 0,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };

  template <bool Vectorized>
  struct Div {
    enum { Cost = 1 };
  };

  typedef drake::solvers::DecisionVariableScalar Real;
  typedef drake::solvers::DecisionVariableScalar Nested;
  typedef drake::solvers::DecisionVariableScalar Literal;
};
}  // namespace Eigen

namespace drake {
namespace solvers {
template <Eigen::Index rows, Eigen::Index cols>
using DecisionVariableMatrix =
    Eigen::Matrix<drake::solvers::DecisionVariableScalar, rows, cols>;
template <Eigen::Index rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX =
    DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using VariableListRef = std::list<Eigen::Ref<const DecisionVariableMatrixX>>;

class VariableList {
 public:
  explicit VariableList(const VariableListRef& variable_list);

  /**
   * Return all the stored DecisionVariableMatrix.
   */
  std::list<DecisionVariableMatrixX> variables() const { return variables_; }

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
   * std::cout<<"The number of unique variables is "<<
   * var_list.num_unique_variables() << std::endl;
   *
   * @endcode
   *
   * The output is
   * <pre>
   * The number of unique variables is 4
   * </pre>
   */
  size_t num_unique_variables() const { return unique_variables_.size(); }

  /**
   * Given a list of DecisionVariableMatrix @p vars, computes the TOTAL number
   * of scalar decision variables stored in @p vars, including duplication.
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
   * std::cout<<"The size of variable list (including duplication) is "<<
   * var_list.size() << std::endl;
   *
   * @endcode
   *
   * The output is
   * <pre>
   * The size of variable list (including duplication) is 5
   * </pre>
   */
  size_t size() const { return size_; }

  /**
   * Determine if the DecisionVariableMatrix stored are all column vectors.
   */
  bool column_vectors_only() const { return column_vectors_only_; }

 private:
  std::list<DecisionVariableMatrixX> variables_;
  size_t size_;
  bool column_vectors_only_;
  std::unordered_set<size_t> unique_variables_;
};

/**
 * Given a DecisionVariableMatrix object, return the Eigen::Matrix that
 * stores the values of each decision variable.
 * @param decision_variable_matrix A DecisionVariableMatrix object.
 */
template <typename T, typename Derived>
Eigen::Matrix<T, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
DecisionVariableMatrixToValueMatrix(
    const Eigen::MatrixBase<Derived>& decision_variable_matrix) {
  Eigen::Matrix<T, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
      double_matrix(decision_variable_matrix.rows(),
                    decision_variable_matrix.cols());
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      double_matrix(i, j) =
          static_cast<T>(decision_variable_matrix(i, j).value());
    }
  }
  return double_matrix;
}

/**
 * Given a DecisionVariableMatrix object, return the Eigen::Matrix that
 * stores the values of each decision variable.
 * @param decision_variable_matrix A DecisionVariableMatrix object.
 */
template <typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
DecisionVariableMatrixToDoubleMatrix(
    const Eigen::MatrixBase<Derived>& decision_variable_matrix) {
  return DecisionVariableMatrixToValueMatrix<double, Derived>(
      decision_variable_matrix);
}

/**
 * Determine if a DecisionVariableMatrix object contains a variable with
 * given index.
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
}  // end namespace solvers
}  // end namespace drake
