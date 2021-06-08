#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <cstddef>
#include <functional>
#include <memory>
#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Represents a symbolic variable.
 *
 * @note Expression::Evaluate and Formula::Evaluate methods take a symbolic
 * environment (Variable → double) and a random number generator. When an
 * expression or a formula includes random variables, `Evaluate` methods use the
 * random number generator to draw a number for a random variable from the given
 * distribution. Then this numeric value is used to substitute all the
 * occurrences of the corresponding random variable in an expression or a
 * formula.
 */
class Variable {
 public:
  typedef size_t Id;

  /** Supported types of symbolic variables. */
  // TODO(soonho-tri): refines the following descriptions.
  enum class Type {
    CONTINUOUS,       ///< A CONTINUOUS variable takes a `double` value.
    INTEGER,          ///< An INTEGER variable takes an `int` value.
    BINARY,           ///< A BINARY variable takes an integer value from {0, 1}.
    BOOLEAN,          ///< A BOOLEAN variable takes a `bool` value.
    RANDOM_UNIFORM,   ///< A random variable whose value will be drawn from
                      ///< uniform real distributed ∈ [0,1).
    RANDOM_GAUSSIAN,  ///< A random variable whose value will be drawn from
                      ///< mean-zero, unit-variance normal.
    RANDOM_EXPONENTIAL,  ///< A random variable whose value will be drawn from
                         ///< exponential distribution with λ=1.
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Variable)

  /** Default constructor. Constructs a dummy variable of CONTINUOUS type. This
   *  is needed to have Eigen::Matrix<Variable>. The objects created by the
   *  default constructor share the same ID, zero. As a result, they all are
   *  identified as a single variable by equality operator (==). They all have
   *  the same hash value as well.
   *
   *  It is allowed to construct a dummy variable but it should not be used to
   *  construct a symbolic expression.
   */
  Variable() : name_{std::make_shared<std::string>()} {}

  /** Constructs a default value.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit Variable(std::nullptr_t) : Variable() {}

  /** Constructs a variable with a string. If not specified, it has CONTINUOUS
   * type by default.*/
  explicit Variable(std::string name, Type type = Type::CONTINUOUS);

  /** Checks if this is a dummy variable (ID = 0) which is created by
   *  the default constructor. */
  [[nodiscard]] bool is_dummy() const { return get_id() == 0; }
  [[nodiscard]] Id get_id() const;
  [[nodiscard]] Type get_type() const;
  [[nodiscard]] std::string get_name() const;
  [[nodiscard]] std::string to_string() const;

  /// Checks the equality of two variables based on their ID values.
  [[nodiscard]] bool equal_to(const Variable& v) const {
    return get_id() == v.get_id();
  }

  /// Compares two variables based on their ID values.
  [[nodiscard]] bool less(const Variable& v) const {
    return get_id() < v.get_id();
  }

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const Variable& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.id_);
    // We do not send the type_ or name_ to the hasher, because the id_ is
    // already unique across all instances, and two Variable instances with
    // matching id_ will always have identical type_ and name_.
  }

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static Id get_next_id();
  Id id_{};  // Unique identifier.
  Type type_{Type::CONTINUOUS};

  // Variable class has shared_ptr<const string> instead of string to be
  // drake::test::IsMemcpyMovable.
  // Please check https://github.com/RobotLocomotion/drake/issues/5974
  // for more information.
  std::shared_ptr<const std::string> name_;  // Name of variable.
};

std::ostream& operator<<(std::ostream& os, Variable::Type type);

/// Creates a dynamically-sized Eigen matrix of symbolic variables.
/// @param rows The number of rows in the new matrix.
/// @param cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
/// @param type The type of variables in the matrix.
MatrixX<Variable> MakeMatrixVariable(
    int rows, int cols, const std::string& name,
    Variable::Type type = Variable::Type::CONTINUOUS);

/// Creates a dynamically-sized Eigen matrix of symbolic Boolean variables.
/// @param rows The number of rows in the new matrix.
/// @param cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
MatrixX<Variable> MakeMatrixBooleanVariable(int rows, int cols,
                                            const std::string& name);

/// Creates a dynamically-sized Eigen matrix of symbolic binary variables.
/// @param rows The number of rows in the new matrix.
/// @param cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
MatrixX<Variable> MakeMatrixBinaryVariable(int rows, int cols,
                                           const std::string& name);

/// Creates a dynamically-sized Eigen matrix of symbolic continuous variables.
/// @param rows The number of rows in the new matrix.
/// @param cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
MatrixX<Variable> MakeMatrixContinuousVariable(int rows, int cols,
                                               const std::string& name);

/// Creates a dynamically-sized Eigen matrix of symbolic integer variables.
/// @param rows The number of rows in the new matrix.
/// @param cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
MatrixX<Variable> MakeMatrixIntegerVariable(int rows, int cols,
                                            const std::string& name);

/// Creates a static-sized Eigen matrix of symbolic variables.
/// @tparam rows The number of rows in the new matrix.
/// @tparam cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
/// @param type The type of variables in the matrix.
template <int rows, int cols>
Eigen::Matrix<Variable, rows, cols> MakeMatrixVariable(
    const std::string& name, Variable::Type type = Variable::Type::CONTINUOUS) {
  Eigen::Matrix<Variable, rows, cols> m;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      m(i, j) = Variable{
          name + "(" + std::to_string(i) + ", " + std::to_string(j) + ")",
          type};
    }
  }
  return m;
}

/// Creates a static-sized Eigen matrix of symbolic Boolean variables.
/// @tparam rows The number of rows in the new matrix.
/// @tparam cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
template <int rows, int cols>
Eigen::Matrix<Variable, rows, cols> MakeMatrixBooleanVariable(
    const std::string& name) {
  return MakeMatrixVariable<rows, cols>(name, Variable::Type::BOOLEAN);
}

/// Creates a static-sized Eigen matrix of symbolic binary variables.
/// @tparam rows The number of rows in the new matrix.
/// @tparam cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
template <int rows, int cols>
Eigen::Matrix<Variable, rows, cols> MakeMatrixBinaryVariable(
    const std::string& name) {
  return MakeMatrixVariable<rows, cols>(name, Variable::Type::BINARY);
}

/// Creates a static-sized Eigen matrix of symbolic continuous variables.
/// @tparam rows The number of rows in the new matrix.
/// @tparam cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
template <int rows, int cols>
Eigen::Matrix<Variable, rows, cols> MakeMatrixContinuousVariable(
    const std::string& name) {
  return MakeMatrixVariable<rows, cols>(name, Variable::Type::CONTINUOUS);
}

/// Creates a static-sized Eigen matrix of symbolic integer variables.
/// @tparam rows The number of rows in the new matrix.
/// @tparam cols The number of cols in the new matrix.
/// @param name The common prefix for variables.
///             The (i, j)-th element will be named as `name(i, j)`.
template <int rows, int cols>
Eigen::Matrix<Variable, rows, cols> MakeMatrixIntegerVariable(
    const std::string& name) {
  return MakeMatrixVariable<rows, cols>(name, Variable::Type::INTEGER);
}

/// Creates a dynamically-sized Eigen vector of symbolic variables.
/// @param rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
/// @param type The type of variables in the vector.
VectorX<Variable> MakeVectorVariable(
    int rows, const std::string& name,
    Variable::Type type = Variable::Type::CONTINUOUS);

/// Creates a dynamically-sized Eigen vector of symbolic Boolean variables.
/// @param rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
VectorX<Variable> MakeVectorBooleanVariable(int rows, const std::string& name);

/// Creates a dynamically-sized Eigen vector of symbolic binary variables.
/// @param rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
VectorX<Variable> MakeVectorBinaryVariable(int rows, const std::string& name);

/// Creates a dynamically-sized Eigen vector of symbolic continuous variables.
/// @param rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
VectorX<Variable> MakeVectorContinuousVariable(int rows,
                                               const std::string& name);

/// Creates a dynamically-sized Eigen vector of symbolic integer variables.
/// @param rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
VectorX<Variable> MakeVectorIntegerVariable(int rows, const std::string& name);

/// Creates a static-sized Eigen vector of symbolic variables.
/// @tparam rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
/// @param type The type of variables in the vector.
template <int rows>
Eigen::Matrix<Variable, rows, 1> MakeVectorVariable(
    const std::string& name, Variable::Type type = Variable::Type::CONTINUOUS) {
  Eigen::Matrix<Variable, rows, 1> vec;
  for (int i = 0; i < rows; ++i) {
    vec[i] = Variable{name + "(" + std::to_string(i) + ")", type};
  }
  return vec;
}

/// Creates a static-sized Eigen vector of symbolic Boolean variables.
/// @tparam rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
template <int rows>
Eigen::Matrix<Variable, rows, 1> MakeVectorBooleanVariable(
    const std::string& name) {
  return MakeVectorVariable<rows>(name, Variable::Type::BOOLEAN);
}

/// Creates a static-sized Eigen vector of symbolic binary variables.
/// @tparam rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
template <int rows>
Eigen::Matrix<Variable, rows, 1> MakeVectorBinaryVariable(
    const std::string& name) {
  return MakeVectorVariable<rows>(name, Variable::Type::BINARY);
}

/// Creates a static-sized Eigen vector of symbolic continuous variables.
/// @tparam rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
template <int rows>
Eigen::Matrix<Variable, rows, 1> MakeVectorContinuousVariable(
    const std::string& name) {
  return MakeVectorVariable<rows>(name, Variable::Type::CONTINUOUS);
}

/// Creates a static-sized Eigen vector of symbolic integer variables.
/// @tparam rows The size of vector.
/// @param name The common prefix for variables.
///             The i-th element will be named as `name(i)`.
template <int rows>
Eigen::Matrix<Variable, rows, 1> MakeVectorIntegerVariable(
    const std::string& name) {
  return MakeVectorVariable<rows>(name, Variable::Type::INTEGER);
}

}  // namespace symbolic
}  // namespace drake

namespace std {

/* Provides std::hash<drake::symbolic::Variable>. */
template <>
struct hash<drake::symbolic::Variable> : public drake::DefaultHash {};

/* Provides std::less<drake::symbolic::Variable>. */
template <>
struct less<drake::symbolic::Variable> {
  bool operator()(const drake::symbolic::Variable& lhs,
                  const drake::symbolic::Variable& rhs) const {
    return lhs.less(rhs);
  }
};

/* Provides std::equal_to<drake::symbolic::Variable>. */
template <>
struct equal_to<drake::symbolic::Variable> {
  bool operator()(const drake::symbolic::Variable& lhs,
                  const drake::symbolic::Variable& rhs) const {
    return lhs.equal_to(rhs);
  }
};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Variable>.
template <>
struct NumTraits<drake::symbolic::Variable>
    : GenericNumTraits<drake::symbolic::Variable> {
  static inline int digits10() { return 0; }
};
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

namespace drake {
namespace symbolic {
/// Checks if two Eigen::Matrix<Variable> @p m1 and @p m2 are structurally
/// equal. That is, it returns true if and only if `m1(i, j)` is structurally
/// equal to `m2(i, j)` for all `i`, `j`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<is_eigen_scalar_same<DerivedA, Variable>::value &&
                              is_eigen_scalar_same<DerivedB, Variable>::value,
                        bool>
CheckStructuralEquality(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::equal_to<Variable>{}).all();
}
}  // namespace symbolic
}  // namespace drake
