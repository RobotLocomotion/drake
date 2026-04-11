#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_EXPRESSION_ALL
#error Do not include this file. Use "drake/common/symbolic/expression.h".
#endif

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"
#include "drake/common/hash.h"
#include "drake/common/reset_after_move.h"

// Remove with deprecation 2026-07-01.
#include <ostream>

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
  /** Supported types of symbolic variables. */
  enum class Type : uint8_t {
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

  /** Identifier for a symbolic variable. Variable equality is defined by
  whether their Ids are equal (i.e., ignoring Variable names). Ids are akin to
  [UUIDs](https://en.wikipedia.org/wiki/Universally_unique_identifier) because
  Ids contain a large number of random bits so are unique not only within a
  single process, but also across multiple runs of programs over time. On the
  other hand, we also guarantee that all Ids created within a single process
  have consistent behavior in their comparison (`operator<`) and hashing
  operators, so that any single program is still run-to-run consistent. */
  class Id {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Id);

    /** Constructs a "dummy" id, not associated with any variable. */
    Id() = default;

    /** Default comparison operators. */
    auto operator<=>(const Id&) const = default;

    /** Implements the @ref hash_append concept. */
    template <class HashAlgorithm>
    // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
    friend void hash_append(HashAlgorithm& hasher, const Id& item) noexcept {
      using drake::hash_append;
      // To maintain consistent hash iteration order across repeated runs of a
      // program, we must only hash the non-random part of our state. The low
      // 32 bits of lo_ don't contain any randomness. (We could also hash our
      // get_type(), but it doesn't add a meaningful amount of entropy.)
      hash_append(hasher, static_cast<uint32_t>(item.lo_));
    }

    /** Returns a string representation of this Id. */
    std::string to_string() const;

   private:
    friend class Variable;
    friend class VariableIdPythonAttorney;

    static Id Create(Type type);

    Type get_type() const {
      // We store the Type enum in the lower byte of hi_.
      return static_cast<Type>(static_cast<uint8_t>(hi_));
    }

    bool is_default() const { return lo_ == 0 && hi_ == 0; }

    // Id represents three pieces of information:
    // - type: the Variable::Type;
    // - serial number: a counter incremented for each new Variable created
    //     within this process; the first Variable created is serial number 1,
    //     the second Variable is serial number 2, etc.;
    // - nonce: a random value that's initialized once per process the first
    //     time it's needed; all variables created within the process are based
    //     on the same nonce; this helps distinguish variables created by one
    //     process from those created by another process when saving them to
    //     disk.
    //
    // For efficiency, we pack these three pieces into two member fields:
    // - the 8-bit type is stored in the low byte of hi_;
    // - the 64-bit serial number is stored in lo_, summed with the nonce in the
    //   upper 4 bytes; to recover the serial number we would need to subtract
    //   the nonce from lo_, but in practice we never need the serial number by
    //   itself;
    // - the 88-bit nonce is stored as:
    //   - 56 bits in the upper 7 bytes of hi_ and
    //   - 32 bits added to the upper 4 bytes of lo_.
    //
    // Why 88 bits for the nonce? Like a UUID, we need enough bits to avoid
    // birthday collisions. Using only 56 or 64 bits ends up with too high a
    // chance of collision. Using 88 bits is a balance between having enough
    // entropy while still allowing for consistency of our sort ordering and
    // hash code.
    //
    // Why not pack both the type and serial number into single 64-bit word,
    // leaving the other 64-bit word exclusively for nonce data? That would
    // put at risk the `type` changing if the serial number got too large.
    //
    // Note that if the serial number's value exceeds 32 bits, it will end up
    // affecting bits of lo_ where the nonce is stored. This is not a problem;
    // in effect it's just a slightly difference random nonce. Ids are still
    // distinct from each other in all ways that matter.
    //
    // Note that the lower 4 bytes of `lo_` store the serial number mod 2^32,
    // which can therefore provide a run-to-run stable hash key when an Id is
    // used in unordered containers, without being affected by the random nonce.
    // This is important for reproducibility.
    //
    // Because nearly all variables have same type (`CONTINUOUS`) and the nonce
    // is invariant within one process, hi_ will typically contain exactly the
    // same value across all Variables in this process, and therefore it's
    // important to declare lo_ first and hi_ second so that the entropy-bearing
    // member field (lo_) is the first one to be compared by operator<=>.
    uint64_t lo_{};
    uint64_t hi_{};
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Variable);

  /** Constructs a default variable of type CONTINUOUS with an `Id` of zero.
   * All default-constructed variables are considered the same variable by the
   * equality operator (==). Similarly, a moved-from variable is also identical
   * to a default-constructed variable (in both its `name` and its `Id`).
   */
  Variable() = default;

  /** Constructs a default value.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit Variable(std::nullptr_t) : Variable() {}

  /** Constructs a variable with a string. If not specified, it has CONTINUOUS
   * type by default.*/
  explicit Variable(std::string name, Type type = Type::CONTINUOUS);

  // The destructor is inlined for performance.
  ~Variable() = default;

  /** Checks if this is the variable created by the default constructor. */
  [[nodiscard]] bool is_dummy() const { return get_id().is_default(); }
  [[nodiscard]] const Id& get_id() const { return id_; }
  [[nodiscard]] Type get_type() const { return get_id().get_type(); }
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
  // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
  friend void hash_append(HashAlgorithm& hasher,
                          const Variable& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.get_id());
    // We do not send the name_ to the hasher, because the id_ is already unique
    // across all instances, so two Variable instances with matching id_ will
    // always have identical names.
  }

 private:
  friend class VariablePythonAttorney;

  // Unique identifier for this Variable. The high-order byte stores the Type.
  // See get_next_id() in the cc file for more details.
  reset_after_move<Id> id_;

  // Variable class has shared_ptr<const string> instead of string to be
  // drake::test::IsMemcpyMovable.
  // Please check https://github.com/RobotLocomotion/drake/issues/5974
  // for more information.
  std::shared_ptr<const std::string> name_;  // Name of variable.
};

DRAKE_DEPRECATED(
    "2026-07-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream& os, const Variable& var);

DRAKE_DEPRECATED(
    "2026-07-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream& os, Variable::Type type);

std::string_view to_string(const Variable::Type& type);

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

/* Provides std::hash<drake::symbolic::Variable::Id>. */
template <>
struct hash<drake::symbolic::Variable::Id> : public drake::DefaultHash {};

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
  constexpr static int digits() { return 0; }
  constexpr static int digits10() { return 0; }
  constexpr static int max_digits10() { return 0; }
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

DRAKE_FORMATTER_AS(, drake::symbolic, Variable::Id, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::symbolic, Variable, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::symbolic, Variable::Type, x,
                   drake::symbolic::to_string(x))
