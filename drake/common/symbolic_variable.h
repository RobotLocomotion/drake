#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
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

/** Represents a symbolic variable. */
class Variable {
 public:
  typedef size_t Id;

  /** Supported types of symbolic variables. */
  // TODO(soonho-tri): refines the following descriptions.
  enum class Type {
    CONTINUOUS,  ///< A CONTINUOUS variable takes a `double` value.
    INTEGER,     ///< An INTEGER variable takes an `int` value.
    BINARY,      ///< A BINARY variable takes an integer value from {0, 1}.
    BOOLEAN,     ///< A BOOLEAN variable takes a `bool` value.
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
  Variable()
      : id_{0},
        type_{Type::CONTINUOUS},
        name_{std::make_shared<std::string>()} {}

  /** Constructs a variable with a string. If not specified, it has CONTINUOUS
   * type by default.*/
  explicit Variable(std::string name, Type type = Type::CONTINUOUS);

  /** Checks if this is a dummy variable (ID = 0) which is created by
   *  the default constructor. */
  bool is_dummy() const { return get_id() == 0; }
  Id get_id() const;
  Type get_type() const;
  std::string get_name() const;
  std::string to_string() const;

  /// Checks the equality of two variables based on their ID values.
  bool equal_to(const Variable& v) const { return get_id() == v.get_id(); }

  /// Compares two variables based on their ID values.
  bool less(const Variable& v) const { return get_id() < v.get_id(); }

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const Variable& item) noexcept {
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

  // Variable class has shared_ptr<string> instead of string to be
  // drake::test::IsMemcpyMovable.
  // Please check https://github.com/RobotLocomotion/drake/issues/5974
  // for more information.
  std::shared_ptr<std::string> name_;  // Name of variable.
};

std::ostream& operator<<(std::ostream& os, Variable::Type type);

}  // namespace symbolic
}  // namespace drake

namespace std {

/* Provides std::hash<drake::symbolic::Variable>. */
template <> struct hash<drake::symbolic::Variable>
    : public drake::DefaultHash {};

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
typename std::enable_if<
    is_eigen_scalar_same<DerivedA, Variable>::value &&
        is_eigen_scalar_same<DerivedB, Variable>::value,
    bool>::type
CheckStructuralEquality(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::equal_to<Variable>{}).all();
}
}  // namespace symbolic
}  // namespace drake
