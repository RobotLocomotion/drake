#pragma once

#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Represents a symbolic variable. */
class Variable {
 public:
  typedef size_t Id;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Variable)

  /** Default constructor. Constructs a dummy variable. This is needed to have
   *  Eigen::Matrix<Variable>. The objects created by the default constructor
   *  share the same ID, zero. As a result, they all are identified as a single
   *  variable by equality operator (==). They all have the same hash value as
   *  well.
   *
   *  It is allowed to construct a dummy variable but it should not be used to
   *  construct a symbolic expression.
   */
  Variable() : id_{0}, name_{std::string()} {}

  /** Constructs a variable with a string . */
  explicit Variable(const std::string& name);

  /** Checks if this is a dummy variable (ID = 0) which is created by
   *  the default constructor. */
  bool is_dummy() const { return get_id() == 0; }
  Id get_id() const;
  size_t get_hash() const { return std::hash<Id>{}(id_); }
  std::string get_name() const;
  std::string to_string() const;

  /// Checks the equality of two variables based on their ID values.
  bool equal_to(const Variable& v) const { return get_id() == v.get_id(); }

  /// Compares two variables based on their ID values.
  bool less(const Variable& v) const { return get_id() < v.get_id(); }

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static Id get_next_id();
  Id id_{};           // Unique identifier.
  std::string name_;  // Name of variable.
};
}  // namespace symbolic

/** Computes the hash value of a variable. */
template <>
struct hash_value<symbolic::Variable> {
  size_t operator()(const symbolic::Variable& v) const { return v.get_hash(); }
};

}  // namespace drake

namespace std {
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
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value &&
        std::is_same<typename DerivedA::Scalar, Variable>::value &&
        std::is_same<typename DerivedB::Scalar, Variable>::value,
    bool>::type
CheckStructuralEquality(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::equal_to<Variable>{}).all();
}
}  // namespace symbolic
}  // namespace drake
