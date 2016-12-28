#pragma once

#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Represents a symbolic variable. */
class Variable {
 public:
  /** Default constructor. This is needed to have Eigen::Matrix<Variable>. The
      objects created by the default constructor share the same ID, zero. As a
      result, they all are identified as a single variable by equality operator
      (==). They all have the same hash value as well.
   */
  Variable() : id_{0}, name_{std::string()} {}

  /** Constructs a variable with a string . */
  explicit Variable(const std::string& name);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& v) = default;

  /** Copy-construct a set from an lvalue. */
  Variable(const Variable& v) = default;

  /** Move-assign. */
  Variable& operator=(Variable&& v) = default;

  /** Copy-assign. */
  Variable& operator=(const Variable& v) = default;

  size_t get_id() const;
  size_t get_hash() const { return std::hash<size_t>{}(id_); }
  std::string get_name() const;
  std::string to_string() const;

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static size_t get_next_id();
  size_t id_{};       // Unique identifier.
  std::string name_;  // Name of variable.
};

/// Compare two variables based on their ID values
bool operator<(const Variable& lhs, const Variable& rhs);

/// Check equality
bool operator==(const Variable& lhs, const Variable& rhs);

}  // namespace symbolic

/** Computes the hash value of a symbolic variable. */
template <>
struct hash_value<symbolic::Variable> {
  size_t operator()(const symbolic::Variable& v) const { return v.get_hash(); }
};
}  // namespace drake

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
