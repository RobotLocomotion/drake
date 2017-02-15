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

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static Id get_next_id();
  Id id_{};           // Unique identifier.
  std::string name_;  // Name of variable.
};

/// Compare two variables based on their ID values
bool operator<(const Variable& lhs, const Variable& rhs);

/// Check equality
bool operator==(const Variable& lhs, const Variable& rhs);

}  // namespace symbolic

/** Computes the hash value of a variable. */
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
