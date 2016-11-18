#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Represents a symbolic variable. */
class Variable {
 public:
  /** Default constructor (DELETED). */
  Variable() = delete;

  /** Constructs a variable with a string . */
  explicit Variable(const std::string& name);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& v) = default;

  /** Copy-construct a set from an lvalue. */
  Variable(const Variable& v) = default;

  /** Move-assign (DELETED). */
  Variable& operator=(Variable&& v) = delete;

  /** Copy-assign (DELETED). */
  Variable& operator=(const Variable& v) = delete;

  size_t get_id() const;
  size_t get_hash() const { return std::hash<size_t>{}(id_); }
  std::string get_name() const;
  std::string to_string() const;

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static size_t get_next_id();
  const size_t id_{};       // Unique identifier.
  const std::string name_;  // Name of variable.
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
