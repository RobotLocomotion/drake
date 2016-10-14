#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include "drake/common/drake_export.h"

namespace drake {
namespace symbolic {

/** Represents a symbolic variable. */
class DRAKE_EXPORT Variable {
 public:
  /** Deletes default constructor. */
  Variable() = delete;

  /** Constructs a variable with a string . */
  explicit Variable(const std::string& name);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& f) = default;

  /** Copy-construct a set from an lvalue. */
  Variable(const Variable& f) = default;

  /** Move-assign (DELETED). */
  Variable& operator=(Variable&& f) = delete;

  /** Copy-assign (DELETED). */
  Variable& operator=(const Variable& f) = delete;

  size_t get_id() const;
  size_t get_hash() const { return std::hash<size_t>{}(id_); }
  std::string get_name() const;
  std::string to_string() const;

  DRAKE_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Variable& var);

 private:
  // Produces a unique ID for a variable.
  static size_t get_next_id();
  const size_t id_{};       // Unique identifier.
  const std::string name_;  // Name of variable.
};

/// Compare two variables based on their ID values
DRAKE_EXPORT bool operator<(const Variable& lhs, const Variable& rhs);

/// Check equality
DRAKE_EXPORT bool operator==(const Variable& lhs, const Variable& rhs);

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Variable>. */
template <>
struct hash<drake::symbolic::Variable> {
  size_t operator()(const drake::symbolic::Variable& v) const {
    return v.get_hash();
  }
};
}  // namespace std
