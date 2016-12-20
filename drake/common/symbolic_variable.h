#pragma once

#include <cstddef>
#include <memory>
#include <ostream>
#include <string>

#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic variable. */
enum class VariableKind { Indeterminate, DecisionVariable };

/** Total ordering between VariableKind. */
bool operator<(VariableKind k1, VariableKind k2);

class VariableCell;  // In drake/common/symbolic_variable_cell.h file.

/** Represents a symbolic variable. */
class Variable {
 public:
  /** Default constructor (DELETED). */
  Variable() = delete;
  explicit Variable(const std::string& name);
  explicit Variable(const std::shared_ptr<VariableCell> ptr);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& v) = default;
  /** Copy-construct a set from an lvalue. */
  Variable(const Variable& v) = default;
  /** Move-assign (DELETED). */
  Variable& operator=(Variable&& v) = delete;
  /** Copy-assign (DELETED). */
  Variable& operator=(const Variable& v) = delete;

  VariableKind kind() const;
  std::string name() const;
  std::string to_string() const;

  size_t hash() const;
  bool equal_to(const Variable& var) const;
  bool less(const Variable& var) const;

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  const std::shared_ptr<VariableCell> ptr_;
};

std::ostream& operator<<(std::ostream& os, const Variable& var);

/// Compares two variables.
bool operator<(const Variable& lhs, const Variable& rhs);

/// Checks equality of two variables.
bool operator==(const Variable& lhs, const Variable& rhs);

Variable make_indeterminate(const std::string& name);

}  // namespace symbolic

/** Computes the hash value of a symbolic variable. */
template <>
struct hash_value<symbolic::Variable> {
  size_t operator()(const symbolic::Variable& v) const { return v.hash(); }
};
}  // namespace drake
