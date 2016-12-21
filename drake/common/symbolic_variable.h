#pragma once

#include <cstddef>
#include <memory>
#include <ostream>
#include <string>

#include "drake/common/hash.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic variable. */
enum class VariableKind { Indeterminate, DecisionVariableScalar };

/** Total ordering between VariableKind. */
bool operator<(VariableKind k1, VariableKind k2);

class VariableCell;  // In drake/common/symbolic_variable_cell.h file.

/** Represents a symbolic variable.
 *
 * Design Rationale:
 *
 *        +-------------------------------------------+
 *        | class Variable                            |
 *        +-------------------------------------------+
 *        | const std::shared_ptr<VariableCell> ptr_; |
 *        +-------------------------------------------+
 *
 * Variafble is a simple wrapper including a shared pointer to VariableCell
 * class, which is the base-class of different kinds of symbolic variables
 * (Indeterminate and DecisionVariableScalar). See the following inheritance
 * diagram.
 *
 *                 +-------------------------+
 *                 | class VariableCell      |
 *                 +-------------------------+
 *                       ---/        \---
 *                   ---/                \--
 *   +--------------/--------+    +---------\--------------+
 *   | class                 |    | class                  |
 *   | Indeterminate         |    | DecisionVariableScalar |
 *   +-----------------------+    +------------------------+
 *
 * This design achieves the following goals:
 *
 * 1) No object slicing: Consider an example of having a set of Variables,
 *    set<Variable>. In the current design, since Variable class is holding a
 *    pointer to VariableCell, we do not have the problem of object slicing.
 *    However, if we handle VariableCell class directly, we need to have
 *    set<VariableCell *> or set<reference_wrapper<VariableCell>> to avoid
 *    object slicing. This means that a user of the set has to manage
 *    VariableCell objects in the heap or needs to worry about their lifetime.
 *
 * 2) Polymorphism: A method @c foo in Variable class simply calls @c
 *    ptr_->foo. Therefore, we still have polymorphism in effect.
 *
 */
class Variable {
 public:
  /** Default constructor (DELETED). */
  Variable() = delete;
  /** Constructs an indeterminate variable from string @p name. */
  explicit Variable(const std::string& name);
  /** Constructs a variable from @p ptr. */
  explicit Variable(const std::shared_ptr<VariableCell> ptr);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& v) = default;
  /** Copy-construct a set from an lvalue. */
  Variable(const Variable& v) = default;
  /** Move-assign (DELETED). */
  Variable& operator=(Variable&& v) = delete;
  /** Copy-assign (DELETED). */
  Variable& operator=(const Variable& v) = delete;

  /** Returns its kind. */
  VariableKind kind() const;
  /** Returns its name. */
  std::string name() const;
  /** Returns its string representation. */
  std::string to_string() const;

  /** Returns its hash value. */
  size_t hash() const;
  /** Checks equality. */
  bool equal_to(const Variable& var) const;
  /** Checks total ordering. */
  bool less(const Variable& var) const;

  friend std::ostream& operator<<(std::ostream& os, const Variable& var);

 private:
  const std::shared_ptr<VariableCell> ptr_;
};

std::ostream& operator<<(std::ostream& os, const Variable& var);

/// Checks total ordering between the two variables.
bool operator<(const Variable& lhs, const Variable& rhs);

/// Checks equality of two variables.
bool operator==(const Variable& lhs, const Variable& rhs);

}  // namespace symbolic

/** Computes the hash value of a symbolic variable. */
template <>
struct hash_value<symbolic::Variable> {
  size_t operator()(const symbolic::Variable& v) const { return v.hash(); }
};
}  // namespace drake
