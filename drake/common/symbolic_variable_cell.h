#pragma once

#include <cstddef>
#include <ostream>
#include <string>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {

/** This class is the abstract base-class for symbolic::Indeterminate and
 * symbolic::DecisionVariableScalar classes. */
class VariableCell {
 public:
  /** Default constructor. */
  VariableCell() = default;
  /** Constructs a variable with kind and string. */
  VariableCell(VariableKind kind, const std::string& name);
  /** Constructs a variable with kind, string, and hash. */
  VariableCell(VariableKind kind, const std::string& name, size_t hash);
  /** Move-construct a set from an rvalue. */
  VariableCell(VariableCell&& v) = default;
  /** Copy-construct a set from an lvalue. */
  VariableCell(const VariableCell& v) = default;
  /** Move-assign. */
  VariableCell& operator=(VariableCell&& v) = default;
  /** Copy-assign. */
  VariableCell& operator=(const VariableCell& v) = default;

  /** Returns its kind. */
  VariableKind kind() const;
  /** Returns its name. */
  const std::string& name() const;
  /** Returns its hash value. */
  size_t hash() const;

  /** Checks equality. */
  virtual bool equal_to(const VariableCell& var) const = 0;
  /** Checks total ordering. */
  virtual bool less(const VariableCell& var) const = 0;
  /** Outputs its string representation to @p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 private:
  VariableKind kind_{};
  std::string name_;
  size_t hash_{};
};

}  // namespace symbolic
}  // namespace drake
