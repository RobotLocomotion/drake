#pragma once
#include <cstddef>
#include <memory>
#include <ostream>
#include <string>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
class VariableCell {
 public:
  /** Default constructor. */
  VariableCell() = default;
  /** Constructs a variable with a string . */
  VariableCell(VariableKind kind, const std::string& name);
  /** Constructs a variable with a string . */
  VariableCell(VariableKind kind, const std::string& name, size_t hash);
  /** Move-construct a set from an rvalue. */
  VariableCell(VariableCell&& v) = default;
  /** Copy-construct a set from an lvalue. */
  VariableCell(const VariableCell& v) = default;
  /** Move-assign. */
  VariableCell& operator=(VariableCell&& v) = default;
  /** Copy-assign. */
  VariableCell& operator=(const VariableCell& v) = default;

  VariableKind kind() const;
  const std::string& name() const;
  size_t hash() const;

  virtual bool equal_to(const VariableCell& var) const = 0;
  virtual bool less(const VariableCell& var) const = 0;
  virtual std::ostream& Display(std::ostream& os) const = 0;

 private:
  VariableKind kind_{};
  std::string name_;
  size_t hash_{};
};

}  // namespace symbolic
}  // namespace drake
