#pragma once
#include <ostream>
#include <string>

#include "drake/common/symbolic_variable_cell.h"

namespace drake {
namespace symbolic {

/** Represents an indeterminate variable. */
class Indeterminate : public VariableCell {
 public:
  /** Constructs an indeterminate variable with name. */
  explicit Indeterminate(const std::string& name);

  /** Checks equality. */
  bool equal_to(const VariableCell& var) const override;
  /** Checks total ordering. */
  bool less(const VariableCell& var) const override;
  /** Outputs its string representation to @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

}  // namespace symbolic
}  // namespace drake
