#include "drake/common/symbolic_indeterminate.h"

#include <ostream>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variable_cell.h"

using std::string;
using std::ostream;

namespace drake {
namespace symbolic {

Indeterminate::Indeterminate(const string& name)
    : VariableCell(VariableKind::Indeterminate, name) {}

bool Indeterminate::equal_to(const VariableCell& var) const {
  DRAKE_ASSERT(kind() == var.kind());
  return name() == var.name();
}

bool Indeterminate::less(const VariableCell& var) const {
  DRAKE_ASSERT(kind() == var.kind());
  return name() < var.name();
}

ostream& Indeterminate::Display(ostream& os) const {
  os << name();
  return os;
}

}  // namespace symbolic
}  // namespace drake
