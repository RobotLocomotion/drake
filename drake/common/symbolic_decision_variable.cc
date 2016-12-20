#include "drake/common/symbolic_decision_variable.h"

#include <ostream>
#include <string>

#include "drake/common/symbolic_variable_cell.h"

using std::string;
using std::ostream;

namespace drake {
namespace symbolic {

bool DecisionVariableScalar::operator==(
    const DecisionVariableScalar& rhs) const {
  return index_ == rhs.index();
}

std::ostream& operator<<(std::ostream& os, const DecisionVariableScalar& var) {
  return var.Display(os);
}

bool DecisionVariableScalar::equal_to(const VariableCell& var) const {
  // Variable::equal_to guarantees the following assertion.
  DRAKE_ASSERT(kind() == var.kind());
  const DecisionVariableScalar& dvs{
      static_cast<const DecisionVariableScalar&>(var)};
  return *this == dvs;
}

bool DecisionVariableScalar::less(const VariableCell& var) const {
  // Variable::less guarantees the following assertion.
  DRAKE_ASSERT(kind() == var.kind());
  const DecisionVariableScalar& dvs{
      static_cast<const DecisionVariableScalar&>(var)};
  // First check equality (based on index).
  if (*this == dvs) {
    return false;
  }
  // Performs a member-wise lexicographical comparison.
  if (name() < dvs.name()) {
    return true;
  }
  if (dvs.name() < name()) {
    return false;
  }
  if (type_ < dvs.type_) {
    return true;
  }
  if (dvs.type_ < type_) {
    return false;
  }
  if (value_ < dvs.value_) {
    return true;
  }
  if (dvs.value_ < value_) {
    return false;
  }
  return index_ < dvs.index_;
}

ostream& DecisionVariableScalar::Display(ostream& os) const {
  return os << name();
}

}  // namespace symbolic
}  // namespace drake
