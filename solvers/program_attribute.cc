#include "drake/solvers/program_attribute.h"

#include <algorithm>
#include <deque>
#include <sstream>

namespace drake {
namespace solvers {

bool AreRequiredAttributesSupported(const ProgramAttributes& required,
                                    const ProgramAttributes& supported) {
  if (required.size() > supported.size()) {
    return false;
  }
  for (const auto& attribute : required) {
    if (supported.find(attribute) == supported.end()) {
      return false;
    }
  }
  return true;
}

std::string to_string(const ProgramAttribute& attr) {
  switch (attr) {
    case ProgramAttribute::kGenericCost:
      return "GenericCost";
    case ProgramAttribute::kGenericConstraint:
      return "GenericConstraint";
    case ProgramAttribute::kQuadraticCost:
      return "QuadraticCost";
    case ProgramAttribute::kQuadraticConstraint:
      return "QuadraticConstraint";
    case ProgramAttribute::kLinearCost:
      return "LinearCost";
    case ProgramAttribute::kLinearConstraint:
      return "LinearConstraint";
    case ProgramAttribute::kLinearEqualityConstraint:
      return "LinearEqualityConstraint";
    case ProgramAttribute::kLinearComplementarityConstraint:
      return "LinearComplementarityConstraint";
    case ProgramAttribute::kLorentzConeConstraint:
      return "LorentzConeConstraint";
    case ProgramAttribute::kRotatedLorentzConeConstraint:
      return "RotatedLorentzConeConstraint";
    case ProgramAttribute::kPositiveSemidefiniteConstraint:
      return "PositiveSemidefiniteConstraint";
    case ProgramAttribute::kExponentialConeConstraint:
      return "ExponentialConeConstraint";
    case ProgramAttribute::kBinaryVariable:
      return "BinaryVariable";
    case ProgramAttribute::kCallback:
      return "Callback";
  }
  DRAKE_UNREACHABLE();
}

std::ostream& operator<<(std::ostream& os, const ProgramAttribute& attr) {
  os << to_string(attr);
  return os;
}

std::string to_string(const ProgramAttributes& attrs) {
  std::ostringstream result;
  result << attrs;
  return result.str();
}

std::ostream& operator<<(std::ostream& os, const ProgramAttributes& attrs) {
  std::deque<ProgramAttribute> sorted(attrs.begin(), attrs.end());
  std::sort(sorted.begin(), sorted.end());
  os << "{ProgramAttributes: ";
  if (sorted.empty()) {
    os << "empty";
  } else {
    os << sorted.front();
    sorted.pop_front();
    for (const auto& attr : sorted) {
      os << ", " << attr;
    }
  }
  os << "}";
  return os;
}

}  // namespace solvers
}  // namespace drake
