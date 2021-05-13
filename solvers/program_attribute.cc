#include "drake/solvers/program_attribute.h"

#include <algorithm>
#include <deque>
#include <sstream>
#include <vector>

#include <fmt/format.h>

namespace drake {
namespace solvers {

bool AreRequiredAttributesSupported(const ProgramAttributes& required,
                                    const ProgramAttributes& supported,
                                    std::string* error_message) {
  // Quick short-circuit if we're guaranteed to fail.
  if ((error_message == nullptr) && (required.size() > supported.size())) {
    return false;
  }

  // Check required vs supported.  If a mismatch is found and we don't need to
  // produce an error message, then we can bail immediately.  Otherwise, tally
  // any unsupported attributes to populate the error_message at the end.
  std::vector<ProgramAttribute> unsupported;
  for (const auto& attribute : required) {
    if (supported.count(attribute) == 0) {
      if (error_message == nullptr) {
        return false;
      } else {
        unsupported.push_back(attribute);
      }
    }
  }
  if (error_message == nullptr) {
    return true;
  }

  // We need to produce an error message.  If nothing was missing, that's easy.
  error_message->clear();
  if (unsupported.empty()) {
    return true;
  }

  // Otherwise, we'll set the error message, i.e.,
  // "a FooCost was declared but is not supported" or
  // "a FooCost and BarCost were declared but are not supported" or
  // "a FooCost, BarCost, and QuuxCost were declared but are not supported".
  std::sort(unsupported.begin(), unsupported.end());
  const int size = unsupported.size();
  std::string baddies;
  for (int i = 0; i < size; ++i) {
    if (i >= 1) {
      if (size == 2) {
        baddies += " and ";
      } else if (i == (size - 1)) {
        baddies += ", and ";
      } else {
        baddies += ", ";
      }
    }
    baddies += to_string(unsupported[i]);
  }
  *error_message = fmt::format(
      (size == 1) ?
          "a {} was declared but is not supported" :
          "a {} were declared but are not supported",
      baddies);
  return false;
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
