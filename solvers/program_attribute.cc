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
                                    std::string* unsupported_message) {
  // Quick short-circuit if we're guaranteed to fail.
  if ((required.size() > supported.size())
      && (unsupported_message == nullptr)) {
    return false;
  }

  // Check required vs supported.  If a mismatch is found and we don't need to
  // produce a descriptive message, then we can bail immediately.  Otherwise,
  // tally any unsupported attributes to populate the message at the end.
  std::vector<ProgramAttribute> unsupported_enums;
  for (const auto& attribute : required) {
    if (supported.count(attribute) == 0) {
      if (unsupported_message == nullptr) {
        return false;
      } else {
        unsupported_enums.push_back(attribute);
      }
    }
  }

  // If nothing was missing, then we're all done.
  if (unsupported_enums.empty()) {
    if (unsupported_message != nullptr) {
      unsupported_message->clear();
    }
    return true;
  }

  // We need to produce an error message, i.e.,
  // "a FooCost was declared but is not supported" or
  // "a FooCost and BarCost were declared but are not supported" or
  // "a FooCost, BarCost, and QuuxCost were declared but are not supported".
  std::sort(unsupported_enums.begin(), unsupported_enums.end());
  const int size = unsupported_enums.size();
  std::string noun_phrase;
  for (int i = 0; i < size; ++i) {
    if (i >= 1) {
      if (size == 2) {
        noun_phrase += " and ";
      } else if (i == (size - 1)) {
        noun_phrase += ", and ";
      } else {
        noun_phrase += ", ";
      }
    }
    noun_phrase += to_string(unsupported_enums[i]);
  }
  *unsupported_message = fmt::format(
      (size == 1) ?
          "a {} was declared but is not supported" :
          "a {} were declared but are not supported",
      noun_phrase);
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
    case ProgramAttribute::kL2NormCost:
      return "L2NormCost";
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

std::string to_string(const ProgramType& program_type) {
  switch (program_type) {
    case ProgramType::kLP:
      return "linear programming";
    case ProgramType::kQP:
      return "quadratic programming";
    case ProgramType::kSOCP:
      return "second order cone programming";
    case ProgramType::kSDP:
      return "semidefinite programming";
    case ProgramType::kGP:
      return "geometric programming";
    case ProgramType::kCGP:
      return "conic geometric programming";
    case ProgramType::kMILP:
      return "mixed-integer linear programming";
    case ProgramType::kMIQP:
      return "mixed-integer quadratic programming";
    case ProgramType::kMISOCP:
      return "mixed-integer second order cone programming";
    case ProgramType::kMISDP:
      return "mixed-integer semidefinite programming";
    case ProgramType::kQuadraticCostConicConstraint:
      return "conic-constrained quadratic programming";
    case ProgramType::kNLP:
      return "nonlinear programming";
    case ProgramType::kLCP:
      return "linear complementarity programming";
    case ProgramType::kUnknown:
      return "uncategorized mathematical programming type";
  }
  DRAKE_UNREACHABLE();
}

std::ostream& operator<<(std::ostream& os, const ProgramType& program_type) {
  os << to_string(program_type);
  return os;
}

}  // namespace solvers
}  // namespace drake
