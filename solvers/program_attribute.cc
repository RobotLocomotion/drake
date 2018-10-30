#include "drake/solvers/program_attribute.h"

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
}  // namespace solvers
}  // namespace drake
