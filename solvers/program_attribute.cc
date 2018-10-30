#include "drake/solvers/program_attribute.h"

namespace drake {
namespace solvers {
bool IsSubsetOfAnotherProgramAttributes(const ProgramAttributes& subset,
                                        const ProgramAttributes& superset) {
  if (subset.size() > superset.size()) {
    return false;
  }
  for (const auto& attribute : subset) {
    if (superset.find(attribute) == superset.end()) {
      return false;
    }
  }
  return true;
}
}  // namespace solvers
}  // namespace drake
