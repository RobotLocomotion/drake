#include "drake/solvers/common_solver_option.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

std::string_view to_string(CommonSolverOption common_solver_option) {
  switch (common_solver_option) {
    case CommonSolverOption::kPrintFileName:
      return "kPrintFileName";
    case CommonSolverOption::kPrintToConsole:
      return "kPrintToConsole";
    case CommonSolverOption::kStandaloneReproductionFileName:
      return "kStandaloneReproductionFileName";
    case CommonSolverOption::kMaxThreads:
      return "kMaxThreads";
  }
  DRAKE_UNREACHABLE();
}

}  // namespace solvers
}  // namespace drake
