#include "drake/solvers/drake_solver_options.h"

namespace drake {
namespace solvers {
std::ostream& operator<<(std::ostream& os,
                         DrakeSolverOption drake_solver_option) {
  switch (drake_solver_option) {
    case DrakeSolverOption::kPrintFileName:
      os << "print file name";
      return os;
    case DrakeSolverOption::kPrintToConsole:
      os << "print to console";
      return os;
    default:
      throw std::runtime_error("DrakeSolverOption: Should not reach here");
  }
}
}  // namespace solvers
}  // namespace drake
