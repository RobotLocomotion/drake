#pragma once

namespace drake {
namespace solvers {

enum SolutionResult {
    kSolutionFound = 0,
    kInvalidInput = -1,
    kInfeasibleConstraint = -2,
    kUnknownError = -3,
  };

}  // namespace drake
}  // namespace solvers
