#pragma once

namespace drake {
namespace solvers {

enum SolutionResult {
    kSolutionFound = 0,
    kInvalidInput = -1,
    kInfeasibleConstraints = -2,
    kUnknownError = -3,
  };

}  // namespace solvers
}  // namespace drake
