#pragma once

#include <iostream>

namespace drake {
namespace solvers {
/**
 * Some options can be applied to not one solver, but many solvers (for example,
 * many solvers support print out the progress in each iteration).
 * DrakeSolverOptions contain the names of these supported options.
 * The user can use these options as "key" in SolverOption::SetOption().
 */
enum class DrakeSolverOption {
  kPrintFileName,   ///< Many solvers support printing the progress of each
                    ///< iteration to a file. The user can call
                    ///< SolverOptions::SetOption(kPrintFileName, file_name)
                    ///< where file_name is a string.
  kPrintToConsole,  ///< Many solvers support printing the progress of each
                    ///< iteration to the console, the user can call
                    ///< SolverOptions::SetOption(kPrintToConsole, 1) to turn
                    ///< on printing to the console, or
                    ///< SolverOptions::SetOption(kPrintToConsole, 0) to
                    ///< turn off printing to the console.
};

std::ostream& operator<<(std::ostream& os,
                         DrakeSolverOption drake_solver_option);
}  // namespace solvers
}  // namespace drake
