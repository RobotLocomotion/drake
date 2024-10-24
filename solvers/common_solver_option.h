#pragma once

#include <optional>
#include <string>
#include <string_view>

// Remove this include on 2024-09-01 upon completion of deprecation.
#include <ostream>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"

namespace drake {
namespace solvers {

/** Some options can be applied to not one solver, but many solvers (for
example, many solvers support printing out the progress in each iteration).
CommonSolverOption contain the names of these supported options. The user can
use these options as "key" in SolverOption::SetOption(). */
enum class CommonSolverOption {
  /** Many solvers support printing the progress of each iteration to a file.
  The user can call SolverOptions::SetOption(kPrintFileName, file_name) where
  file_name is a string. If the user doesn't want to print to a file, then use
  SolverOptions::SetOption(kPrintFileName, ""), where the empty string `""`
  indicates no print. */
  kPrintFileName,

  /** Many solvers support printing the progress of each iteration to the
  console, the user can call `SolverOptions::SetOption(kPrintToConsole, 1)` to
  turn on printing to the console, or use `0` to turn off printing to the
  console. */
  kPrintToConsole,
  /** Some of our solver interfaces support writing a standalone (e.g. it does
   * not depend on Drake) minimal reproduction of the problem to a file. This is
   * especially useful for sending bug reports upstream to the developers of the
   * solver. To enable this, use e.g.
   * SolverOptions::SetOption(kStandaloneReproductionFileName,
   * "reproduction.txt"). To disable, use
   * SolverOptions::SetOption(kStandaloneReproductionFileName, ""), where the
   * empty string "" indicates that no file should be written.
   */
  kStandaloneReproductionFileName,
  /** Many solvers are multi-threaded. The user can request the maximum number
   * of threads used by the solver with this `int` option. When not set, the
   * value defaults to Parallelism.Max().num_threads(), which can be controlled
   * via the \ref drake::Parallelism "DRAKE_NUM_THREADS" environment variable.
   *
   * @pre The number of threads must be greater than 0.
   * @note Setting this value higher than the actual hardware concurrency may
   * result in a degraded performance. It is recommended to set this value lower
   * than or equal to Parallelism.Max().num_threads().
   * @note A solver may choose to use fewer threads than the value specified.
   * @note This options does NOT disable multi-threading in BLAS/LAPACK which is
   * used by many solvers under the hood. Therefore, some internal operations of
   * the solvers may still be multi-core.
   */
  kMaxThreads,
};

/** Returns the short, unadorned name of the option, e.g., `kPrintFileName`. */
std::string_view to_string(CommonSolverOption);

DRAKE_DEPRECATED("2024-09-01", "Use options.to_string(), instead.")
std::ostream& operator<<(std::ostream&, CommonSolverOption);

namespace internal {

/* Aggregated values for CommonSolverOption, for Drake-internal use only. */
struct CommonSolverOptionValues {
  std::string print_file_name;
  bool print_to_console{false};
  std::string standalone_reproduction_file_name;
  std::optional<int> max_threads;
};

}  // namespace internal
}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, CommonSolverOption, x,
                   ::drake::solvers::to_string(x))
