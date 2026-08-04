#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "drake/common/fmt.h"

namespace drake {
namespace solvers {

/** Some options can be applied to not one solver, but many solvers (for
example, many solvers support printing out the progress in each iteration).
CommonSolverOption contain the names of these supported options. The user can
use these options as "key" in SolverOption::SetOption(). If the solver doesn't
support the option, the option is ignored. */
enum class CommonSolverOption {
  /** Many solvers support printing the progress of each iteration to a file.
  The user can call SolverOptions::SetOption(kPrintFileName, "filename.log")
  to enable this. To disable, set the option to the empty string `""`, which
  indicates that no file should be written. */
  kPrintFileName,

  /** Many solvers support printing the progress of each iteration to the
  console. The user can call `SolverOptions::SetOption(kPrintToConsole, 1)`
  to enable this, or use `0` to turn off printing to the console. */
  kPrintToConsole,

  /** Some solvers support writing a standalone (e.g., it does not depend on
  Drake) minimal reproduction of the problem to a file. This is especially
  useful for sending bug reports upstream to the developers of the solver.
  The user can call
  `SolverOptions::SetOption(kStandaloneReproductionFileName, "filename.txt")`
  to enable this. To disable, set the option to the empty string `""`, which
  indicates that no file should be written. */
  kStandaloneReproductionFileName,

  /** Some solvers are multi-threaded. The user can request the maximum number
  of threads used by the solver with this `int` option. When not set, the value
  defaults to Parallelism.Max().num_threads(), which can be controlled via the
  \ref drake::Parallelism "DRAKE_NUM_THREADS" environment variable.
  @pre The number of threads must be greater than 0.
  @note Setting this value higher than the actual hardware concurrency may
  result in a degraded performance. It is recommended to set this value lower
  than or equal to Parallelism.Max().num_threads().
  @note A solver may choose to use fewer threads than the value specified.
  @note This options does NOT disable multi-threading in BLAS/LAPACK which is
  used by many solvers under the hood. Therefore, some internal operations of
  the solvers may still be multi-core. */
  kMaxThreads,
};

/** Returns the short, unadorned name of the option, e.g., `kPrintFileName`. */
std::string_view to_string(CommonSolverOption);

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
