#include "drake/solvers/solve.h"

#include <memory>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

#include <thread>
#include <future>



namespace drake {
namespace solvers {
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  drake::log()->debug("solvers::Solve will use {}", solver_id);
  std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
  const Eigen::VectorXd initial_guess_xd = initial_guess;
  return Solve(prog, initial_guess_xd, {});
}

MathematicalProgramResult Solve(const MathematicalProgram& prog) {
  return Solve(prog, {}, {});
}

namespace {
// Checks if a future has completed execution.
// This function is taken from monte_carlo.cc. It will be used in the "thread
// pool" implementation (which doesn't use the openMP).
template<typename T>
bool IsFutureReady(const std::future<T> &future) {
  // future.wait_for() is the only method to check the status of a future
  // without waiting for it to complete.
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
};
} // namespace

std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<MathematicalProgram>& prog_list,
    const int num_threads, bool verbose) {
   std::vector<MathematicalProgramResult> ret(
      prog_list.size());

   auto solve_prog_i = [&ret, &prog_list](int i)
   {
     ret.at(i) = Solve(prog_list.at(i), {}, {});
     return i;
   };

   const int num_threads_actual =
      num_threads > 0
          ? num_threads
          : std::min(static_cast<int>(std::thread::hardware_concurrency()),
                     static_cast<int>(prog_list.size()));

   // We implement the "thread pool" idea here, by following
  // MonteCarloSimulationParallel class. This implementation doesn't use openMP
  // library.
  std::list<std::future<int>> active_operations;
  // Keep track of how many progs have been dispatched already.
  int progs_dispatched = 0;

  while ((active_operations.size() > 0 ||
          (progs_dispatched < static_cast<int>(prog_list.size()) ))) {
    // Check for completed operations.
    for (auto operation = active_operations.begin();
         operation != active_operations.end();) {
      if (IsFutureReady(*operation)) {
        // This call to future.get() is necessary to propagate any exception
        // thrown during SOS setup/solve.
        const int prog_count = operation->get();
        if (verbose) {
          drake::log()->info("prog {}/{} completed", prog_count,
                             prog_list.size());
        }
        // Erase returned iterator to the next node in the list.
        operation = active_operations.erase(operation);
      } else {
        // Advance to next node in the list.
        ++operation;
      }
    }

    // Dispatch new prog.
    while (static_cast<int>(active_operations.size()) < num_threads_actual &&
           progs_dispatched < static_cast<int>(prog_list.size())) {
      active_operations.emplace_back(std::async(
          std::launch::async, std::move(solve_prog_i), progs_dispatched));
      if (verbose) {
        drake::log()->info("prog {}/{} dispatched", progs_dispatched,
                           prog_list.size());
      }
      ++progs_dispatched;
    }

    // Wait a bit before checking for completion.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return ret;
}
}  // namespace solvers
}  // namespace drake
