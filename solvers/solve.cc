#include "drake/solvers/solve.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <variant>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::DynamicParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;

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

std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::vector<const SolverOptions*>* solver_options,
    const std::vector<std::optional<SolverId>>* solver_ids,
    const Parallelism parallelism, const bool dynamic_schedule) {
  DRAKE_THROW_UNLESS(std::all_of(progs.begin(), progs.end(), [](auto prog) {
    return prog != nullptr;
  }));
  if (initial_guesses != nullptr) {
    DRAKE_THROW_UNLESS(initial_guesses->size() == progs.size());
  }
  if (solver_options != nullptr) {
    DRAKE_THROW_UNLESS(solver_options->size() == progs.size());
  }
  if (solver_ids != nullptr) {
    DRAKE_THROW_UNLESS(solver_ids->size() == progs.size());
  }

  // Pre-allocate the results (i.e., don't use push_back) so that we can safely
  // write to the vector on multiple threads concurrently.
  std::vector<MathematicalProgramResult> results{progs.size()};

  // Track which results are set during the par-for loop. Some programs are not
  // thread safe so will be skipped inside the loop, and we'll need to circle
  // back and solve them serially later. (N.B. we cannot use vector<bool> here
  // because it's not thread safe.)
  std::vector<uint8_t> result_is_populated(progs.size(), 0);

  // As unique types of solvers are encountered by the threads, we will cache
  // the solvers into this data structure so that we don't have to create and
  // destroy them at every call to Solve.
  std::vector<std::unordered_map<SolverId, std::unique_ptr<SolverInterface>>>
      solvers(parallelism.num_threads());

  // Extracts the options from the variant and sets the maximum thread number to
  // 1 if set_max_threads_to_one is true.
  auto GetOptions = [&solver_options](const int64_t i,
                                      bool set_max_threads_to_one) {
    // Copy the solver options and make sure that they solve with only 1
    // thread. This is achieved by setting the
    // CommonSolverOption.kMaxThreads to 1. If the solver options
    // explicitly specify to use more threads using a solver specific
    // option, then more threads than 1 may be used.
    bool option_available =
        solver_options != nullptr && solver_options->at(i) != nullptr;
    SolverOptions options =
        option_available ? *solver_options->at(i) : SolverOptions();
    if (set_max_threads_to_one) {
      options.SetOption(CommonSolverOption::kMaxThreads, 1);
    }
    return options;
  };

  // This is the worker callback for the i'th program.
  auto solve_ith = [&](const int thread_num, const int64_t i) {
    if (!progs[i]->IsThreadSafe()) {
      // If this program is not thread safe, exit after identifying the
      // necessary solver and solve it later.
      return;
    }

    // Access the required solver.
    const SolverId solver_id =
        solver_ids == nullptr
            ? ChooseBestSolver(*(progs.at(i)))
            : solver_ids->at(i).value_or(ChooseBestSolver(*(progs.at(i))));
    if (!solvers.at(thread_num).contains(solver_id)) {
      // If this thread has not solved this type of program yet, save the solver
      // for use later.
      solvers.at(thread_num).insert({solver_id, MakeSolver(solver_id)});
    }
    const SolverInterface* solver = solvers.at(thread_num).at(solver_id).get();

    // Convert the initial guess into the requisite optional.
    std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
    if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
      initial_guess = *(initial_guesses->at(i));
    }

    const SolverOptions options = GetOptions(i, true /* solving_in_parallel */);

    // Solve the program.
    solver->Solve(*(progs.at(i)), initial_guess, options, &(results.at(i)));
    result_is_populated.at(i) = true;
  };

  // Call solve_ith in parallel for all of the progs.
  (dynamic_schedule ? &(DynamicParallelForIndexLoop<decltype(solve_ith)>)
                    : &(StaticParallelForIndexLoop<decltype(solve_ith)>))(
      DegreeOfParallelism(parallelism.num_threads()), 0, ssize(progs),
      solve_ith, ParallelForBackend::BEST_AVAILABLE);

  // Now finish solving the programs that couldn't be solved in parallel.
  for (int i = 0; i < ssize(progs); ++i) {
    if (result_is_populated.at(i)) {
      continue;
    }
    // Convert the initial guess into the requisite optional.
    std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
    if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
      initial_guess = *(initial_guesses->at(i));
    }

    const SolverOptions options =
        GetOptions(i, false /* solving_in_parallel */);

    // We will use just the solvers at the 0th index as cached solvers.
    const SolverId solver_id =
        solver_ids->at(i).value_or(ChooseBestSolver(*(progs.at(i))));
    if (!solvers.at(0).contains(solver_id)) {
      // If this thread has not solved this type of program yet, save the
      // solver for use later.
      solvers.at(0).insert({solver_id, MakeSolver(solver_id)});
    }
    solvers.at(0).at(solver_id)->Solve(*(progs.at(i)), initial_guess, options,
                                       &(results.at(i)));
  }
  return results;
}

std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::optional<SolverOptions>& solver_options,
    const std::optional<SolverId>& solver_id, const Parallelism parallelism,
    const bool dynamic_schedule) {
  // Broadcast the option and id arguments into vectors (if given).
  std::optional<std::vector<const SolverOptions*>> broadcast_options;
  std::optional<std::vector<std::optional<SolverId>>> broadcast_ids;
  if (solver_options.has_value()) {
    broadcast_options.emplace(progs.size(), &(*solver_options));
  }
  if (solver_id.has_value()) {
    broadcast_ids.emplace(progs.size(), solver_id);
  }
  // Delegate to the primary overload.
  return SolveInParallel(
      progs, initial_guesses,
      broadcast_options.has_value() ? &(*broadcast_options) : nullptr,
      broadcast_ids.has_value() ? &(*broadcast_ids) : nullptr,  // BR
      parallelism, dynamic_schedule);
}

}  // namespace solvers
}  // namespace drake
