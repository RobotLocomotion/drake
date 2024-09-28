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

namespace {
using SolverOptionValue = std::variant<const std::vector<const SolverOptions*>*,
                                       const std::optional<SolverOptions>*>;
std::vector<MathematicalProgramResult> SolveInParallelImpl(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const SolverOptionValue solver_options,
    const std::vector<std::optional<SolverId>>* solver_ids,
    const Parallelism parallelism, bool dynamic_schedule) {
  std::vector<std::optional<MathematicalProgramResult>> results_parallel{
      progs.size(), std::nullopt};

  std::unordered_map<SolverId, std::unique_ptr<SolverInterface>> solvers;
  for (int i = 0; i < ssize(progs); ++i) {
    // Identity the best solver for the program and store it in the map of
    // available solvers.
    const SolverId solver_id =
        solver_ids->at(i).value_or(ChooseBestSolver(*(progs.at(i))));
    if (!solvers.contains(solver_id)) {
      solvers.insert({solver_id, MakeSolver(solver_id)});
    }
    DRAKE_THROW_UNLESS(
        solvers.at(solver_id)->AreProgramAttributesSatisfied(*(progs.at(i))));
  }

  // Extracts the options from the variant and sets the maximum thread number to
  // 1 if set_max_threads_to_one is true.
  auto GetOptions = [&solver_options](const int64_t i,
                                      bool set_max_threads_to_one) {
    SolverOptions options{};
    int variant_index = solver_options.index();
    if (variant_index == 0) {
      const SolverOptions* maybe_options =
          std::get<const std::vector<const SolverOptions*>*>(solver_options)
              ->at(i);
      if (maybe_options != nullptr) {
        options = *maybe_options;
      }
    } else {
      const std::optional<SolverOptions> maybe_options =
          *std::get<const std::optional<SolverOptions>*>(solver_options);
      if (maybe_options.has_value()) {
        options = maybe_options.value();
      }
    }
    // Copy the solver options and make sure that they solve with only 1
    // thread. This is achieved by setting the
    // CommonSolverOption.kMaxThreads to 1. If the solver options
    // explicitly specify to use more threads using a solver specific
    // option, then more threads than 1 may be used.
    if (set_max_threads_to_one) {
      options.SetOption(CommonSolverOption::kMaxThreads, 1);
    }
    return options;
  };

  auto DoSolveParallel = [&](const int thread_num, const int64_t i) {
    unused(thread_num);

    if (!progs[i]->IsThreadSafe()) {
      // If this program is not thread safe, exit after identifying the
      // necessary solver and solve it later.
      return;
    }

    // Convert the initial guess into the requisite optional.
    std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
    if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
      initial_guess = *(initial_guesses->at(i));
    }

    const SolverOptions options = GetOptions(i, true /* solving_in_parallel */);

    // Solve the program.
    MathematicalProgramResult result_local;
    // Identity the best solver for the program and store it in the map of
    // available solvers.
    const SolverId solver_id =
        solver_ids->at(i).value_or(ChooseBestSolver(*(progs.at(i))));
    solvers.at(solver_id)->Solve(*(progs.at(i)), initial_guess, options,
                                 &result_local);
    results_parallel[i] = std::move(result_local);
  };

  if (dynamic_schedule) {
    DynamicParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()),
                                0, ssize(progs), DoSolveParallel,
                                ParallelForBackend::BEST_AVAILABLE);
  } else {
    StaticParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()),
                               0, ssize(progs), DoSolveParallel,
                               ParallelForBackend::BEST_AVAILABLE);
  }

  // Now finish solving the programs which cannot be solved in parallel and
  // write the final results to the results vector.
  std::vector<MathematicalProgramResult> results(progs.size());
  for (int i = 0; i < ssize(progs); ++i) {
    if (results_parallel[i] == std::nullopt) {
      // Convert the initial guess into the requisite optional.
      std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
      if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
        initial_guess = *(initial_guesses->at(i));
      }

      const SolverOptions options =
          GetOptions(i, false /* solving_in_parallel */);

      // The best solver was identified and constructed in the parallel phase,
      // so we can just select the best solver from the solver map.
      const SolverId solver_id =
          solver_ids->at(i).value_or(ChooseBestSolver(*(progs.at(i))));
      solvers.at(solver_id)->Solve(*(progs.at(i)), initial_guess, options,
                                   &(results.at(i)));
    } else {
      results[i] = *results_parallel[i];
    }
  }
  return results;
}

}  // namespace

std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::vector<const SolverOptions*>* solver_options,
    const std::vector<std::optional<SolverId>>* solvers,
    const Parallelism parallelism, bool dynamic_schedule) {
  DRAKE_THROW_UNLESS(std::all_of(progs.begin(), progs.end(), [](auto prog) {
    return prog != nullptr;
  }));
  if (initial_guesses != nullptr) {
    DRAKE_THROW_UNLESS(progs.size() == initial_guesses->size());
  }
  if (solver_options != nullptr) {
    DRAKE_THROW_UNLESS(progs.size() == solver_options->size());
  }

  if (solvers != nullptr) {
    DRAKE_THROW_UNLESS(progs.size() == solvers->size());
  }

  return SolveInParallelImpl(progs, initial_guesses, solver_options, solvers,
                             parallelism, dynamic_schedule);
}

std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::optional<SolverOptions>& solver_options,
    const std::optional<SolverId>& solver_id, const Parallelism parallelism,
    bool dynamic_schedule) {
  DRAKE_THROW_UNLESS(std::all_of(progs.begin(), progs.end(), [](auto prog) {
    return prog != nullptr;
  }));
  if (initial_guesses != nullptr) {
    DRAKE_THROW_UNLESS(progs.size() == initial_guesses->size());
  }

  std::vector<std::optional<SolverId>> solver_ids{progs.size(), solver_id};
  return SolveInParallelImpl(progs, initial_guesses, &solver_options,
                             &solver_ids, parallelism, dynamic_schedule);
}

}  // namespace solvers
}  // namespace drake
