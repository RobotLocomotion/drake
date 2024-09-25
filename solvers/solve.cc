#include "drake/solvers/solve.h"

#include <memory>
#include <utility>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/get_program_type.h"
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
// Returns the initial guess and solver options stored at the given index if
// avaible otherwise std::nullopt. The returned solver option has
// CommonSolverOption::kMaxThreads set to 1 regardless of the value of
// solver_options[i].CommonSolverOption::kMaxThreads since the return of this
// will be used by SolveInParallel.
std::pair<std::optional<Eigen::VectorXd>, SolverOptions>
GetInitialGuessAndSolveOptionsAtIndexForParallel(
    int index, const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::vector<const SolverOptions*>* solver_options) {
  SolverOptions options;
  if (solver_options->at(index)) {
    options = *(solver_options->at(index));
  }
  // Copy the solver options and make sure that they solve with only 1
  // thread. This is achieved by setting the
  // CommonSolverOption.kMaxThreads to 1. If the solver options
  // explicitly specify to use more threads using a solver specific
  // option, then more threads than 1 may be used.
  options.SetOption(CommonSolverOption::kMaxThreads, 1);
  std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
  if (initial_guesses->at(index) != nullptr) {
    initial_guess = *(initial_guesses->at(index));
  }
  return std::make_pair(initial_guess, options);
}

// Returns the initial guess and solver options stored at the given index if
// avaible otherwise std::nullopt. The returned solver option has
// CommonSolverOption::kMaxThreads set to 1 regardless of the value of
// solver_options[i].CommonSolverOption::kMaxThreads since the return of this
// will be used by SolveInParallel.
std::pair<std::optional<Eigen::VectorXd>, SolverOptions>
GetInitialGuessAndSolveOptionsAtIndexForParallel(
    int index, const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::optional<SolverOptions>& solver_options) {
  SolverOptions options;
  if (solver_options.has_value()) {
    options = solver_options.value();
  }
  // Copy the solver options and make sure that they solve with only 1
  // thread. This is achieved by setting the
  // CommonSolverOption.kMaxThreads to 1. If the solver options
  // explicitly specify to use more threads using a solver specific
  // option, then more threads than 1 may be used.
  options.SetOption(CommonSolverOption::kMaxThreads, 1);
  std::optional<Eigen::VectorXd> initial_guess{std::nullopt};
  if (initial_guesses->at(index) != nullptr) {
    initial_guess = *(initial_guesses->at(index));
  }
  return std::make_pair(initial_guess, options);
}

template <typename T,
          typename = std::enable_if_t<
              std::is_same_v<T, const std::vector<const SolverOptions*>*> ||
              std::is_same_v<T, const std::optional<SolverOptions>&>>>
std::vector<MathematicalProgramResult> SolveInParallelImpl(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const T solver_options, const std::vector<const SolverInterface*>* solvers,
    bool use_thread_index_for_solver, const Parallelism parallelism,
    bool dynamic_schedule) {
  std::vector<std::optional<MathematicalProgramResult>> results_parallel{
      progs.size(), std::nullopt};

  auto DoSolveParallel = [&](const int thread_num, const int64_t i) {
    unused(thread_num);
    if (!progs[i]->IsThreadSafe()) {
      results_parallel[i] = std::nullopt;
    }
    auto [initial_guess, options] =
        GetInitialGuessAndSolveOptionsAtIndexForParallel(i, initial_guesses,
                                                         solver_options);

    if (solvers == nullptr || solvers->at(i) == nullptr) {
      results_parallel[i] = Solve(*(progs.at(i)), initial_guess, options);
    } else {
      MathematicalProgramResult* result_local{nullptr};
      const int solver_index = use_thread_index_for_solver ? thread_num : i;
      solvers->at(solver_index)
          ->Solve(*(progs.at(i)), initial_guess, options, result_local);
      results_parallel[i] = std::move(*result_local);
    }
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
      auto [initial_guess, options] =
          GetInitialGuessAndSolveOptionsAtIndexForParallel(i, initial_guesses,
                                                           solver_options);
      if (solvers == nullptr || solvers->at(i) == nullptr) {
        results[i] = Solve(*(progs.at(i)), initial_guess, options);
      } else {
        solvers->at(i)->Solve(*(progs.at(i)), initial_guess, options,
                              &(results.at(i)));
      }
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
    const std::vector<const SolverInterface*>* solvers,
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
    for (int i = 0; i < ssize(progs); ++i) {
      DRAKE_THROW_UNLESS(
          solvers->at(i) == nullptr ||
          solvers->at(i)->AreProgramAttributesSatisfied(*(progs.at(i))));
    }
  }

  return SolveInParallelImpl<const std::vector<const SolverOptions*>*>(
      progs, initial_guesses, solver_options, solvers,
      false, /* use_thread_index_for_solver is false since a unique solver is
               used for every program */
      parallelism, dynamic_schedule);
}

// std::vector<MathematicalProgramResult> SolveInParallel(
//    const std::vector<const MathematicalProgram*>& progs,
//    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
//    const std::vector<const SolverOptions*>* solver_options = nullptr,
//    const std::optional<SolverId>& solver_id = std::nullopt,
//    const Parallelism parallelism = Parallelism::Max(),
//    bool dynamic_schedule = true) {
//  DRAKE_THROW_UNLESS(std::all_of(progs.begin(), progs.end(), [](auto prog) {
//    return prog != nullptr;
//  }));
//  if (initial_guesses != nullptr) {
//    DRAKE_THROW_UNLESS(progs.size() == initial_guesses->size());
//  }
//  if (solver_options != nullptr) {
//    DRAKE_THROW_UNLESS(progs.size() == solver_options->size());
//  }
//
//  std::vector<std::unique_ptr<SolverInterface>> solvers;
//  // This is used for passing to SolveInParallelImpl. This should not outlive
//  // solvers.
//  std::vector<const SolverInterface*> solvers_bare_pointer;
//  solvers.reserve(parallelism.num_threads());
//  solvers_bare_pointer.reserve(parallelism.num_threads());
//  for (int i = 0; i < parallelism.num_threads(); ++i) {
//    solvers.push_back(MakeSolver(solver_id.value_or(ChooseBestSolver(progs))));
//    solvers_bare_pointer.push_back(solvers.back().get());
//  }
//  return SolveInParallelImpl<const std::optional<SolverId>>(
//      progs, initial_guesses, solver_options, solvers,
//      true, /* use_thread_index_for_solver is false since
//               a unique solver is used in every thread */
//      parallelism, dynamic_schedule);
//}

 std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
    const std::optional<SolverOptions>& solver_options,
    const std::optional<SolverId>& solver_id = std::nullopt,
    const Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = true) {
  DRAKE_THROW_UNLESS(std::all_of(progs.begin(), progs.end(), [](auto prog) {
    return prog != nullptr;
  }));
  if (initial_guesses != nullptr) {
    DRAKE_THROW_UNLESS(progs.size() == initial_guesses->size());
  }

  std::vector<std::unique_ptr<SolverInterface>> solvers;
  // This is used for passing to SolveInParallelImpl. This should not outlive
  // solvers.
  std::vector<const SolverInterface*> solvers_bare_pointer;
  solvers.reserve(parallelism.num_threads());
  solvers_bare_pointer.reserve(parallelism.num_threads());
  for (int i = 0; i < parallelism.num_threads(); ++i) {
    solvers.push_back(MakeSolver(solver_id.value_or(ChooseBestSolver(progs))));
    solvers_bare_pointer.push_back(solvers.back().get());
  }
  return SolveInParallelImpl(progs, initial_guesses, solver_options, solvers,
                             true, /* use_thread_index_for_solver is false
                             since
                                      a unique solver is used in every thread
                                      */
                             parallelism, dynamic_schedule);
}

}  // namespace solvers
}  // namespace drake
