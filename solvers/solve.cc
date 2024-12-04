#include "drake/solvers/solve.h"

#include <iostream>
#include <memory>
#include <unordered_map>
#include <utility>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
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

template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::function<T(int64_t, int64_t)>& prog_generator,
    const int64_t range_start, const int64_t range_end,
    const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&
        initial_guesses_generator,
    const std::function<std::optional<SolverOptions>(int64_t, int64_t)>&
        solver_options_generator,
    const std::function<std::optional<SolverId>(int64_t, int64_t)>&
        solver_ids_generator,
    Parallelism parallelism, bool dynamic_schedule,
    const std::function<void(T*, const MathematicalProgramResult&, int64_t,
                             int64_t)>* prog_teardown) {
  DRAKE_THROW_UNLESS(range_start < range_end);
  const int num_progs = range_end - range_start;
  // Pre-allocate the results (i.e., don't use push_back) so that we can safely
  // write to the vector on multiple threads concurrently.
  std::vector<MathematicalProgramResult> results{
      static_cast<std::size_t>(num_progs)};

  // Track which results are set during the par-for loop. Some programs are not
  // thread safe so will be skipped inside the loop, and we'll need to circle
  // back and solve them serially later. (N.B. we cannot use vector<bool> here
  // because it's not thread safe.)
  std::vector<uint8_t> result_is_populated(num_progs, 0);

  // As unique types of solvers are encountered by the threads, we will cache
  // the solvers into this data structure so that we don't have to create and
  // destroy them at every call to Solve.
  std::vector<std::unordered_map<SolverId, std::unique_ptr<SolverInterface>>>
      solvers(parallelism.num_threads());

  const auto range_index_to_output_index = [&](int64_t i) {
    return static_cast<int>(i) - range_start;
  };

  // The worker lambda behaves slightly differently depending on whether we are
  // in the par-for loop or in the single-threaded cleanup pass later on.
  // This is the worker callback for the i'th program.
  auto solve_ith = [&](const bool in_parallel, const int thread_num,
                       const int64_t i) {
    const int output_index = range_index_to_output_index(i);
    T prog = prog_generator(thread_num, i);
    if (prog == nullptr) {
      return;
    }

    // Access (or choose) the required solver.
    std::optional<SolverId> solver_id = solver_ids_generator(thread_num, i);
    if (!solver_id.has_value()) {
      solver_id = ChooseBestSolver(*prog);
    }

    // IPOPT's MUMPs linear solver is not thread-safe. Therefore, if we are
    // operating in parallel we need to skip this program.
    // TODO(#21476) Revisit this logic after we have SPRAL available.
    if (in_parallel && solver_id == IpoptSolver::id()) {
      static const logging::Warn log_once(
          "IpoptSolver cannot currently solve programs in parallel, so will be "
          "run serially. Consider specifying a different solver, e.g., Nlopt.");
      return;
    }

    // Find (or create) the specified solver.
    auto solver_iter = solvers[thread_num].find(solver_id.value());
    if (solver_iter == solvers[thread_num].end()) {
      // If this thread has not solved this type of program yet, save the solver
      // for use later.
      solver_iter = solvers[thread_num].emplace_hint(
          solver_iter, solver_id.value(), MakeSolver(solver_id.value()));
    }
    const SolverInterface& solver = *(solver_iter->second);

    // Adjust the solver options to obey `parallelism`. If this solve is part of
    // the par-for, then the solver is only allowed one thread; otherwise
    // (during the serial cleanup pass) it is allowed `parallelism`. Note that
    // if the user set a solver-specific options to use more threads, that could
    // cause it to exceed its limit.
    std::optional<SolverOptions> new_options =
        solver_options_generator(thread_num, i);
    if (!new_options.has_value()) {
      new_options.emplace();
    }
    new_options->SetOption(CommonSolverOption::kMaxThreads,
                           in_parallel ? 1 : parallelism.num_threads());

    std::optional<Eigen::VectorXd> initial_guess =
        initial_guesses_generator(thread_num, i);

    // Solve the program.
    solver.Solve(*prog, initial_guess, new_options, &(results[output_index]));
    result_is_populated[output_index] = true;

    if (prog_teardown != nullptr) {
      (*prog_teardown)(&prog, results[output_index], thread_num, i);
    }
    //
    //    std::cout << prog << std::endl;
    //    std::cout << "INTERNAL SOLVE" << std::endl;
    //    std::cout << (results[output_index]).get_optimal_cost() << std::endl;
    //    std::cout << (results[output_index]).get_solution_result() <<
    //    std::endl; std::cout << fmt::format(
    //                     "{}\n",
    //                     fmt_eigen(results[output_index].get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << "DONE" << std::endl;
  };
  const auto solve_ith_parallel = [&](const int thread_num, const int64_t i) {
    solve_ith(/* in parallel */ true, thread_num, i);
  };

  const auto solve_ith_serial = [&](const int64_t i) {
    solve_ith(/* in parallel */ false, /* thread_num */ 0, i);
  };

  // Call solve_ith in parallel for all the progs if more than one thread is
  // available.
  if (parallelism.num_threads() > 1) {
    if (dynamic_schedule) {
      DynamicParallelForIndexLoop(
          DegreeOfParallelism(parallelism.num_threads()), range_start,
          range_end, solve_ith_parallel, ParallelForBackend::BEST_AVAILABLE);
    } else {
      StaticParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()),
                                 range_start, range_end, solve_ith_parallel,
                                 ParallelForBackend::BEST_AVAILABLE);
    }
  }

  // De-allocate the solvers cache except for the first worker. (We'll use the
  // first worker's cache for the serial solves, below.) The clearing is
  // important in case the solvers hold onto scarce resources (e.g., licenses).
  solvers.resize(1);

  // Finish solving the programs that couldn't be solved in parallel.
  for (int64_t i = range_start; i < range_end; ++i) {
    if (!result_is_populated[range_index_to_output_index(i)]) {
      solve_ith_serial(i);
    }
  }
  return results;
}

template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::function<T(int64_t, int64_t)>& prog_generator,
    const int64_t range_start, const int64_t range_end,
    const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&
        initial_guesses_generator,
    const SolverOptions* solver_options,
    const std::optional<SolverId>& solver_id, Parallelism parallelism,
    bool dynamic_schedule,
    const std::function<void(T*, const MathematicalProgramResult&, int64_t,
                             int64_t)>* prog_teardown) {
  auto solver_options_generator =
      [&solver_options](int64_t, int64_t) -> std::optional<SolverOptions> {
    static const std::optional<SolverOptions> returned_options =
        solver_options == nullptr
            ? std::nullopt
            : std::optional<SolverOptions>{*solver_options};
    return returned_options;
  };
  auto solver_id_generator = [&solver_id](int64_t,
                                          int64_t) -> std::optional<SolverId> {
    return solver_id;
  };
  return SolveInParallel<T>(prog_generator, range_start, range_end,
                            initial_guesses_generator, solver_options_generator,
                            solver_id_generator, parallelism, dynamic_schedule,
                            prog_teardown);
}

namespace {
template <typename T>
T* GetRawPointer(T* rawPointer) {
  return rawPointer;
}

template <typename T>
T* GetRawPointer(const std::unique_ptr<T>& uniquePointer) {
  return uniquePointer.get();
}
}  // namespace

template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<T>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::vector<const SolverOptions*>* solver_options,
    const std::vector<std::optional<SolverId>>* solver_ids,
    const Parallelism parallelism, const bool dynamic_schedule) {
  DRAKE_THROW_UNLESS(
      std::all_of(progs.begin(), progs.end(), [](const auto& prog) {
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

  auto prog_generator =
      [&progs](const int, const int64_t i) -> const MathematicalProgram* {
    return GetRawPointer(progs[i]);
  };

  auto initial_guess_generator =
      [&initial_guesses](int64_t, int64_t i) -> std::optional<Eigen::VectorXd> {
    if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
      // Convert the initial guess from nullable to optional.
      // TODO(jwnimmer-tri) The SolverBase should offer a nullable overload so
      // that we can avoid this copying.
      return *(initial_guesses->at(i));
    } else {
      return std::nullopt;
    }
  };

  auto solver_options_generator =
      [&solver_options](int64_t, int64_t i) -> std::optional<SolverOptions> {
    if (solver_options != nullptr) {
      return *(solver_options->at(i));
    } else {
      return std::nullopt;
    }
  };

  auto solver_ids_generator =
      [&solver_ids](int64_t, int64_t i) -> std::optional<SolverId> {
    if (solver_ids != nullptr && solver_ids->at(i).has_value()) {
      return solver_ids->at(i);
    } else {
      return std::nullopt;
    }
  };

  return SolveInParallel<const MathematicalProgram*>(
      prog_generator, static_cast<int64_t>(0),
      static_cast<int64_t>(progs.size()), initial_guess_generator,
      solver_options_generator, solver_ids_generator, parallelism,
      dynamic_schedule, nullptr);
}

template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<T>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const SolverOptions* solver_options,
    const std::optional<SolverId>& solver_id, Parallelism parallelism,
    bool dynamic_schedule) {
  auto prog_generator =
      [&progs](const int, const int64_t i) -> const MathematicalProgram* {
    return GetRawPointer(progs[i]);
  };

  auto initial_guess_generator =
      [&initial_guesses](int64_t, int64_t i) -> std::optional<Eigen::VectorXd> {
    if (initial_guesses != nullptr && initial_guesses->at(i) != nullptr) {
      return *(initial_guesses->at(i));
    } else {
      return std::nullopt;
    }
  };

  return SolveInParallel<const MathematicalProgram*>(
      prog_generator, static_cast<int64_t>(0),
      static_cast<int64_t>(progs.size()), initial_guess_generator,
      solver_options, solver_id, parallelism, dynamic_schedule, nullptr);
}

// Explicit template instantiations for the generator forms of SolveInParallel.
#define DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_GENERATOR_INSTANTIATE(PointerType)    \
  template std::vector<MathematicalProgramResult>                              \
  SolveInParallel<PointerType>(                                                \
      const std::function<PointerType(int64_t, int64_t)>&, const int64_t,      \
      const int64_t,                                                           \
      const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&,  \
      const std::function<std::optional<SolverOptions>(int64_t, int64_t)>&,    \
      const std::function<std::optional<SolverId>(int64_t, int64_t)>&,         \
      Parallelism, bool,                                                       \
      const std::function<void(PointerType*, const MathematicalProgramResult&, \
                               int64_t, int64_t)>*);                           \
                                                                               \
  template std::vector<MathematicalProgramResult>                              \
  SolveInParallel<PointerType>(                                                \
      const std::function<PointerType(int64_t, int64_t)>&, const int64_t,      \
      const int64_t,                                                           \
      const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&,  \
      const SolverOptions*, const std::optional<SolverId>&, Parallelism, bool, \
      const std::function<void(PointerType*, const MathematicalProgramResult&, \
                               int64_t, int64_t)>*)

DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_GENERATOR_INSTANTIATE(
    const MathematicalProgram*);
DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_GENERATOR_INSTANTIATE(MathematicalProgram*);
DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_GENERATOR_INSTANTIATE(
    std::unique_ptr<MathematicalProgram>);
#undef DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_GENERATOR_INSTANTIATE

// Explicit template instantiations for the vector forms of SolveInParallel.
#define DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_INSTANTIATE(PointerType)           \
  template std::vector<MathematicalProgramResult>                           \
  SolveInParallel<PointerType>(const std::vector<PointerType>&,             \
                               const std::vector<const Eigen::VectorXd*>*,  \
                               const std::vector<const SolverOptions*>*,    \
                               const std::vector<std::optional<SolverId>>*, \
                               Parallelism, bool);                          \
  template std::vector<MathematicalProgramResult>                           \
  SolveInParallel<PointerType>(                                             \
      const std::vector<PointerType>&,                                      \
      const std::vector<const Eigen::VectorXd*>*, const SolverOptions*,     \
      const std::optional<SolverId>&, Parallelism, bool)

DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_INSTANTIATE(const MathematicalProgram*);
DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_INSTANTIATE(MathematicalProgram*);
DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_INSTANTIATE(
    std::unique_ptr<MathematicalProgram>);
#undef DRAKE_EXPLICIT_SOLVE_IN_PARALLEL_INSTANTIATE

}  // namespace solvers
}  // namespace drake
