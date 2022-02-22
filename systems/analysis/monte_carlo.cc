#include "drake/systems/analysis/monte_carlo.h"

#include <atomic>
#include <future>
#include <list>
#include <mutex>
#include <thread>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace internal {

std::unique_ptr<Simulator<double>> MakeRandomSimulator(
    const SimulatorFactory& make_simulator,
    RandomGenerator* generator) {
  std::unique_ptr<Simulator<double>> result = make_simulator(generator);
  result->get_system().SetRandomContext(
      &result->get_mutable_context(), generator);
  return result;
}

}  // namespace internal

double RandomSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, RandomGenerator* const generator) {
  DRAKE_THROW_UNLESS(generator != nullptr);
  auto simulator = internal::MakeRandomSimulator(make_simulator, generator);
  simulator->AdvanceTo(final_time);
  return output(simulator->get_system(), simulator->get_context());
}

namespace internal {

// Serial (single-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationSerial(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples,
    RandomGenerator* const generator) {
  DRAKE_DEMAND(num_samples > 0);
  DRAKE_DEMAND(generator != nullptr);
  std::vector<RandomSimulationResult> simulation_results(num_samples);

  for (int sample = 0; sample < num_samples; ++sample) {
    simulation_results[sample].generator_snapshot = *generator;
    auto simulator = internal::MakeRandomSimulator(make_simulator, generator);
    simulator->AdvanceTo(final_time);
    simulation_results[sample].output = output(
        simulator->get_system(), simulator->get_context());
  }

  return simulation_results;
}

namespace {

// Checks if a future has completed execution.
template <typename T>
bool IsFutureReady(const std::future<T>& future) {
  // future.wait_for() is the only method to check the status of a future
  // without waiting for it to complete.
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
}

}  // namespace

// Parallel (multi-threaded) implementation of MonteCarloSimulation, using
// C++ standard library threads with manual dynamic scheduling.
std::vector<RandomSimulationResult> MonteCarloSimulationParallelStd(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples,
    RandomGenerator* const generator, const int num_threads) {
  DRAKE_DEMAND(num_samples > 0);
  DRAKE_DEMAND(generator != nullptr);
  DRAKE_DEMAND(num_threads > 1);

  // Initialize storage for all simulation results. The full vector must be
  // constructed up front (i.e. we can't use reserve()) to avoid a race
  // condition on checking the size of the vector when the worker threads write
  // simulation results.
  std::vector<RandomSimulationResult> simulation_results(
      num_samples, RandomSimulationResult(RandomGenerator()));

  // Storage for active parallel simulation operations.
  std::list<std::future<int>> active_operations;
  // Keep track of how many simulations have been dispatched already.
  int simulations_dispatched = 0;

  while (active_operations.size() > 0 ||
         simulations_dispatched < num_samples) {
    // Check for completed operations.
    for (auto operation = active_operations.begin();
         operation != active_operations.end();) {
      if (IsFutureReady(*operation)) {
        // This call to future.get() is necessary to propagate any exception
        // thrown during simulation execution.
        const int sample_num = operation->get();
        drake::log()->debug("Simulation {} completed", sample_num);
        // Erase returns iterator to the next node in the list.
        operation = active_operations.erase(operation);
      } else {
        // Advance to next node in the list.
        ++operation;
      }
    }

    // Dispatch new operations.
    while (static_cast<int>(active_operations.size()) < num_threads
           && simulations_dispatched < num_samples) {
      // Create the simulation result using the current generator state.
      simulation_results.at(simulations_dispatched) =
          RandomSimulationResult(*generator);

      // Make the simulator.
      auto simulator = make_simulator(generator);
      const auto& system = simulator->get_system();
      system.SetRandomContext(&simulator->get_mutable_context(), generator);

      auto perform_simulation =
          [simulator = std::move(simulator), &simulation_results, &output,
           final_time, sample_num = simulations_dispatched] () {
        simulator->AdvanceTo(final_time);
        simulation_results.at(sample_num).output =
            output(simulator->get_system(), simulator->get_context());
        return sample_num;
      };

      active_operations.emplace_back(
          std::async(std::launch::async, std::move(perform_simulation)));
      drake::log()->debug("Simulation {} dispatched", simulations_dispatched);
      ++simulations_dispatched;
    }

    // Wait a bit before checking for completion.
    // TODO(calderpg-tri) When std::when_any([std::future,...]) or equivalent is
    // available, this can be replaced.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return simulation_results;
}

// Parallel (multi-threaded) implementation of MonteCarloSimulation, using
// OpenMP threads with dynamic scheduling.
std::vector<RandomSimulationResult> MonteCarloSimulationParallelOmp(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    double final_time, int num_samples, RandomGenerator* generator,
    int num_threads) {
  DRAKE_DEMAND(num_samples > 0);
  DRAKE_DEMAND(generator != nullptr);
  DRAKE_DEMAND(num_threads > 1);

  // The input and output of each sample will be stored here.
  std::vector<RandomSimulationResult> simulation_results(num_samples);

  // We need to randomize the initial conditions serially, so that the sequence
  // of random numbers is deterministic. We'll do that by moving the generator
  // object from one per-sample mailbox to the next within the parallel block,
  // with each worker thread blocking until it receives a generator.
  struct GeneratorMailbox {
    std::promise<RandomGenerator> promise;
    std::future<RandomGenerator> future = promise.get_future();
  };
  std::vector<GeneratorMailbox> mailboxes(num_samples + 1);
  mailboxes.front().promise.set_value(std::move(*generator));

  // Manually keep track of exception state, so that we can bail out early.
  std::atomic<bool> cancelled{false};
  std::mutex simulation_error_mutex;
  std::exception_ptr simulation_error;

  // Begin the parallel looping.
  unused(num_threads);
#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic, 1) num_threads(num_threads)
#endif
  for (int i = 0; i < num_samples; ++i) {
    if (cancelled) {
      continue;
    }
    try {
      // Wait until it's our turn to use the generator, then create the random
      // simulator, then pass the generator along to the next sample.
      std::future<RandomGenerator>& incoming_gen = mailboxes[i].future;
      std::promise<RandomGenerator>& outgoing_gen = mailboxes[i + 1].promise;
      incoming_gen.wait();
      RandomGenerator local_gen = incoming_gen.get();
      simulation_results[i].generator_snapshot = local_gen;
      std::unique_ptr<Simulator<double>> simulator =
          internal::MakeRandomSimulator(make_simulator, &local_gen);
      outgoing_gen.set_value(std::move(local_gen));

      // Now that we're done with the random generator, we can independently
      // simulate and calculate the output.
      simulator->AdvanceTo(final_time);
      simulation_results[i].output =
          output(simulator->get_system(), simulator->get_context());
      simulator.reset();
    } catch (...) {
      std::lock_guard<std::mutex> lock(simulation_error_mutex);
      simulation_error = std::current_exception();
      cancelled.store(true);
    }
  }

  // Re-throw any exception.
  if (cancelled) {
    std::rethrow_exception(simulation_error);
  }

  // Write back the final generator state.
  DRAKE_DEMAND(IsFutureReady(mailboxes.back().future));
  *generator = std::move(mailboxes.back().future.get());

  return simulation_results;
}

int SelectNumberOfThreadsToUse(const int num_parallel_executions) {
  const int hardware_concurrency =
      static_cast<int>(std::thread::hardware_concurrency());

  int num_threads = 0;

  if (num_parallel_executions > 1) {
    num_threads = num_parallel_executions;
    if (num_threads > hardware_concurrency) {
      drake::log()->warn(
          "Provided num_parallel_executions value of {} is greater than the "
          "value of hardware concurrency {} for this computer, this is likely "
          "to result in poor performance",
          num_threads, hardware_concurrency);
    } else {
      drake::log()->debug(
          "Using provided value of {} parallel executions", num_threads);
    }
  } else if (num_parallel_executions == kNoConcurrency) {
    num_threads = 1;
    drake::log()->debug("kNoConcurrency specified, using a single thread");
  } else if (num_parallel_executions == kUseHardwareConcurrency) {
    num_threads = hardware_concurrency;
    drake::log()->debug(
        "kUseHardwareConcurrency specified, using hardware concurrency {}",
        num_threads);
  } else {
    throw std::runtime_error(fmt::format(
        "Specified num_parallel_executions {} is not valid. Valid options are "
        "kNoConcurrency, kUseHardwareConcurrency, or num_parallel_executions "
        ">= 1",
        num_parallel_executions));
  }

  return num_threads;
}

}  // namespace internal

std::vector<RandomSimulationResult> MonteCarloSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples, RandomGenerator* generator,
    const int num_parallel_executions) {
  DRAKE_THROW_UNLESS(make_simulator != nullptr);
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(!std::isnan(final_time));
  DRAKE_THROW_UNLESS(num_samples >= 0);

  // Check num_parallel_executions vs the available hardware concurrency.
  // This also serves to sanity-check the num_parallel_executions argument.
  const int num_threads =
      internal::SelectNumberOfThreadsToUse(num_parallel_executions);

  // Bail out early if there's no work to be done.
  if (num_samples == 0) {
    return {};
  }

  // Create a generator if the user didn't provide one.
  std::unique_ptr<RandomGenerator> owned_generator;
  if (generator == nullptr) {
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  // Since the parallel implementation incurs additional overhead even in the
  // num_threads=1 case, dispatch to the serial implementation in these cases.
  auto* parallel_func =
#if defined(_OPENMP)
      &internal::MonteCarloSimulationParallelOmp;
#else
      &internal::MonteCarloSimulationParallelStd;
#endif
  if (num_threads > 1) {
    return parallel_func(
        make_simulator, output, final_time, num_samples, generator,
        num_threads);
  } else {
    return internal::MonteCarloSimulationSerial(
        make_simulator, output, final_time, num_samples, generator);
  }
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
