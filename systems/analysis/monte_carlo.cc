#include "drake/systems/analysis/monte_carlo.h"

#include <future>
#include <list>
#include <mutex>
#include <thread>

#include "drake/common/text_logging.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {

double RandomSimulation(const RandomSimulatorFactory& make_simulator,
                        const ScalarSystemFunction& output,
                        const double final_time,
                        RandomGenerator* const generator) {
  auto simulator = make_simulator(generator);

  const System<double>& system = simulator->get_system();
  system.SetRandomContext(&simulator->get_mutable_context(), generator);

  simulator->AdvanceTo(final_time);

  return output(system, simulator->get_context());
}

namespace {

// Serial (single-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationSerial(
    const RandomSimulatorFactory& make_simulator,
    const ScalarSystemFunction& output, const double final_time,
    const int num_samples, RandomGenerator* const generator) {
  std::vector<RandomSimulationResult> simulation_results;
  simulation_results.reserve(num_samples);

  for (int sample = 0; sample < num_samples; ++sample) {
    RandomSimulationResult simulation_result(*generator);
    simulation_result.output =
        RandomSimulation(make_simulator, output, final_time, generator);
    simulation_results.push_back(std::move(simulation_result));
  }

  return simulation_results;
}

// Checks if a future has completed execution.
template <typename T>
bool IsFutureReady(const std::future<T>& future) {
  // future.wait_for() is the only method to check the status of a future
  // without waiting for it to complete.
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
}

// Parallel (multi-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationParallel(
    const RandomSimulatorFactory& make_simulator,
    const ScalarSystemFunction& output, const double final_time,
    const int num_samples, RandomGenerator* const generator,
    const int num_threads) {
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

  while (active_operations.size() > 0 || simulations_dispatched < num_samples) {
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
    while (static_cast<int>(active_operations.size()) < num_threads &&
           simulations_dispatched < num_samples) {
      // Create the simulation result using the current generator state.
      simulation_results.at(simulations_dispatched) =
          RandomSimulationResult(*generator);

      // Make the simulator.
      auto simulator = make_simulator(generator);
      const auto& system = simulator->get_system();
      system.SetRandomContext(&simulator->get_mutable_context(), generator);

      auto perform_simulation = [simulator = std::move(simulator),
                                 &simulation_results, &output, final_time,
                                 sample_num = simulations_dispatched]() {
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

}  // namespace

std::vector<RandomSimulationResult> MonteCarloSimulation(
    const RandomSimulatorFactory& make_simulator,
    const ScalarSystemFunction& output, const double final_time,
    const int num_samples, RandomGenerator* generator,
    const Parallelism parallelism) {
  // Create a generator if the user didn't provide one.
  std::unique_ptr<RandomGenerator> owned_generator;
  if (generator == nullptr) {
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  // Since the parallel implementation incurs additional overhead even in the
  // num_threads=1 case, dispatch to the serial implementation in these cases.
  const int num_threads = parallelism.num_threads();
  if (num_threads > 1) {
    return MonteCarloSimulationParallel(make_simulator, output, final_time,
                                        num_samples, generator, num_threads);
  } else {
    return MonteCarloSimulationSerial(make_simulator, output, final_time,
                                      num_samples, generator);
  }
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
