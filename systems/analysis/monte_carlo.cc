#include "drake/systems/analysis/monte_carlo.h"

#include <future>
#include <list>
#include <mutex>
#include <unordered_map>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {
/// Checks if a future has completed execution.
template<typename T>
inline bool IsFutureReady(const std::future<T>& future) {
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
}
}  // namespace

double RandomSimulation(const SimulatorFactory& make_simulator,
                        const ScalarSystemFunction& output, double final_time,
                        RandomGenerator* generator) {
  auto simulator = make_simulator(generator);

  const System<double>& system = simulator->get_system();
  system.SetRandomContext(&simulator->get_mutable_context(), generator);

  simulator->AdvanceTo(final_time);

  return output(system, simulator->get_context());
}

std::vector<RandomSimulationResult> MonteCarloSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples, RandomGenerator* generator,
    const int num_parallel_executions) {
  std::unique_ptr<RandomGenerator> owned_generator{};
  if (generator == nullptr) {
    // Create a generator to be used for this set of tests.
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  // Operations on the random generator are not thread safe.
  std::mutex generator_mutex;

  // Allocate results for all simulations.
  std::unordered_map<int, RandomSimulationResult> simulation_results(
      num_samples);

  // Storage for active parallel simulations.
  std::list<std::future<std::pair<int, RandomSimulationResult>>>
      active_operations;
  // Keep track of how many simulations have been dispatched already.
  int simulations_dispatched = 0;

  while (active_operations.size() > 0 ||
         simulations_dispatched < num_samples) {
    // Check for completed operations.
    for (auto itr = active_operations.begin();
         itr != active_operations.end();) {
      if (IsFutureReady(*itr)) {
        simulation_results.insert(itr->get());
        // Erase returns iterator to the next node in the list.
        itr = active_operations.erase(itr);
      } else {
        // Advance to next node in the list.
        ++itr;
      }
    }

    // Dispatch new operations.
    while (static_cast<int>(active_operations.size()) < num_parallel_executions
           && simulations_dispatched < num_samples) {
      const auto perform_simulation =
          [&make_simulator, &output, final_time, &generator, &generator_mutex] (
              int sample_num) {
        // This is equivalent to the work performed by RandomSimulation, just
        // with access to the RandomGenerator protected via mutex.
        RandomSimulationResult result(*generator);
        std::unique_ptr<Simulator<double>> simulator;

        {
          std::lock_guard<std::mutex> lock(generator_mutex);
          simulator = make_simulator(generator);

          const System<double>& system = simulator->get_system();
          system.SetRandomContext(&simulator->get_mutable_context(), generator);
        }

        simulator->AdvanceTo(final_time);

        result.output =
            output(simulator->get_system(), simulator->get_context());

        return std::make_pair(sample_num, result);
      };

      active_operations.emplace_back(std::async(
          std::launch::async, perform_simulation, simulations_dispatched));
      simulations_dispatched += 1;
    }
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  }

  // Copy out ordered results.
  std::vector<RandomSimulationResult> data;
  data.reserve(num_samples);

  for (int index = 0; index < num_samples; ++index) {
    data.emplace_back(std::move(simulation_results.at(index)));
  }

  return data;
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
