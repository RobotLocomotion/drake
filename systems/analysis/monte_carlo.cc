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

/// Serial (single-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationSerial(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    double final_time, int num_samples, RandomGenerator* generator) {
  std::vector<RandomSimulationResult> simulation_results;
  simulation_results.reserve(num_samples);

  for (int sample = 0; sample < num_samples; ++sample) {
    RandomSimulationResult simulation_result(*generator);
    simulation_result.output =
        RandomSimulation(make_simulator, output, final_time, generator);
    simulation_results.push_back(simulation_result);
  }

  return simulation_results;
}

/// Parallel (multi-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationParallel(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    double final_time, int num_samples, RandomGenerator* generator,
    int num_parallel_executions) {
  // Initialize all the simulations in deterministic order.
  std::vector<std::unique_ptr<Simulator<double>>> simulators;
  simulators.reserve(num_samples);
  std::vector<RandomSimulationResult> simulation_results;
  simulation_results.reserve(num_samples);

  for (int sample = 0; sample < num_samples; ++sample) {
    RandomSimulationResult simulation_result(*generator);
    simulation_results.push_back(simulation_result);

    simulators.emplace_back(make_simulator(generator));
    auto& simulator = *simulators.back();
    const System<double>& system = simulator.get_system();
    system.SetRandomContext(&simulator.get_mutable_context(), generator);
  }

  // Storage for active parallel simulation operations.
  std::list<std::future<std::pair<int, double>>> active_operations;
  // Keep track of how many simulations have been dispatched already.
  int simulations_dispatched = 0;

  while (active_operations.size() > 0 ||
         simulations_dispatched < num_samples) {
    // Check for completed operations.
    for (auto itr = active_operations.begin();
         itr != active_operations.end();) {
      if (IsFutureReady(*itr)) {
        auto result = itr->get();
        drake::log()->info("Simulation {} completed", result.first);
        simulation_results.at(result.first).output = result.second;
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
          [&simulators, &simulation_results, &output, final_time] (
              int sample_num) {
        auto& simulator = *simulators.at(sample_num);
        simulator.AdvanceTo(final_time);
        const double output_val =
            output(simulator.get_system(), simulator.get_context());
        return std::make_pair(sample_num, output_val);
      };

      active_operations.emplace_back(std::async(
          std::launch::async, perform_simulation, simulations_dispatched));
      drake::log()->info("Simulation {} dispatched", simulations_dispatched);
      simulations_dispatched += 1;
    }

    // Wait a bit before checking for completion.
    // TODO(calderpg-tri) When std::when_any([std::future,...]) or equivalent is
    // available, this can be replaced.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return simulation_results;
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

int GetHardwareConcurrency() {
  return static_cast<int>(std::thread::hardware_concurrency());
}

std::vector<RandomSimulationResult> MonteCarloSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples, RandomGenerator* generator,
    const std::optional<int>& num_parallel_executions) {
  if (num_parallel_executions.has_value()) {
    DRAKE_THROW_UNLESS(num_parallel_executions.value() >= 1);
  }

  std::unique_ptr<RandomGenerator> owned_generator{};
  if (generator == nullptr) {
    // Create a generator to be used for this set of tests.
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  const int hardware_concurrency = GetHardwareConcurrency();
  int num_threads = 0;
  if (num_parallel_executions.has_value()) {
    num_threads = num_parallel_executions.value();
    if (num_threads > hardware_concurrency) {
      drake::log()->warn(
          "Provided num_parallel_executions value of {} is greater than the "
          "value of hardware concurrency {} for this computer, this is likely "
          "to result in poor peformance",
          num_threads, hardware_concurrency);
    } else {
      drake::log()->debug(
          "Using provided value of {} parallel executions", num_threads);
    }
  } else {
    num_threads = hardware_concurrency;
    drake::log()->debug(
        "num_parallel_executions not provided, using default value of {} "
        "parallel executions",
        num_threads);
  }

  // Since the parallel implementation incurs additional overhead even in the
  // num_threads=1 case, dispatch to the serial implementation in these cases.
  if (num_threads > 1) {
    return MonteCarloSimulationParallel(
        make_simulator, output, final_time, num_samples, generator,
        num_threads);
  } else {
    return MonteCarloSimulationSerial(
        make_simulator, output, final_time, num_samples, generator);
  }
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
