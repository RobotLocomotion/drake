#include "drake/systems/analysis/monte_carlo.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {

double RandomSimulation(const SimulatorFactory make_simulator,
                        const ScalarSystemFunction output, double final_time,
                        RandomGenerator* generator) {
  auto simulator = make_simulator(generator);

  if (!simulator->has_owned_system())
    DRAKE_ABORT_MSG(
        "A SimulatorFactory must return a Simulator which owns "
        "the System.  Use the constructor for Simulator which "
        "takes a unique_ptr to the System");

  const System<double>& system = simulator->get_system();
  system.SetRandomContext(&simulator->get_mutable_context(), generator);

  simulator->StepTo(final_time);

  return output(system, simulator->get_context());
}

std::vector<std::pair<const RandomGenerator, double>> MonteCarloSimulation(
    const SimulatorFactory make_simulator, const ScalarSystemFunction output,
    double final_time, int num_samples, RandomGenerator* generator) {
  std::unique_ptr<RandomGenerator> owned_generator{};
  if (generator == nullptr) {
    // Create a generator to be used for this set of tests.
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  std::vector<std::pair<const RandomGenerator, double>> data;
  data.reserve(num_samples);

  // TODO(russt): Support running these in multiple threads, as they are
  // trivially parallelizable.
  for (int i = 0; i < num_samples; i++) {
    const RandomGenerator generator_snapshot = *generator;
    double value =
        RandomSimulation(make_simulator, output, final_time, generator);
    data.emplace_back(std::make_pair(generator_snapshot, value));
  }

  return data;
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
