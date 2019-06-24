#include "drake/systems/primitives/random_source.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace systems {

namespace internal {
template<typename Generator>
typename Generator::result_type generate_unique_seed() {
  static never_destroyed<typename Generator::result_type> seed(
      Generator::default_seed);
  return seed.access()++;
}

template RandomGenerator::result_type generate_unique_seed<RandomGenerator>();

}  // namespace internal

int AddRandomInputs(double sampling_interval_sec,
                    DiagramBuilder<double>* builder) {
  int count = 0;
  // Note: the mutable assignment to const below looks odd, but
  // there is (currently) no builder->GetSystems() method.
  for (const auto* system : builder->GetMutableSystems()) {
    for (int i = 0; i < system->num_input_ports(); i++) {
      const systems::InputPort<double>& port =
          system->get_input_port(i);
      // Check for the random label.
      if (!port.is_random()) {
        continue;
      }

      typedef typename Diagram<double>::InputPortLocator InputPortLocator;
      // Check if the input is already wired up.
      InputPortLocator id{&port.get_system(), port.get_index()};
      if (builder->connection_map_.count(id) > 0 ||
          builder->diagram_input_set_.count(id) > 0) {
        continue;
      }

      count++;
      switch (port.get_random_type().value()) {
        case RandomDistribution::kUniform: {
          const auto* uniform = builder->AddSystem<UniformRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(uniform->get_output_port(0), port);
          continue;
        }
        case RandomDistribution::kGaussian: {
          const auto* gaussian = builder->AddSystem<GaussianRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(gaussian->get_output_port(0), port);
          continue;
        }
        case RandomDistribution::kExponential: {
          const auto* exponential = builder->AddSystem<ExponentialRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(exponential->get_output_port(0), port);
          continue;
        }
      }
      DRAKE_UNREACHABLE();
    }
  }
  return count;
}

}  // namespace systems
}  // namespace drake

