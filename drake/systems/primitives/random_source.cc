#include "drake/systems/primitives/random_source.h"

namespace drake {
namespace systems {

int AddRandomInputs(double sampling_interval_sec,
                    DiagramBuilder<double>* builder) {
  int count = 0;
  for (const auto system : builder->GetMutableSystems()) {
    for (int i = 0; i < system->get_num_input_ports(); i++) {
      const systems::InputPortDescriptor<double>& port =
          system->get_input_port(i);
      // Check for the random label.
      if (!port.is_random()) {
        continue;
      }

      typedef typename Diagram<double>::PortIdentifier PortIdentifier;
      // Check if the input is already wired up.
      PortIdentifier id{port.get_system(), port.get_index()};
      if (builder->dependency_graph_.find(id) !=
              builder->dependency_graph_.end() ||
          builder->diagram_input_set_.find(id) !=
              builder->diagram_input_set_.end()) {
        continue;
      }

      switch (port.get_random_type().value()) {
        case RandomDistribution::kUniform: {
          const auto uniform = builder->AddSystem<UniformRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(uniform->get_output_port(0), port);
        } break;
        case RandomDistribution::kGaussian: {
          const auto gaussian = builder->AddSystem<GaussianRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(gaussian->get_output_port(0), port);
        } break;
        case RandomDistribution::kExponential: {
          const auto exponential = builder->AddSystem<ExponentialRandomSource>(
              port.size(), sampling_interval_sec);
          builder->Connect(exponential->get_output_port(0), port);
        } break;
        default: {
          DRAKE_ABORT_MSG(
              "InputPortDescriptor has an unsupported RandomDistribution.");
        }
      }
      count++;
    }
  }
  return count;
}

}  // namespace systems
}  // namespace drake
