#include "drake/systems/analysis/simulator_config_functions.h"

#include <memory>
#include <string>

#include "drake/common/nice_type_name.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator_flags.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

using drake::NiceTypeName;
using drake::systems::IntegratorBase;
using drake::systems::Simulator;

void ApplySimulatorConfig(
    Simulator<double>* simulator,
    const SimulatorConfig& config) {
  DRAKE_THROW_UNLESS(simulator != nullptr);
  IntegratorBase<double>& integrator =
      drake::systems::ResetIntegratorFromFlags(
          simulator, config.integration_scheme, config.max_step_size);
  if (integrator.supports_error_estimation()) {
    integrator.set_fixed_step_mode(!config.use_error_control);
  }
  if (!integrator.get_fixed_step_mode()) {
    integrator.set_target_accuracy(config.accuracy);
  }
  simulator->set_target_realtime_rate(config.target_realtime_rate);
  simulator->set_publish_at_initialization(config.publish_every_time_step);
  simulator->set_publish_every_time_step(config.publish_every_time_step);
}

namespace {
// A hollow shell of a System.  TODO(jeremy.nimmer) Move into drake primitives.
class DummySystem final : public drake::systems::LeafSystem<double> {
 public:
  DummySystem() {}
};
std::string GetIntegrationSchemeName(const IntegratorBase<double>& integrator) {
  const std::string current_type = NiceTypeName::Get(integrator);
  Simulator<double> dummy_simulator(std::make_unique<DummySystem>());
  for (const auto& scheme : drake::systems::GetIntegrationSchemes()) {
    ResetIntegratorFromFlags(&dummy_simulator, scheme, 0.001);
    if (NiceTypeName::Get(dummy_simulator.get_integrator()) == current_type) {
      return scheme;
    }
  }
  throw std::runtime_error(
      "Unrecognized integration scheme " + current_type);
}
}  // namespace

SimulatorConfig ExtractSimulatorConfig(
    const Simulator<double>& simulator) {
  SimulatorConfig result;
  const IntegratorBase<double>& integrator = simulator.get_integrator();
  result.integration_scheme = GetIntegrationSchemeName(integrator);
  result.max_step_size = integrator.get_maximum_step_size();
  if (integrator.supports_error_estimation()) {
    result.use_error_control = !integrator.get_fixed_step_mode();
    const double accuracy_in_use = integrator.get_accuracy_in_use();
    DRAKE_DEMAND(!std::isnan(accuracy_in_use));
    result.accuracy = accuracy_in_use;
  } else {
    result.use_error_control = false;
    result.accuracy = 0.0;
  }
  result.target_realtime_rate = simulator.get_target_realtime_rate();
  result.publish_every_time_step = simulator.get_publish_every_time_step();
  return result;
}

}  // namespace systems
}  // namespace drake
