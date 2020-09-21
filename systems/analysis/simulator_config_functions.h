#pragma once

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config.h"

namespace drake {
namespace systems {

/// Modifies the `simulator` to use the given config.  (Always replaces the
/// Integrator with a new one; be careful not to keep old references around.)
void ApplySimulatorConfig(
    drake::systems::Simulator<double>* simulator,
    const SimulatorConfig& config);

/// Reports the simulator's current configuration.
SimulatorConfig ExtractSimulatorConfig(
    const drake::systems::Simulator<double>& simulator);

}  // namespace systems
}  // namespace drake
