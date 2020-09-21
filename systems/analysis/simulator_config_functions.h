#pragma once

#include "drake/systems/analysis/simulator.h"
#include "sim/common/simulator_config.h"

// TODO(jeremy.nimmer) Move this file into Drake once we like how it works.
// See https://github.com/RobotLocomotion/drake/issues/12903.

namespace anzu {
namespace sim {

/// Modify the simulator to use the given config.  (Always replaces the
/// Integrator with a new one; be careful not to keep old references around.)
void ApplySimulatorConfig(
    drake::systems::Simulator<double>* simulator,
    const SimulatorConfig& config);

/// Report the simulator's current config.
SimulatorConfig ExtractSimulatorConfig(
    const drake::systems::Simulator<double>& simulator);

}  // namespace sim
}  // namespace anzu
