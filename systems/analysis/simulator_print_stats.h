#pragma once

#include <string>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {

/// This method prints relevant stepping and integration statistics for a
/// simulator that integrated a system with continuous states.
/// @param[in] simulator
///   The simulator to print statistics for.
/// @param[in] integration_scheme_name
///   The name of the integration scheme to be displayed in the output.
void PrintSimulatorStatistics(const Simulator<double>& simulator,
                              const std::string& integration_scheme_name);

}  // namespace systems
}  // namespace drake
