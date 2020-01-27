#pragma once

/// @file
/// This file defines gflags settings to control Simulator settings.
/// Only include this from translation units that declare a `main` function.

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

// Declares integrator gflags.
DECLARE_string(simulator_integration_scheme);
DECLARE_double(simulator_max_time_step);
DECLARE_double(simulator_accuracy);

// Declares simulator gflags.
DECLARE_double(simulator_target_realtime_rate);
DECLARE_bool(simulator_publish_every_time_step);

namespace drake {
namespace systems {

/// Resets the integrator used to advanced the continuous time dynamics of the
/// system associated with `simulator` according to the gflags declared in this
/// file.
/// @param[in,out] simulator
///   On input, a valid pointer to a Simulator. On output the
///   integrator for `simulator` is reset according to the gflags declared in
///   this file.
/// @returns  A reference to the the newly created integrator owned by
/// `simulator`.
IntegratorBase<double>& ResetIntegratorFromGflags(Simulator<double>* simulator);

/// Makes a new simulator according to the gflags declared in this file.
/// @param[in] system
///   The System to be associated with the newly crated Simulator. You must
///   ensure that `system` has a longer lifetime than the new Simulator.
/// @param[in] context
///   The Context that will be used as the initial condition for the simulation;
///   otherwise the Simulator will obtain a default Context from `system`.
/// @returns The newly created Simulator.
std::unique_ptr<Simulator<double>> MakeSimulatorFromGflags(
    const System<double>& system,
    std::unique_ptr<Context<double>> context = nullptr);

}  // namespace systems
}  // namespace drake
