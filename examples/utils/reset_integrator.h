#pragma once

#include <string>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {

/// Returns the list of integrators supported by ResetIntegrator(), separated by
/// commas.
inline std::string supported_integrators() {
  return std::string(
      "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
      "'implicit_euler'");
}

/// Resets the integrator used to advanced the continuous time dynamics of the
/// system associated with `simulator` according to `integration_scheme`.
/// @param[in] integration_scheme
///   Name of the desired integration scheme. It must be one of the names in
///   supported_integrators() or a std::runtime_exception exception is thrown.
/// @param[in] max_time_step
///   Desired maximum time step to be used by the integrator.
/// @param[in] accuracy Desired integration accuracy. See
/// IntegratorBase::set_target_accuracy().
/// @param[in,out] simulator On input, a valid pointer to a systems::Simulator.
/// On output the integrator for `simulator` is reset according to
/// `integration_scheme`.
///
/// @throws std::runtime_error if `integration_scheme` is not listed in
/// supported_integrators().
systems::IntegratorBase<double>& ResetIntegrator(
    std::string integration_scheme, double max_time_step, double accuracy,
    systems::Simulator<double>* simulator);

}  // namespace examples
}  // namespace drake
