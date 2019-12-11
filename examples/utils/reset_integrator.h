#pragma once

#include <string>

#include <gflags/gflags.h>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {

/// Defines gflag `integration_scheme` to specify the integration scheme used by
/// systems::Simulator to advance continuous dynamics.
/// @see ResetIntegrator()
#define DEFINE_integration_scheme()                                           \
DEFINE_string(integration_scheme, "implicit_euler",                           \
              "Integration scheme to be used. Available options are: "        \
              "'bogacki_shampine3',"                                          \
              "'explicit_euler','implicit_euler','semi_explicit_euler',"      \
              "'radau1','radau3',"                                            \
              "'runge_kutta2','runge_kutta3','runge_kutta5'");

/// Resets the integrator used to advanced the continuous time dynamics of the
/// system associated with `simulator` according to `integration_scheme`.
/// @param[in] integration_scheme
///   Name of the desired integration scheme. It must be one of the names in
///   defined by DEFINE_integration_scheme() or a std::runtime_exception
///   exception is thrown.
/// @param[in] max_time_step
///   Desired maximum time step to be used by the integrator.
/// @param[in] accuracy Desired integration accuracy. See
/// IntegratorBase::set_target_accuracy().
/// @param[in,out] simulator On input, a valid pointer to a systems::Simulator.
/// On output the integrator for `simulator` is reset according to
/// `integration_scheme`.
///
/// @throws std::runtime_error if `integration_scheme` is not listed in
/// DEFINE_integration_scheme().
systems::IntegratorBase<double>& ResetIntegrator(
    std::string integration_scheme, double max_time_step, double accuracy,
    systems::Simulator<double>* simulator);

}  // namespace examples
}  // namespace drake
