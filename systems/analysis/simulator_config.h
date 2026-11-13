#pragma once

#include <string>
#include <variant>

#include "drake/common/name_value.h"
#include "drake/multibody/cenic/cenic_integrator_parameters.h"

namespace drake {
namespace systems {
// TODO(dale.mcconachie) Update to include all configurable properties of
// IntegratorBase. Currently, initial_step_size_target, minimum_step_size, and
// throw_on_minimum_step_size_violation are missing.
/// The set of all configurable properties on a Simulator and IntegratorBase.
struct SimulatorConfig {
  template <typename Archive>
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(integration_scheme));
    a->Visit(DRAKE_NVP(max_step_size));
    a->Visit(DRAKE_NVP(accuracy));
    a->Visit(DRAKE_NVP(use_error_control));
    a->Visit(DRAKE_NVP(start_time));
    a->Visit(DRAKE_NVP(target_realtime_rate));
    a->Visit(DRAKE_NVP(integrator_params));
  }

  std::string integration_scheme{"runge_kutta3"};
  double max_step_size{0.1};
  double accuracy{1.0e-4};
  bool use_error_control{true};
  /// Starting time of the simulation. We will set the context time to
  /// `start_time` at the beginning of the simulation.
  double start_time{0.0};
  double target_realtime_rate{0.0};
  /// (Advanced) Integrator-specific parameters, beyond the generic ones above.
  /// The default (std::monostate, or `null` in YAML) leaves the integrator's
  /// own default parameters in place. Otherwise, the alternative must match
  /// `integration_scheme`; currently the only choice is
  /// multibody::CenicIntegratorParameters (with `integration_scheme: cenic`),
  /// which is selected in YAML with the `!CenicIntegratorParameters` tag. It
  /// is an error to provide parameters for a different integration scheme.
  /// See, e.g., the "Explicit external inputs" section of
  /// drake::multibody::CenicIntegrator.
  std::variant<std::monostate, multibody::CenicIntegratorParameters>
      integrator_params;
};
}  // namespace systems
}  // namespace drake
