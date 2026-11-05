#pragma once

#include <string>

#include "drake/common/name_value.h"

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
    a->Visit(DRAKE_NVP(cenic_explicit_external_inputs));
  }

  std::string integration_scheme{"runge_kutta3"};
  double max_step_size{0.1};
  double accuracy{1.0e-4};
  bool use_error_control{true};
  /// Starting time of the simulation. We will set the context time to
  /// `start_time` at the beginning of the simulation.
  double start_time{0.0};
  double target_realtime_rate{0.0};
  /// (Advanced) Only used when `integration_scheme` is "cenic", and ignored
  /// otherwise. If true, the MultibodyPlant's external inputs (actuation and
  /// external forces) are held fixed over each step instead of being linearized
  /// with finite differences so they can be treated implicitly. This avoids
  /// re-evaluating upstream systems several times per step, which is useful
  /// when using fixed-step CENIC (`use_error_control: false`) in place of
  /// discrete mode. See drake::multibody::contact_solvers::icf::
  /// IcfSolverParameters::explicit_external_inputs, and the "Discrete-like
  /// mode" section of drake::multibody::CenicIntegrator.
  bool cenic_explicit_external_inputs{false};
};
}  // namespace systems
}  // namespace drake
