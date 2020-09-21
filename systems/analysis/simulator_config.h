#pragma once

#include <string>

#include "drake/common/name_value.h"

// TODO(jeremy.nimmer) Move this file into Drake once we like how it works.
// See https://github.com/RobotLocomotion/drake/issues/12903.

namespace anzu {
namespace sim {

// N.B. The names and defaults here match drake/systems/analysis exactly.
/// The set of configurable properties on a simulator.
struct SimulatorConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(integration_scheme));
    a->Visit(DRAKE_NVP(max_step_size));
    a->Visit(DRAKE_NVP(accuracy));
    a->Visit(DRAKE_NVP(use_error_control));
    a->Visit(DRAKE_NVP(target_realtime_rate));
    a->Visit(DRAKE_NVP(publish_every_time_step));
  }

  std::string integration_scheme{"runge_kutta3"};
  double max_step_size{1.0e-3};
  double accuracy{1.0e-2};
  bool use_error_control{true};
  double target_realtime_rate{0.0};
  double publish_every_time_step{false};
};

}  // namespace sim
}  // namespace anzu
