#pragma once

#warning The include path drake/systems/trajectory_optimization/direct_transcription.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/direct_transcription.h.

#include "drake/common/drake_deprecated.h"
#include "drake/planning/trajectory_optimization/direct_transcription.h"
// Deprecated includes from original include file.
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using TimeStep DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use drake::planning::trajectory_optimization::TimeStep instead.") =
    drake::planning::trajectory_optimization::TimeStep;

using DirectTranscription DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use drake::planning::trajectory_optimization::DirectTranscription "
    "instead.") = drake::planning::trajectory_optimization::DirectTranscription;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
