#pragma once

// TODO(jwnimmer-tri) This file provides trajectories in support of demos.
// This data should come from files loaded at runtime, instead.

#include <memory>
#include <tuple>

#include "drake/automotive/curve2.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace automotive {

/**
 * Creates TrajectoryCar constructor demo arguments.  The details of the
 * trajectory are not documented / promised by this API.
 *
 * @param index Selects which pre-programmed trajectory to use.
 * @return tuple of curve, speed, start_time
 */
DRAKE_EXPORT
std::tuple<Curve2<double>, double, double> CreateTrajectoryParams(int index);

}  // namespace automotive
}  // namespace drake
