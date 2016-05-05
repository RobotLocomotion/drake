#pragma once

#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/examples/Cars/gen/euler_floating_joint_state.h"
#include "drake/examples/Cars/gen/simple_car_state.h"
#include "drake/systems/vector.h"

namespace drake {

// TODO(jwnimmer-tri) These renamings are temporary, in order to support the
// incremental porting of Cars from System1 to System2.

template <typename T>
using DrivingCommand1 = class DrivingCommand<T>;
template <typename T>
using EulerFloatingJointState1 = EulerFloatingJointState<T>;
template <typename T>
using SimpleCarState1 = class SimpleCarState<T>;

}  // namespace drake
