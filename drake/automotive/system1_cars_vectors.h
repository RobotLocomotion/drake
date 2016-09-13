#pragma once

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/system1_vector.h"

namespace drake {
namespace cars {

// TODO(jwnimmer-tri) These renamings are temporary, in order to support the
// incremental porting of Cars from System1 to System2.

template <typename T>
using DrivingCommand1 = class System1Vector<DrivingCommand<T>, T>;
template <typename T>
using EulerFloatingJointState1 =
    class System1Vector<EulerFloatingJointState<T>, T>;
template <typename T>
using SimpleCarState1 = class System1Vector<SimpleCarState<T>, T>;

}  // namespace cars
}  // namespace drake
