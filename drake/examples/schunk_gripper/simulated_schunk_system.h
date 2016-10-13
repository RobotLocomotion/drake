#pragma once

#include <memory>

#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace schunk_gripper {

/// Create a System for a simulated Schunk.
///
/// At present this System is only tested and instantiated for T == `double`.
template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkSystem();

}  // namespace schunk_gripper
}  // namespace examples
}  // namespace drake
