#pragma once

#include <memory>

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace schunk_wsg {

/// Create a System for a simulated Schunk WSG 50.
///
/// At present this System is only tested and instantiated for T == `double`.
template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkWsgSystem();

}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
