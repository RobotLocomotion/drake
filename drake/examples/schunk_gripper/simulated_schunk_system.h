#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace schunk_gripper {

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkSystem();

}  // namespace schunk_gripper
}  // namespace examples
}  // namespace drake
