#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/multibody/plant/multibody_plant.h"

namespace anzu {
namespace robot_bridge {

// Creates a DofMask-compatible plant with 7 DoFs:
// - 1 DoFs belongs to default model instance
// - 1 DoFs belongs to main model instance
// - 5 DoFs belongs to another model; however, the outboard bodies are
//   influenced by the main model instance.
std::pair<
    std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
    drake::multibody::ModelInstanceIndex
>
MakeDummyPlant();

// Creates a DofMask-incompatible plant.
std::unique_ptr<drake::multibody::MultibodyPlant<double>>
MakeDummyFloatingBodyPlant();

}  // namespace robot_bridge
}  // namespace anzu
