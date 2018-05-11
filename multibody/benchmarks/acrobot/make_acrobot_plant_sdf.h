#pragma once

#include <memory>
#include <string>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

/// This method makes a MultibodyPlant model for an acrobot SDF model.
/// @throws std::runtime_error if the parsing the acrobot SDF model fails.
std::unique_ptr<multibody_plant::MultibodyPlant<double>> MakeAcrobotPlantSdf();
}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
