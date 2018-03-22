#pragma once

#include <string>
#include <sdf/sdf.hh>
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

/// This method makes a MultibodyPlant model for a acrobot SDF model defined
/// in acrobot_sdf.h.
///
/// @param[in] finalize
///   If `true`, MultibodyPlant::Finalize() gets called on the new plant.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlantSdf(bool finalize);
}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
