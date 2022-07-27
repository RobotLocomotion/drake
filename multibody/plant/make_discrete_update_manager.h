#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
std::unique_ptr<DiscreteUpdateManager<T>> MakeDiscreteUpdateManager(
    DiscreteContactSolver contact_solver);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
