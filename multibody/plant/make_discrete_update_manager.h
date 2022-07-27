#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

/* Constructs and returns a suitable discrete update manager for a
 MultibodyPlant given the type of contact solver. Returns `nullptr` if the
 contact solver type is `kTamsi`.
 @throws an exception if invoked with incompatible contact solver and scalar
 type.
 @tparam_default_scalar */
template <typename T>
std::unique_ptr<DiscreteUpdateManager<T>> MakeDiscreteUpdateManager(
    DiscreteContactSolver contact_solver);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
