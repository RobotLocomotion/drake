#pragma once

// @file
// The purpose of this header and its corresponding cc file is to reduce the
// cycle time for edit-compile-test as we develop new functionalities for
// discrete updates. This header only depends on discrete_update_manager.h which
// contains the (rarely modified) virtual base class, DiscreteUpdateManager,
// whereas the cc file depends on concrete derived classes such as
// CompliantContactManager that are currently under development. By including
// this header (instead of directly including headers of concrete derived
// classes) in multibody_plant.cc, we avoid recompiling MultibodyPlant every
// time some concrete update manager changes. Instead, we only need to relink
// them.

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

/* Constructs and returns a suitable discrete update manager for a
 MultibodyPlant given the type of contact solver.
 @tparam_default_scalar */
template <typename T>
std::unique_ptr<DiscreteUpdateManager<T>> MakeDiscreteUpdateManager(
    DiscreteContactSolver contact_solver);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
