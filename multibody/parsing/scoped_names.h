#pragma once

#include <string>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Finds an optionally model-scoped frame.
///
/// Returns `nullptr` if the frame is not found, as well as all the error
/// cases of `MultibodyPlant::HasFrameByName(std::string)`.
///
/// @tparam_default_scalar
template <typename T>
const drake::multibody::Frame<T>* GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& full_name);

/// Equivalent to `GetScopedFrameByNameMaybe`, but throws if the frame
/// is not found.
///
/// @tparam_default_scalar
template <typename T>
const drake::multibody::Frame<T>& GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& full_name);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
