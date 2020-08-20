#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace multibody {
namespace internal {
namespace parameter_conversion {

// Conversions for SpatialInertia<T>
// A flattened SpatialInertia<T> looks like:
// [0]: mass
// [1]: center_of_mass_x
// [2]: center_of_mass_y
// [3]: center_of_masS_z
// [4]: moment_of_inertia_Ixx
// [5]: moment_of_inertia_Iyy
// [6]: moment_of_inertia_Izz
// [7]: product_of_inertia_Ixy
// [8]: product_of_inertia_Ixz
// [9]: product_of_inertia_Iyz

// Converts a SpatialInertia<T> to a BasicVector<T>
template <typename T>
systems::BasicVector<T> ToBasicVector(const SpatialInertia<T>& spatial_inertia);

// Converts a BasicVector<T> to a SpatialInertia<T>
template <typename T>
SpatialInertia<T> ToSpatialInertia(
    const systems::BasicVector<T>& spatial_inertia_vector);

extern template systems::BasicVector<double> ToBasicVector<double>(
    const SpatialInertia<double>&);
extern template systems::BasicVector<::drake::AutoDiffXd>
ToBasicVector<::drake::AutoDiffXd>(const SpatialInertia<::drake::AutoDiffXd>&);
extern template systems::BasicVector<::drake::symbolic::Expression>
ToBasicVector<::drake::symbolic::Expression>(
    const SpatialInertia<::drake::symbolic::Expression>&);

extern template SpatialInertia<double> ToSpatialInertia<double>(
    const systems::BasicVector<double>&);
extern template SpatialInertia<::drake::AutoDiffXd> ToSpatialInertia<
    ::drake::AutoDiffXd>(const systems::BasicVector<::drake::AutoDiffXd>&);
extern template SpatialInertia<::drake::symbolic::Expression>
ToSpatialInertia<::drake::symbolic::Expression>(
    const systems::BasicVector<::drake::symbolic::Expression>&);

}  // namespace parameter_conversion
}  // namespace internal
}  // namespace multibody
}  // namespace drake
