// For now, this is an empty .cc file that only serves to confirm
// parameter_conversion.h is a stand-alone header.

#include "drake/multibody/tree/parameter_conversion.h"

namespace drake {
namespace multibody {
namespace internal {
namespace parameter_conversion {

template <typename T>
systems::BasicVector<T> ToBasicVector(
    const SpatialInertia<T>& spatial_inertia) {
  const Vector3<T>& com = spatial_inertia.get_com();
  const UnitInertia<T>& unit_inertia = spatial_inertia.get_unit_inertia();
  return systems::BasicVector<T>(
      {
      // mass
      spatial_inertia.get_mass(),
      // center of mass
      com(0), com(1), com(2),
      // unit inertia
      unit_inertia(0, 0), unit_inertia(1, 1), unit_inertia(2, 2),
      unit_inertia(1, 0), unit_inertia(2, 0), unit_inertia(2, 1)});
}

template <typename T>
SpatialInertia<T> ToSpatialInertia(
    const systems::BasicVector<T>& spatial_inertia_vector) {
  DRAKE_DEMAND(spatial_inertia_vector.size() == 10);
  return SpatialInertia<T>(
      spatial_inertia_vector[0],
      Vector3<T>(spatial_inertia_vector[1], spatial_inertia_vector[2],
                 spatial_inertia_vector[3]),
      UnitInertia<T>(spatial_inertia_vector[4], spatial_inertia_vector[5],
                     spatial_inertia_vector[6], spatial_inertia_vector[7],
                     spatial_inertia_vector[8], spatial_inertia_vector[9]));
}

template systems::BasicVector<double> ToBasicVector<double>(
    const SpatialInertia<double>&);
template systems::BasicVector<::drake::AutoDiffXd>
ToBasicVector<::drake::AutoDiffXd>(const SpatialInertia<::drake::AutoDiffXd>&);
template systems::BasicVector<::drake::symbolic::Expression>
ToBasicVector<::drake::symbolic::Expression>(
    const SpatialInertia<::drake::symbolic::Expression>&);

template SpatialInertia<double> ToSpatialInertia<double>(
    const systems::BasicVector<double>&);
template SpatialInertia<::drake::AutoDiffXd> ToSpatialInertia<
    ::drake::AutoDiffXd>(const systems::BasicVector<::drake::AutoDiffXd>&);
template SpatialInertia<::drake::symbolic::Expression>
ToSpatialInertia<::drake::symbolic::Expression>(
    const systems::BasicVector<::drake::symbolic::Expression>&);

}  // namespace parameter_conversion
}  // namespace internal
}  // namespace multibody
}  // namespace drake
