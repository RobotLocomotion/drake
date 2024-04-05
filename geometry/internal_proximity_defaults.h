#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {
namespace internal {

/* Applies the default proximity values in `defaults` to the proximity
   properties of the geometries in geometry_state. For detailed semantics, see
   the 3-argument overload.

   @pre geometry_state is not nullptr.
   @tparam_default_scalar
 */
template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const DefaultProximityProperties& defaults);

/* Applies the default proximity values in `defaults` to the proximity
   properties of the geometry with the given geometry_id in geometry_state as
   appropriate. For a given property, no value will be written if (a)
   `defaults` contains no value for it, or (b) the geometry's proximity
   property already contains a value.

   @pre geometry_state is not nullptr.
   @pre geometry_id indicates a geometry with an assigned proximity role.
   @tparam_default_scalar
 */
template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const DefaultProximityProperties& defaults,
                            GeometryId geometry_id);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
