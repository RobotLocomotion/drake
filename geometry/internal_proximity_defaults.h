#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {
namespace internal {

/* Iterates over all of the proximity geometries of a scene graph, ensuring
   that they all have default values for properties filled in. For detailed
   semantics, see the 3-argument overload.

   @pre geometry_state is not nullptr.
   @tparam_default_scalar
 */
template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const SceneGraphConfig& config);

/* Ensures that one proximity geometry has default values filled in. The values
   (if present) are supplied via the `config.default_proximity_properties`
   struct. If any proximity property is already present, its value will be
   unchanged.

   @pre geometry_state is not nullptr.
   @pre geometry_id indicates a geometry with an assigned proximity role.
   @tparam_default_scalar
 */
template <typename T>
void ApplyProximityDefaults(GeometryState<T>* geometry_state,
                            const SceneGraphConfig& config,
                            GeometryId geometry_id);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
