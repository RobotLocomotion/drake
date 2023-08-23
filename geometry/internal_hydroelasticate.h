#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {
namespace internal {

/* Iterates over all of the geometries of a scene graph, ensuring that
   all geometries have hydroelastic properties.
   @pre geometry_state is not nullptr.
   @tparam_default_scalar
 */
template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config);

/* Ensures that one geometry has hydroelastic properties.
   @pre geometry_state is not nullptr.
   @pre geometry_id indicates a geometry with an assigned proximity role.
   @tparam_default_scalar
 */
template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                     const SceneGraphConfig& config, GeometryId geometry_id);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
