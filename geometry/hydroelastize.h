#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {
namespace internal {

/** Iterate over all of the geometries of a scene graph, ensuring that
    all geometries have hydroelastic properties.
    @pre geometry_state is not nullptr.
 */
template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config);

/** Ensure that one geometry has hydroelastic properties.
    @pre geometry_state is not nullptr.
 */
template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config, GeometryId geometry_id);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
