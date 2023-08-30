#pragma once

#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

/** Iterate over all of the geometries of a scene graph, ensuring that
    all geometries have hydroelastic properties.
    @pre scene_graph is not nullptr.
 */
// template <typename T>
// void Hydroelastize(SceneGraph<T>* scene_graph);

namespace internal {
template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state);
}  // namespace internal

// TODO(rpoyner-tri): gain access to configuration values for hydro properties
// TODO(rpoyner-tri): ignore "tiny bubbles"

}  // namespace geometry
}  // namespace drake
