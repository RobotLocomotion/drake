#pragma once

#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {
namespace internal {

/** Iterate over all of the geometries of a scene graph, ensuring that
    all geometries have hydroelastic properties.
    @pre scene_graph is not nullptr.
 */
template <typename T>
void Hydroelastize(GeometryState<T>* geometry_state);
// TODO(rpoyner-tri): gain access to configuration values for hydro properties
// TODO(rpoyner-tri): ignore "tiny bubbles"

}  // namespace internal
}  // namespace geometry
}  // namespace drake
