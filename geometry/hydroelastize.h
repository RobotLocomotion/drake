#pragma once

#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

/** Iterate over all of the geometries of a scene graph, ensuring that
    all geometries have hydroelastic properties.
    @pre scene_graph is not nullptr.
 */
template <typename T>
void Hydroelastize(SceneGraph<T>* scene_graph);

}  // namespace geometry
}  // namespace drake
