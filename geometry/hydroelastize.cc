#include "drake/geometry/hydroelastize.h"

namespace drake {
namespace geometry {

template <typename T>
void Hydroelastize(SceneGraph<T>* scene_graph) {
  unused(scene_graph);
  // Iterate over all geometries.
    // Get current proximity properties (if any).
    // Update proximity properties to a suitable hydro set (requires Reifier).
    // AssignRole(kReplace).
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&Hydroelastize<T>));

}  // namespace geometry
}  // namespace drake
