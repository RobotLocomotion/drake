#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> RenderEngine::Clone() const {
  return std::unique_ptr<RenderEngine>(DoClone());
}

optional<RenderIndex> RenderEngine::RegisterVisual(
    GeometryIndex index, const drake::geometry::Shape& shape,
    const PerceptionProperties& properties,
    const Isometry3<double>& X_WG, bool needs_updates) {
  optional<RenderIndex> render_index =
      DoRegisterVisual(shape, properties, X_WG);
  if (render_index && needs_updates) {
    update_indices_.insert({*render_index, index});
  }
  return render_index;
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
