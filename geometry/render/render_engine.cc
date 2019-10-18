#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;

std::unique_ptr<RenderEngine> RenderEngine::Clone() const {
  return std::unique_ptr<RenderEngine>(DoClone());
}

bool RenderEngine::RegisterVisual(
    GeometryId id, const drake::geometry::Shape& shape,
    const PerceptionProperties& properties,
    const RigidTransformd& X_WG, bool needs_updates) {
  // TODO(SeanCurtis-TRI): Test that the id hasn't already been used.
  const bool accepted = DoRegisterVisual(id, shape, properties, X_WG);
  if (accepted) {
    if (needs_updates) {
      update_ids_.insert(id);
    } else {
      anchored_ids_.insert(id);
    }
  }
  return accepted;
}

bool RenderEngine::RemoveGeometry(GeometryId id) {
  const bool removed = DoRemoveGeometry(id);
  // The derived sub-class should report geometry removal if and only if the
  // base class is tracking the id.
  if (removed) {
    DRAKE_DEMAND(update_ids_.erase(id) > 0 || anchored_ids_.erase(id) > 0);
  } else {
    DRAKE_DEMAND(update_ids_.count(id) == 0 || anchored_ids_.count(id) == 0);
  }
  return removed;
}

bool RenderEngine::has_geometry(GeometryId id) const {
  return update_ids_.count(id) > 0 || anchored_ids_.count(id) > 0;
}

RenderLabel RenderEngine::GetRenderLabelOrThrow(
    const PerceptionProperties& properties) const {
  RenderLabel label =
      properties.GetPropertyOrDefault("label", "id", default_render_label_);
  if (label == RenderLabel::kUnspecified || label == RenderLabel::kEmpty) {
    throw std::logic_error(
        "Cannot register a geometry with the 'unspecified' or 'empty' render "
        "labels. The bad label may have come from a default-constructed "
        "RenderLabel or the RenderEngine may have provided it as a default for "
        "missing render labels in the properties.");
  }
  return label;
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
