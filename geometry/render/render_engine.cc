#include "drake/geometry/render/render_engine.h"

#include <typeinfo>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;

std::unique_ptr<RenderEngine> RenderEngine::Clone() const {
  std::unique_ptr<RenderEngine> clone(DoClone());
  // Make sure that derived classes have actually overridden DoClone().
  // Particularly important for derivations of derivations.
  // Note: clang considers typeid(*clone) to be an expression with side effects.
  // So, we capture a reference to the polymorphic type and provide that to
  // typeid to make both clang and gcc happy.
  const RenderEngine& clone_ref = *clone;
  if (typeid(*this) != typeid(clone_ref)) {
    throw std::logic_error(fmt::format(
        "Error in cloning RenderEngine class of type {}; the clone returns "
        "type {}. {}::DoClone() was probably not implemented",
        NiceTypeName::Get(*this), NiceTypeName::Get(clone_ref),
        NiceTypeName::Get(*this)));
  }
  return clone;
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

void RenderEngine::SetDefaultLightPosition(const Vector3<double>&) {}

}  // namespace render
}  // namespace geometry
}  // namespace drake
