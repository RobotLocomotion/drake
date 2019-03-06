#include "drake/geometry/dev/internal_geometry.h"

namespace drake {
namespace geometry {
namespace dev {
namespace internal {

InternalGeometry::InternalGeometry(
    SourceId source_id, std::unique_ptr<Shape> shape, FrameId frame_id,
    GeometryId geometry_id, std::string name, const Isometry3<double>& X_FG,
    InternalIndex index)
    : shape_spec_(std::move(shape)),
      id_(geometry_id),
      name_(std::move(name)),
      internal_index_(index),
      source_id_(source_id),
      frame_id_(frame_id),
      X_PG_(X_FG),
      X_FG_(X_FG),
      parent_geometry_id_(nullopt) {}

bool InternalGeometry::has_role(Role role) const {
  switch (role) {
    case Role::kProximity:
      return has_proximity_role();
    case Role::kPerception:
      return has_perception_role();
    case Role::kIllustration:
      return has_illustration_role();
    case Role::kUnassigned:
      return !(has_proximity_role() || has_perception_role() ||
          has_illustration_role());
    default:
      // THis should never be reached. The switch statement should be exhaustive
      // of all enumeration values.
      DRAKE_DEMAND(false);
      return false;
  }
}

optional<RenderIndex> InternalGeometry::render_index(
    const std::string& renderer_name) const {
  auto iter = render_indices_.find(renderer_name);
  if (iter != render_indices_.end()) return iter->second;
  return {};
}

void InternalGeometry::set_render_index(std::string renderer_name,
                                        RenderIndex index) {
  // Internally, this should never be called with a duplicate renderer name.
  DRAKE_ASSERT(render_indices_.count(renderer_name) == 0);
  render_indices_[move(renderer_name)] = index;
}

}  // namespace internal
}  // namespace dev
}  // namespace geometry
}  // namespace drake
