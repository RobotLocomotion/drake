#include "drake/geometry/internal_geometry.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

InternalGeometry::InternalGeometry(
    SourceId source_id, std::unique_ptr<Shape> shape, FrameId frame_id,
    GeometryId geometry_id, std::string name, const Isometry3<double>& X_FG,
    GeometryIndex index)
    : shape_spec_(std::move(shape)),
      id_(geometry_id),
      name_(std::move(name)),
      index_(index),
      source_id_(source_id),
      frame_id_(frame_id),
      X_PG_(X_FG),
      X_FG_(X_FG),
      parent_geometry_id_(nullopt) {}

bool InternalGeometry::has_role(Role role) const {
  switch (role) {
    case Role::kProximity:
      return has_proximity_role();
    case Role::kIllustration:
      return has_illustration_role();
    case Role::kPerception:
      throw std::logic_error("Unsupported internal geometry role: perception");
    case Role::kUnassigned:
      return !(has_proximity_role() || has_illustration_role());
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
