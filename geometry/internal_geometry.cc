#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {
namespace internal {

InternalGeometry::InternalGeometry(
    SourceId source_id, std::unique_ptr<Shape> shape, FrameId frame_id,
    GeometryId geometry_id, std::string name, const Isometry3<double>& X_FG,
    FrameIndex index, const VisualMaterial& material)
    : shape_spec_(std::move(shape)),
      id_(geometry_id),
      name_(std::move(name)),
      index_(index),
      source_id_(source_id),
      frame_id_(frame_id),
      X_PG_(X_FG),
      X_FG_(X_FG),
      parent_geometry_id_(nullopt),
      visual_material_(material) {}


}  // namespace internal
}  // namespace geometry
}  // namespace drake
