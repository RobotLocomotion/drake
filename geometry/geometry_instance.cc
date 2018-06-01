#include "drake/geometry/geometry_instance.h"

#include <utility>

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape)
    : GeometryInstance(X_PG, std::move(shape), VisualMaterial()) {}

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   const VisualMaterial& vis_material)
    : id_(GeometryId::get_new_id()),
      X_PG_(X_PG),
      shape_(std::move(shape)),
      visual_material_(vis_material) {}

}  // namespace geometry
}  // namespace drake
