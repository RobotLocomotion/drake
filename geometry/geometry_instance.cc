#include "drake/geometry/geometry_instance.h"

#include <utility>

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   std::string name)
    : GeometryInstance(X_PG, std::move(shape), std::move(name),
                       VisualMaterial()) {}

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   std::string name,
                                   const VisualMaterial& vis_material)
    : id_(GeometryId::get_new_id()),
      X_PG_(X_PG),
      shape_(std::move(shape)),
      name_(std::move(name)),
      visual_material_(vis_material) {}

}  // namespace geometry
}  // namespace drake
