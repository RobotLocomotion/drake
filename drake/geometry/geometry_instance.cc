#include "drake/geometry/geometry_instance.h"

#include <utility>

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape)
    : X_PG_(X_PG), shape_(std::move(shape)) {}

}  // namespace geometry
}  // namespace drake
