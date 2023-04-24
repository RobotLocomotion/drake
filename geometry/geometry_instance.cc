#include "drake/geometry/geometry_instance.h"

#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const math::RigidTransform<double>& X_PG,
                                   const Shape& shape, const std::string& name)
    : GeometryInstance(X_PG, shape.Clone(), name) {}

GeometryInstance::GeometryInstance(const math::RigidTransform<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   const std::string& name)
    : id_(GeometryId::get_new_id()), X_PG_(X_PG) {
  DRAKE_THROW_UNLESS(shape != nullptr);
  shape_ = std::move(shape);
  set_name(name);
}

void GeometryInstance::set_name(const std::string& name) {
  name_ = internal::CanonicalizeStringName(name);
  if (name_.empty()) {
    throw std::logic_error(fmt::format(
        "GeometryInstance given the name '{}' which is an empty canonical "
        "string",
        name));
  }
}

}  // namespace geometry
}  // namespace drake
