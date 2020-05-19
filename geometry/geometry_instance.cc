#include "drake/geometry/geometry_instance.h"

#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const math::RigidTransform<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   const std::string& name)
    : id_(GeometryId::get_new_id()),
      X_PG_(X_PG),
      shape_(std::move(shape)) {
  set_name(name);
}

void GeometryInstance::set_name(const std::string& name) {
  name_ = internal::CanonicalizeStringName(name);
  if (name_.empty()) {
    throw std::logic_error("GeometryInstance given the name '" + name +
                           "' which is an empty canonical string");
  }
}

}  // namespace geometry
}  // namespace drake
