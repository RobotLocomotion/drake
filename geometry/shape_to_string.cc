#include "drake/geometry/shape_to_string.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {

void ShapeToString::ImplementGeometry(const Sphere& sphere, void*) {
  string_ = fmt::format("Sphere(r: {})", sphere.radius());
}

void ShapeToString::ImplementGeometry(const Cylinder& cylinder, void*) {
  string_ = fmt::format("Cylinder(r: {}, l: {})", cylinder.radius(),
                        cylinder.length());
}

void ShapeToString::ImplementGeometry(const HalfSpace&, void*) {
  string_ = "Halfspace";
}

void ShapeToString::ImplementGeometry(const Box& box, void*) {
  string_ = fmt::format("Box(w: {}, d: {}, h: {})", box.width(), box.depth(),
                        box.height());
}

void ShapeToString::ImplementGeometry(const Capsule& capsule, void*) {
  string_ = fmt::format("Capsule(r: {}, l: {})", capsule.radius(),
                        capsule.length());
}

void ShapeToString::ImplementGeometry(const Ellipsoid& ellipsoid, void*) {
  string_ = fmt::format("Ellipsoid(a: {}, b: {}, c: {})", ellipsoid.a(),
                        ellipsoid.b(), ellipsoid.c());
}

void ShapeToString::ImplementGeometry(const Mesh& mesh, void*) {
  string_ = fmt::format("Mesh(s: {}, path: {})", mesh.scale(), mesh.filename());
}

void ShapeToString::ImplementGeometry(const Convex& convex, void*) {
  string_ =
      fmt::format("Convex(s: {}, path: {})", convex.scale(), convex.filename());
}

}  // namespace geometry
}  // namespace drake
