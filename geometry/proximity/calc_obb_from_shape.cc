#include "drake/geometry/proximity/calc_obb_from_shape.h"

#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_obb_from_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

using math::RigidTransform;

std::optional<Obb> CalcObb(const Shape& shape) {
  return shape.Visit<std::optional<Obb>>(overloaded{
      [](const Box& box) {
        // For a box, the OBB is aligned with the geometry frame and centered at
        // origin.
        return Obb(RigidTransform<double>::Identity(), box.size() / 2);
      },
      [](const Capsule& capsule) {
        // For a capsule, the OBB is aligned with the geometry frame (z-axis
        // along capsule).
        const double radius = capsule.radius();
        const double half_length = capsule.length() / 2.0;
        const Vector3<double> half_size(radius, radius, half_length + radius);
        return Obb(RigidTransform<double>::Identity(), half_size);
      },
      [](const Convex& convex) {
        return internal::MakeObb(convex.source(), convex.scale3());
      },
      [](const Cylinder& cylinder) {
        // For a cylinder, the OBB is aligned with the geometry frame (z-axis
        // along cylinder).
        const double radius = cylinder.radius();
        const double half_length = cylinder.length() / 2.0;
        const Vector3<double> half_size(radius, radius, half_length);
        return Obb(RigidTransform<double>::Identity(), half_size);
      },
      [](const Ellipsoid& ellipsoid) {
        const Vector3<double> half_size(ellipsoid.a(), ellipsoid.b(),
                                        ellipsoid.c());
        return Obb(RigidTransform<double>::Identity(), half_size);
      },
      [](const HalfSpace&) {
        return std::nullopt;
      },
      [](const Mesh& mesh) {
        return internal::MakeObb(mesh.source(), mesh.scale3());
      },
      [](const MeshcatCone&) {
        return std::nullopt;
      },
      [](const Sphere& sphere) {
        // For a sphere, the OBB is a cube centered at origin.
        const double radius = sphere.radius();
        return Obb(RigidTransform<double>::Identity(),
                   Vector3<double>(radius, radius, radius));
      }});
}

}  // namespace geometry
}  // namespace drake
