#include "drake/geometry/proximity/calc_obb.h"

#include <set>

#include "drake/common/eigen_types.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace {

/* Creates an OBB for the polygon surface mesh in its canonical frame. */
Obb CalcObb(const PolygonSurfaceMesh<double>& mesh) {
  std::set<int> all_vertices;
  for (int i = 0; i < mesh.num_vertices(); ++i) {
    all_vertices.insert(i);
  }
  ObbMaker<PolygonSurfaceMesh<double>> obb_maker(mesh, all_vertices);
  return obb_maker.Compute();
}

}  // namespace

using Eigen::Vector3d;
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
        return CalcObb(convex.GetConvexHull());
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
        return CalcObb(mesh.GetConvexHull());
      },
      [](const MeshcatCone& cone) {
        const double half_height = cone.height() / 2.0;
        const Vector3<double> half_size(cone.a(), cone.b(), half_height);
        return Obb(RigidTransform<double>(Vector3<double>(0, 0, half_height)),
                   half_size);
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
