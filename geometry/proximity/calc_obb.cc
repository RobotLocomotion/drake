#include "drake/geometry/proximity/calc_obb.h"

#include <set>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

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

namespace internal {

Obb MakeObb(const MeshSource& mesh_source, const Vector3d& scale) {
  if (mesh_source.extension() == ".obj") {
    // For OBJ files, create a TriangleSurfaceMesh and use ObbMaker.
    TriangleSurfaceMesh<double> surface_mesh =
        ReadObjToTriangleSurfaceMesh(mesh_source, scale);
    // Create a set containing all vertex indices.
    std::set<int> all_vertices;
    for (int i = 0; i < surface_mesh.num_vertices(); ++i) {
      all_vertices.insert(i);
    }
    ObbMaker<TriangleSurfaceMesh<double>> obb_maker(surface_mesh, all_vertices);
    return obb_maker.Compute();
  } else if (mesh_source.extension() == ".vtk") {
    // For VTK files, create a VolumeMesh and use ObbMaker.
    const VolumeMesh<double> volume_mesh =
        ReadVtkToVolumeMesh(mesh_source, scale);
    // Create a set containing all vertex indices.
    std::set<int> all_vertices;
    for (int i = 0; i < volume_mesh.num_vertices(); ++i) {
      all_vertices.insert(i);
    }
    ObbMaker<VolumeMesh<double>> obb_maker(volume_mesh, all_vertices);
    return obb_maker.Compute();
  } else if (mesh_source.extension() == ".gltf") {
    // For glTF files, we create the convex hull of the mesh and then compute
    // the OBB of the convex hull so that we can reuse existing functions.
    const PolygonSurfaceMesh<double> polygon_mesh =
        MakeConvexHull(mesh_source, scale);
    std::set<int> all_vertices;
    for (int i = 0; i < polygon_mesh.num_vertices(); ++i) {
      all_vertices.insert(i);
    }
    ObbMaker<PolygonSurfaceMesh<double>> obb_maker(polygon_mesh, all_vertices);
    return obb_maker.Compute();
  } else {
    throw std::runtime_error(
        fmt::format("MakeObb only applies to .obj, .vtk, and .gltf meshes; "
                    "unsupported extension '{}' for geometry data: {}.",
                    mesh_source.extension(), mesh_source.description()));
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
