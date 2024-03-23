#include "drake/multibody/tree/geometry_spatial_inertia.h"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>

#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using geometry::Shape;

// @tparam MeshType must be either geometry::Mesh or geometry::Convex
template <typename MeshType>
SpatialInertia<double> CalcMeshSpatialInertia(const MeshType& mesh,
                                              double density) {
  const auto& extension = mesh.extension();
  if (extension == ".obj") {
    return CalcSpatialInertia(
        geometry::ReadObjToTriangleSurfaceMesh(mesh.filename(), mesh.scale()),
        density);
  }
  if (extension == ".vtk") {
    return CalcSpatialInertia(
        geometry::ConvertVolumeToSurfaceMesh(
            geometry::internal::MakeVolumeMeshFromVtk<double>(mesh)),
        density);
  }
  throw std::runtime_error(fmt::format(
      "CalcSpatialInertia currently only supports .obj or tetrahedral-mesh"
      " .vtk files for mesh geometries but was given '{}'.",
      mesh.filename()));
}

}  // namespace

SpatialInertia<double> CalcSpatialInertia(const geometry::Shape& shape,
                                          double density) {
  return shape.Visit<SpatialInertia<double>>(overloaded{
      [density](const geometry::Box& box) {
        return SpatialInertia<double>::SolidBoxWithDensity(
            density, box.width(), box.depth(), box.height());
      },
      [density](const geometry::Capsule& capsule) {
        return SpatialInertia<double>::SolidCapsuleWithDensity(
            density, capsule.radius(), capsule.length(), Vector3d::UnitZ());
      },
      [density](const geometry::Convex& convex) {
        return CalcMeshSpatialInertia(convex, density);
      },
      [density](const geometry::Cylinder& cylinder) {
        return SpatialInertia<double>::SolidCylinderWithDensity(
            density, cylinder.radius(), cylinder.length(), Vector3d::UnitZ());
      },
      [density](const geometry::Ellipsoid& ellipsoid) {
        return SpatialInertia<double>::SolidEllipsoidWithDensity(
            density, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
      },
      [](const geometry::HalfSpace&) -> SpatialInertia<double> {
        throw std::logic_error(
            "CalcSpatialInertia: Cannot compute mass of a HalfSpace");
      },
      [density](const geometry::Mesh& mesh) {
        return CalcMeshSpatialInertia(mesh, density);
      },
      [](const geometry::MeshcatCone&) -> SpatialInertia<double> {
        throw std::logic_error(
            "CalcSpatialInertia: MeshcatCone is not yet supported");
      },
      [density](const geometry::Sphere& sphere) {
        return SpatialInertia<double>::SolidSphereWithDensity(density,
                                                              sphere.radius());
      }});
}

SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density) {
  /* This algorithm is based on:
   - https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
   - http://number-none.com/blow/inertia/bb_inertia.doc
  */
  // The co-variance matrix of a canonical tetrahedron with vertices at
  // (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1) with an assumption of *unit*
  // density.
  Matrix3<double> C_canonical;
  // clang-format off
  C_canonical << 1 / 60.0,  1 / 120.0, 1 / 120.0,
                 1 / 120.0, 1 / 60.0,  1 / 120.0,
                 1 / 120.0, 1 / 120.0, 1 / 60.0;
  // clang-format on

  // We'll accumulate volume, center of mass, and inertia. The quantities
  // we accumulate will be off by a factor of six to reduce operations.
  double vol_times_six = 0.0;
  Vector3d accum_com = Vector3d::Zero();
  Matrix3<double> C = Matrix3<double>::Zero();
  // The *transpose* of the affine transformation that takes us from the
  // canonical co-variance matrix to the matrix for the particular tet.
  Matrix3<double> A_T = Matrix3<double>::Zero();
  for (const auto& tri : mesh.triangles()) {
    const Vector3d& p = mesh.vertex(tri.vertex(0));
    const Vector3d& q = mesh.vertex(tri.vertex(1));
    const Vector3d& r = mesh.vertex(tri.vertex(2));
    double tet_vol_times_six = p.cross(q).dot(r);
    A_T.row(0) = p;
    A_T.row(1) = q;
    A_T.row(2) = r;
    // We're computing C += det(A)⋅ACAᵀ. Fortunately, det(A) is equal to 6V.
    C += A_T.transpose() * C_canonical * A_T * tet_vol_times_six;
    vol_times_six += tet_vol_times_six;
    accum_com += (p + q + r) * tet_vol_times_six;
  }

  const double volume = vol_times_six / 6.0;
  const double mass = density * volume;
  const Vector3d p_GoGcm = accum_com / (vol_times_six * 4);
  // We can compute I = C.trace * 1₃ - C. Two key points:
  //  1. We don't want I, we want G, the unit inertia. Our computation of C is
  //     *mass* weighted with an implicit assumption of unit density. So, to
  //     make it a *unit* inertia, we must divide by mass = ρV = 1 * V = V.
  //  2. G is symmetric, so we'll forego doing the full matrix multiplication
  //     and go get the six terms we actually care about.
  C /= volume;
  const double trace_C = C.trace();
  const UnitInertia G_GGo_G(trace_C - C(0, 0), trace_C - C(1, 1),
                            trace_C - C(2, 2), -C(1, 0), -C(2, 0), -C(2, 1));
  return SpatialInertia<double>{mass, p_GoGcm, G_GGo_G};
}

}  // namespace multibody
}  // namespace drake
