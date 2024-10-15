#include "drake/multibody/tree/geometry_spatial_inertia.h"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>

#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using geometry::Mesh;
using geometry::Shape;
using internal::CalcSpatialInertiaResult;

// @tparam MeshType must be either geometry::Mesh or geometry::Convex
CalcSpatialInertiaResult CalcMeshSpatialInertia(const Mesh& mesh,
                                                double density) {
  const auto& extension = mesh.extension();
  if (extension == ".obj") {
    return internal::DoCalcSpatialInertia(
        geometry::ReadObjToTriangleSurfaceMesh(mesh.source(), mesh.scale()),
        density);
  }
  if (extension == ".vtk") {
    return internal::DoCalcSpatialInertia(
        geometry::ConvertVolumeToSurfaceMesh(
            geometry::internal::MakeVolumeMeshFromVtk<double>(mesh)),
        density);
  }
  return fmt::format(
      "CalcSpatialInertia currently only supports .obj or tetrahedral-mesh"
      " .vtk files for Mesh shapes but was given '{}'.",
      mesh.source().description());
}

SpatialInertia<double> MaybeThrow(CalcSpatialInertiaResult result) {
  if (std::holds_alternative<std::string>(result)) {
    throw std::invalid_argument(std::get<std::string>(result));
  }
  return std::get<SpatialInertia<double>>(result);
}

}  // namespace

namespace internal {

CalcSpatialInertiaResult DoCalcSpatialInertia(const geometry::Shape& shape,
                                              double density) {
  return shape.Visit<CalcSpatialInertiaResult>(overloaded{
      [density](const geometry::Box& box) {
        return SpatialInertia<double>::SolidBoxWithDensity(
            density, box.width(), box.depth(), box.height());
      },
      [density](const geometry::Capsule& capsule) {
        return SpatialInertia<double>::SolidCapsuleWithDensity(
            density, capsule.radius(), capsule.length(), Vector3d::UnitZ());
      },
      [density](const geometry::Convex& convex) {
        // Note: if converting from poly to tri proves to be an unbearable cost,
        // we can skip the explicit conversion, and tessellate polygonal faces
        // implicitly as we compute spatial inertia.
        return DoCalcSpatialInertia(
            geometry::internal::MakeTriangleFromPolygonMesh(
                convex.GetConvexHull()),
            density);
      },
      [density](const geometry::Cylinder& cylinder) {
        return SpatialInertia<double>::SolidCylinderWithDensity(
            density, cylinder.radius(), cylinder.length(), Vector3d::UnitZ());
      },
      [density](const geometry::Ellipsoid& ellipsoid) {
        return SpatialInertia<double>::SolidEllipsoidWithDensity(
            density, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
      },
      [](const geometry::HalfSpace&) -> CalcSpatialInertiaResult {
        return std::string(
            "CalcSpatialInertia: Cannot compute mass of a HalfSpace");
      },
      [density](const geometry::Mesh& mesh) {
        return CalcMeshSpatialInertia(mesh, density);
      },
      [](const geometry::MeshcatCone&) -> CalcSpatialInertiaResult {
        return std::string(
            "CalcSpatialInertia: MeshcatCone is not yet supported");
      },
      [density](const geometry::Sphere& sphere) {
        return SpatialInertia<double>::SolidSphereWithDensity(density,
                                                              sphere.radius());
      }});
}

CalcSpatialInertiaResult DoCalcSpatialInertia(
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
  // A valid mesh should have an inherently positive volume.  Emit an error if
  // volume is negative or nearly zero. This test of "reasonably positive"
  // volume is more stringent than the mass ≥ 0 test in
  // SpatialInertia::IsPhysicallyValid(). Reminder: The volume of a mesh can be
  // calculated (and should be positive) whereas spatial inertia does not deal
  // with volume. Instead, spatial inertia deals with mass, including idealized
  // zero volume massive objects such as particles, rods, and plates.  Note: If
  // we skip emitting an error for negative or zero volume, a different error
  // would still be otherwise emitted in code called by the spatial inertia
  // constructor below and this function still fails.  For negative volume, the
  // associated less-helpful spatial inertia error message would be e.g., "mass
  // = -0.5 is not positive and finite.", whereas a zero volume creates a
  // divide-by-zero in two places below and the associated obscure error
  // message would be "Unable to calculate the eigenvalues or eigenvectors of
  // the 3x3 matrix associated with a RotationalInertia.".  Related: issue
  // #21924 [github.com/RobotLocomotion/drake/issues/21924].
  constexpr double kEpsilon = 1.0E-14;  // ≈ 64*numeric_limits<double>::epsilon
  if (volume <= kEpsilon) {
    // TODO(Mitiguy) Consider changing the function signature to add an optional
    //  mesh_name argument (e.g., mesh_name = someFilename.obj) and using
    //  mesh_name in the following error_message. For a simulation involving
    //  many meshes, this helps quickly identify an offending mesh.
    // TODO(Mitiguy) Sean Curtis thought the following error message may not be
    //  enough guidance, so he suggested adding a hyperlink in error_message to
    //  https://drake.mit.edu/troubleshooting.html for a proper treatment of
    //  the issue (ideally with Sean's input/expertise).
    const std::string error_message = fmt::format(
        "{}(): The calculated volume of a triangle surface mesh is {} whereas "
        "a reasonable positive value of at least {} was expected. The mesh may "
        "have bad geometry, e.g., it is an open mesh or the winding (order of "
        "vertices) of at least one face does not produce an outward normal.",
        __func__, volume, kEpsilon);
    return error_message;
  }

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

  auto result = SpatialInertia<double>{mass, p_GoGcm, G_GGo_G,
                                       /* skip_validity_check = */ true};
  if (!result.IsPhysicallyValid()) {
    return result.CriticizeNotPhysicallyValid();
  }
  return result;
}

}  // namespace internal

SpatialInertia<double> CalcSpatialInertia(const geometry::Shape& shape,
                                          double density) {
  return MaybeThrow(internal::DoCalcSpatialInertia(shape, density));
}

SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density) {
  return MaybeThrow(internal::DoCalcSpatialInertia(mesh, density));
}

}  // namespace multibody
}  // namespace drake
