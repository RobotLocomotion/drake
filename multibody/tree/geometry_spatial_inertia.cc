#include "drake/multibody/tree/geometry_spatial_inertia.h"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>

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
using geometry::ShapeReifier;
using std::pow;

// SpatialInertia semantics and notation are documented in spatial_inertia.h.
class SpatialInertiaCalculator final : public ShapeReifier {
 public:
  SpatialInertia<double> Calculate(const Shape& shape, double density) {
    density_ = density;
    shape.Reify(this);
    return spatial_inertia_;
  }

  SpatialInertia<double> spatial_inertia() const { return spatial_inertia_; }

 private:
  void ImplementGeometry(const geometry::Box& box, void*) final {
    spatial_inertia_ = SpatialInertia<double>::SolidBoxWithDensity(
        density_, box.width(), box.depth(), box.height());
  }

  void ImplementGeometry(const geometry::Capsule& capsule, void*) final {
    spatial_inertia_ = SpatialInertia<double>::SolidCapsuleWithDensity(
        density_, capsule.radius(), capsule.length(), Vector3d::UnitZ());
  }

  void ImplementGeometry(const geometry::Convex& convex, void*) final {
    ImplementMesh(convex);
  }

  void ImplementGeometry(const geometry::Cylinder& cylinder, void*) final {
    spatial_inertia_ = SpatialInertia<double>::SolidCylinderWithDensity(
        density_, cylinder.radius(), cylinder.length(), Vector3d::UnitZ());
  }

  void ImplementGeometry(const geometry::Ellipsoid& ellipsoid, void*) final {
    spatial_inertia_ = SpatialInertia<double>::SolidEllipsoidWithDensity(
        density_, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  }

  void ImplementGeometry(const geometry::HalfSpace&, void*) final {
    throw std::logic_error(
        "Cannot compute mass properties for an infinite volume: HalfSpace");
  }

  void ImplementGeometry(const geometry::Mesh& mesh, void*) final {
    ImplementMesh(mesh);
  }

  void ImplementGeometry(const geometry::MeshcatCone&, void*) final {
    throw std::logic_error(
        "Cannot compute mass properties for the visualization-only "
        "MeshcatCone");
  }

  void ImplementGeometry(const geometry::Sphere& sphere, void*) final {
    spatial_inertia_ = SpatialInertia<double>::SolidSphereWithDensity(
        density_, sphere.radius());
  }

  template <typename MeshType>
  void ImplementMesh(const MeshType& mesh) {
    const auto& extension = mesh.extension();
    std::unique_ptr<geometry::TriangleSurfaceMesh<double>> surface_mesh;
    if (extension == ".obj") {
      surface_mesh = std::make_unique<geometry::TriangleSurfaceMesh<double>>(
          geometry::ReadObjToTriangleSurfaceMesh(mesh.filename(),
                                                 mesh.scale()));
    } else if (extension == ".vtk") {
      surface_mesh = std::make_unique<geometry::TriangleSurfaceMesh<double>>(
          geometry::ConvertVolumeToSurfaceMesh(
              geometry::internal::MakeVolumeMeshFromVtk<double>(mesh)));
    } else {
      throw std::runtime_error(fmt::format(
          "CalcSpatialInertia currently only supports .obj or tetrahedral-mesh"
          " .vtk files for mesh geometries but was given '{}'.",
          mesh.filename()));
    }

    spatial_inertia_ = CalcSpatialInertia(*surface_mesh, density_);
  }

  double density_{};
  SpatialInertia<double> spatial_inertia_;
};

}  // namespace

SpatialInertia<double> CalcSpatialInertia(const geometry::Shape& shape,
                                          double density) {
  SpatialInertiaCalculator calculator;
  calculator.Calculate(shape, density);
  return calculator.spatial_inertia();
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
