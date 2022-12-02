#include "drake/multibody/tree/geometry_spatial_inertia.h"

#include <algorithm>
#include <filesystem>
#include <string>

#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using geometry::Shape;
using geometry::ShapeReifier;
using std::pow;

/* The spatial inertia documented in the header file. See documentation there
 for semantics and notation. */
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
    const double volume = box.width() * box.depth() * box.height();
    const double mass = volume * density_;
    const Vector3d p_SoScm_S = Vector3d::Zero();
    const UnitInertia<double> G_SSo_S =
        UnitInertia<double>::SolidBox(box.width(), box.depth(), box.height());
    spatial_inertia_ = SpatialInertia<double>{mass, p_SoScm_S, G_SSo_S};
  }

  void ImplementGeometry(const geometry::Capsule& capsule, void*) final {
    const double volume = (M_PI * 4.0 / 3.0 * pow(capsule.radius(), 3) +
                           M_PI * pow(capsule.radius(), 2) * capsule.length());
    const double mass = volume * density_;
    const Vector3d p_SoScm_S = Vector3d::Zero();
    const UnitInertia<double> G_SSo_S =
        UnitInertia<double>::SolidCapsule(capsule.radius(), capsule.length());
    spatial_inertia_ = SpatialInertia<double>{mass, p_SoScm_S, G_SSo_S};
  }

  void ImplementGeometry(const geometry::Convex& convex, void*) final {
    ImplementMesh(convex);
  }

  void ImplementGeometry(const geometry::Cylinder& cylinder, void*) final {
    const double volume = M_PI * pow(cylinder.radius(), 2) * cylinder.length();
    const double mass = volume * density_;
    const Vector3d p_SoScm_S = Vector3d::Zero();
    const UnitInertia<double> G_SSo_S = UnitInertia<double>::SolidCylinder(
        cylinder.radius(), cylinder.length());
    spatial_inertia_ = SpatialInertia<double>{mass, p_SoScm_S, G_SSo_S};
  }

  void ImplementGeometry(const geometry::Ellipsoid& ellipsoid, void*) final {
    const double volume =
        4.0 * M_PI / 3.0 * ellipsoid.a() * ellipsoid.b() * ellipsoid.c();
    const double mass = volume * density_;
    const Vector3d p_SoScm_S = Vector3d::Zero();
    const UnitInertia<double> G_SSo_S = UnitInertia<double>::SolidEllipsoid(
        ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
    spatial_inertia_ = SpatialInertia<double>{mass, p_SoScm_S, G_SSo_S};
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
    const double volume = 4.0 * M_PI / 3.0 * pow(sphere.radius(), 3);
    const double mass = volume * density_;
    const Vector3d p_SoScm_S = Vector3d::Zero();
    const UnitInertia<double> G_SSo_S =
        UnitInertia<double>::SolidSphere(sphere.radius());
    spatial_inertia_ = SpatialInertia<double>{mass, p_SoScm_S, G_SSo_S};
  }

  template <typename MeshType>
  void ImplementMesh(const MeshType& mesh) {
    std::string extension = std::filesystem::path(mesh.filename()).extension();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    // TODO(russt): Support .vtk files.
    if (extension != ".obj") {
      throw std::runtime_error(fmt::format(
          "CalcMassProperties currently only supports .obj files for mesh "
          "geometries but was given '{}'.",
          mesh.filename()));
    }
    const geometry::TriangleSurfaceMesh<double> surface_mesh =
        geometry::ReadObjToTriangleSurfaceMesh(mesh.filename(), mesh.scale());

    spatial_inertia_ = CalcSpatialInertia(surface_mesh, density_);
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
