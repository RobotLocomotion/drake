#include "drake/geometry/proximity/mesh_to_vtk.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;
using std::unique_ptr;

// We test a subset of all possible objects (VolumeMesh, SurfaceMesh,
// VolumeMeshFieldLinear, SurfaceMeshFieldLinear) created from all primitives
// (sphere, cylinder, box) and contact surfaces as shown as (X) in this table.
//
//                          sphere  cylinder   box    contact_surface
//  VolumeMesh               (X)      (X)      (X)
//  SurfaceMesh                                (X)
//  VolumeMeshFieldLinear                      (X)
//  SurfaceMeshFieldLinear                                 (X)

// TODO(DamrongGuoy): Add ellipsoids and capsules in the test table above.

GTEST_TEST(MeshToVtkTest, SphereTetrahedra) {
  const Sphere sphere(1.0);
  // resolution_hint 0.5 for a unit sphere should give 16 vertices on the
  // equator.
  const auto mesh = MakeSphereVolumeMesh<double>(sphere, 0.5);
  VolumeMeshToVtk("sphere_tet.vtk", mesh, "Tetrahedral Mesh of Sphere");
}

GTEST_TEST(MeshToVtkTest, CylinderTetrahedra) {
  const Cylinder cylinder(1.0, 2.0);
  // resolution_hint 0.5 for a cylinder with unit radius should give 16
  // vertices on each bounding circle of the caps.
  const auto mesh = MakeCylinderVolumeMesh<double>(cylinder, 0.5);
  VolumeMeshToVtk("cylinder_tet.vtk", mesh, "Tetrahedral Mesh of Cylinder");
}

GTEST_TEST(MeshToVtkTest, BoxTetrahedra) {
  const Box box(4.0, 4.0, 2.0);
  // resolution_hint 0.5 is enough to have vertices on the medial axis.
  const auto mesh = MakeBoxVolumeMesh<double>(box, 0.5);
  VolumeMeshToVtk("box_tet.vtk", mesh, "Tetrahedral Mesh of Box");
}

GTEST_TEST(MeshToVtkTest, BoxTriangles) {
  const Box box(4.0, 4.0, 2.0);
  // Very coarse resolution_hint 4.0 should give the coarsest mesh.
  const auto mesh = MakeBoxSurfaceMesh<double>(box, 4.0);
  SurfaceMeshToVtk("box_tri.vtk", mesh, "Triangular Mesh of Box");
}

GTEST_TEST(MeshToVtkTest, BoxTetrahedraPressureField) {
  const Box box(4.0, 4.0, 2.0);
  // resolution_hint 0.5 is enough to have vertices on the medial axis.
  const auto mesh = MakeBoxVolumeMesh<double>(box, 0.5);
  const double kElasticModulus = 1.0e+5;
  const auto pressure =
      MakeBoxPressureField<double>(box, &mesh, kElasticModulus);
  VolumeMeshFieldLinearToVtk("box_tet_pressure.vtk", pressure,
                             "Pressure Field in Box");
}

// Helper function to create a contact surface between a soft box and a rigid
// box.
unique_ptr<ContactSurface<double>> BoxContactSurface() {
  const Box soft_box(4., 4., 2.);
  // resolution_hint 0.5 is enough to have vertices on the medial axis.
  const auto soft_mesh = MakeBoxVolumeMesh<double>(soft_box, 0.5);
  const double kElasticModulus = 1.0e+5;
  const auto soft_pressure =
      MakeBoxPressureField<double>(soft_box, &soft_mesh, kElasticModulus);
  // The soft box is at the center of World.
  RigidTransformd X_WS = RigidTransformd::Identity();

  const Box rigid_box(4, 4, 2);
  // Very coarse resolution_hint 4.0 should give the coarsest mesh.
  const auto rigid_mesh = MakeBoxSurfaceMesh<double>(rigid_box, 4.0);
  // The rigid box intersects the soft box in a unit cube at the corner
  // (2.0, 2.0, 1.0).
  RigidTransformd X_WR(Vector3d{3., 3., 1.});

  return mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
      GeometryId::get_new_id(), soft_pressure, X_WS,
      GeometryId::get_new_id(), rigid_mesh, X_WR);
}

GTEST_TEST(MeshToVtkTest, BoxContactSurfacePressure) {
  unique_ptr<ContactSurface<double>> contact = BoxContactSurface();
  auto contact_pressure =
      dynamic_cast<const SurfaceMeshFieldLinear<double, double>*>(
          &contact->e_MN());
  ASSERT_NE(contact_pressure, nullptr);
  SurfaceMeshFieldLinearToVtk("box_rigid_soft_contact_pressure.vtk",
                              *contact_pressure,
                              "Pressure Distribution on Contact Surface");
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
