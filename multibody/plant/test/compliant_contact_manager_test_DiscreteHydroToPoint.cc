#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace internal {

constexpr double kEps = std::numeric_limits<double>::epsilon();

GTEST_TEST(CompliantContactManagerTest, DiscreteHydroelasticToPointContact) {
  auto id_M = geometry::GeometryId::get_new_id();
  auto id_N = geometry::GeometryId::get_new_id();
  // The order of id_M and id_N determines the orientation of the contact patch.
  // For id_M < id_N, the face normal points *out of* geometry N and *into* M.
  ASSERT_TRUE(id_M < id_N);
  // This is a square contact patch of one polygon that is 1cm long.
  // It lies on XY-plane (Z=0). We imagine the geometry M is above XY-plane,
  // and the geometry N is below XY-plane. The polygon vertices are ordered
  // in such a way that the face normal is in +Z direction.
  auto surface_mesh = std::make_unique<geometry::PolygonSurfaceMesh<double>>(
      std::vector<int>{4, 0, 1, 2, 3},
      std::vector<Vector3d>{
          {0, 0, 0}, {1e-2, 0, 0}, {1e-2, 1e-2, 0}, {0., 1e-2, 0.}});
  ASSERT_TRUE(
      CompareMatrices(surface_mesh->face_normal(0), Vector3d::UnitZ(), kEps));
  // The contact patch has constant pressure value of 10 MPa.
  // This contrived example lacks the part of the contact patch passing the
  // boundary of the geometries, i.e., there is no zero pressure on the patch.
  auto pressure_field_on_contact_patch =
      std::make_unique<geometry::PolygonSurfaceMeshFieldLinear<double, double>>(
          std::vector<double>{1e7, 1e7, 1e7, 1e7}, surface_mesh.get(),
          // Constant pressure means zero gradient *along* the contact patch.
          std::vector<Vector3d>{{0, 0, 0}});
  // Pressure gradient *into* the volume of geometry M sampling on the contact
  // patch. Pressure increases by 1 GPa per meter along the +Z direction.
  // It is comparable to high-density polyethylene (HDPE) whose Young's
  // modulus is about 0.97 to 1.38 GPa.
  // See https://en.wikipedia.org/wiki/Young%27s_modulus#Approximate_values
  auto pressure_gradients_into_volume =
      std::make_unique<std::vector<Vector3d>>(
          std::vector<Vector3d>{{0, 0, 1e9}});
  geometry::ContactSurface<double> patch(
      id_M, id_N,
      std::move(surface_mesh), std::move(pressure_field_on_contact_patch),
      std::move(pressure_gradients_into_volume), nullptr);
  // Double check that the two geometry Ids are not switched, and the
  // surface normal is not reversed.
  ASSERT_EQ(patch.id_M(), id_M);
  ASSERT_EQ(patch.id_N(), id_N);
  ASSERT_TRUE(CompareMatrices(patch.face_normal(0),
                              Vector3d::UnitZ(), kEps));
  ASSERT_NEAR(patch.area(0), 1e-4, kEps);

  CompliantContactManager<double> manager;
  double normal_force_on_polygon;
  double effective_stiffness;
  double surrogate_signed_distance;
  bool success =
      CompliantContactManagerTester::DiscreteHydroelasticToPointContact(
          manager, patch, 0, &normal_force_on_polygon, &effective_stiffness,
          &surrogate_signed_distance);

  ASSERT_TRUE(success);
  EXPECT_NEAR(normal_force_on_polygon, 1e3, kEps);
  EXPECT_NEAR(effective_stiffness, 1e5, kEps);
  EXPECT_NEAR(surrogate_signed_distance, -1e-2, kEps);
}


GTEST_TEST(CompliantContactManagerTest, DiscreteHydroToPointContactTricky) {
  auto id_M = geometry::GeometryId::get_new_id();
  auto id_N = geometry::GeometryId::get_new_id();
  // The order of id_M and id_N determines the orientation of the contact patch.
  // For id_M < id_N, the face normal points *out of* geometry N and *into* M.
  ASSERT_TRUE(id_M < id_N);
  // This is a square contact patch of one polygon that is 1cm long.
  // It lies on XY-plane (Z=0). We imagine the geometry M is above XY-plane,
  // and the geometry N is below XY-plane. The polygon vertices are ordered
  // in such a way that the face normal is in +Z direction.
  auto surface_mesh = std::make_unique<geometry::PolygonSurfaceMesh<double>>(
      std::vector<int>{4, 0, 1, 2, 3},
          std::vector<Vector3d>{
              {0, 0, 0}, {1e-2, 0, 0}, {1e-2, 1e-2, 0}, {0., 1e-2, 0.}});
  ASSERT_TRUE(
      CompareMatrices(surface_mesh->face_normal(0), Vector3d::UnitZ(), kEps));
  // Imagine the contact patch starts from the boundary of the geometry M
  // from vertex 0 to vertex 1, so their pressure values are zero.
  // The other two vertices are inside the volume at the same distance from
  // the boundary, so they have the same pressure value of 10 MPa (about
  // 1,450 psi).
  // This incomplete contact patch doesn't make a closed boundary curve on the
  // boundary surface of the geometry M.
  auto pressure_field_on_contact_patch =
      std::make_unique<geometry::PolygonSurfaceMeshFieldLinear<double, double>>(
          std::vector<double>{0, 0, 1e7, 1e7}, surface_mesh.get(),
          // The pressure gradient *along* the contact patch increases by 1e7
          // Pascals along 1e-2 meters = 1e9 Pa/m in +Y direction.
          std::vector<Vector3d>{{0, 1e9, 0}});
  // ======================
  // Very important details
  // ======================
  // We set up a very tricky pressure gradient *into* the volume of geometry M
  // by making it to be *almost tangent* to the contact patch.  Specifically we
  // make it to be almost the same as the pressure gradient *along* the contact
  // patch with a very small component *into* the volume of geometry M.
  // Pressure increases around 1 GPa per meter in the gradient direction.
  // It is comparable to high-density polyethylene (HDPE) whose Young's
  // modulus is about 0.97 to 1.38 GPa.
  // See https://en.wikipedia.org/wiki/Young%27s_modulus#Approximate_values
  auto pressure_gradients_into_volume =
      std::make_unique<std::vector<Vector3d>>(
          std::vector<Vector3d>{{0, 1e9, 5e-6}});
  geometry::ContactSurface<double> patch(
      id_M, id_N,
      std::move(surface_mesh), std::move(pressure_field_on_contact_patch),
      std::move(pressure_gradients_into_volume), nullptr);
  // Double check that the two geometry Ids are not switched, and the
  // surface normal is not reversed.
  ASSERT_EQ(patch.id_M(), id_M);
  ASSERT_EQ(patch.id_N(), id_N);
  ASSERT_TRUE(CompareMatrices(patch.face_normal(0),
                              Vector3d::UnitZ(), kEps));
  ASSERT_NEAR(patch.area(0), 1e-4, kEps);

  CompliantContactManager<double> manager;
  double normal_force_on_polygon;
  double effective_stiffness;
  double surrogate_signed_distance;
  bool success =
      CompliantContactManagerTester::DiscreteHydroelasticToPointContact(
          manager, patch, 0, &normal_force_on_polygon, &effective_stiffness,
          &surrogate_signed_distance);

  ASSERT_TRUE(success);
  // area * centroidal pressure = 1e-4 * 5e6 = 5e2 Newtons
  EXPECT_NEAR(normal_force_on_polygon, 5e2, 1e-12);
  // area * normal pressure gradient
  // = area * pressure_gradients_into_volume.Z()
  // = 1e-4 * 5e-6 = 5e-10 Pascal*meter
  EXPECT_NEAR(effective_stiffness, 5e-10, kEps);
  // - centroidal pressure / normal pressure gradient
  // = - centroidal pressure / pressure_gradients_into_volume.Z()
  // = -5e6 / 5e-6 = -1e12 meters = 1 terameters
  EXPECT_NEAR(surrogate_signed_distance, -1e12, 1);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
