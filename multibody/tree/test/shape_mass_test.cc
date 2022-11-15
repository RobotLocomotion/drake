#include "drake/multibody/tree/shape_mass.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

constexpr double kDensity = 1.5;
constexpr double kTol = 1e-15;

// Asserts that the given spatial inertia is equivalent to the given mass
// properties.
::testing::AssertionResult MassEqual(const SpatialInertia<double>& dut,
                                     const double mass, const Vector3d& p_SScm,
                                     const UnitInertia<double>& G_SScm_S,
                                     double tolerance = 0.0) {
  if (std::abs(mass - dut.get_mass()) > tolerance) {
    return ::testing::AssertionFailure()
           << "Expected equal masses\n"
           << "  expected:   " << mass << "\n"
           << "  tested:     " << dut.get_mass() << "\n"
           << "  difference: " << std::abs(mass - dut.get_mass()) << "\n"
           << "  is greater than tolerance: " << tolerance << "\n";
  }
  ::testing::AssertionResult result =
      CompareMatrices(dut.get_com(), p_SScm, tolerance);
  if (!result) return result;
  if (!dut.get_unit_inertia().IsNearlyEqualTo(G_SScm_S, tolerance)) {
    return ::testing::AssertionFailure()
           << "Expected equal unit inertias\n"
           << "  expected\n"
           << G_SScm_S << "\n"
           << "  tested\n"
           << dut.get_unit_inertia() << "\n"
           << "  with tolerance: " << tolerance << "\n"
           << "(with mass: " << dut.get_mass() << "\n"
           << " and com: " << dut.get_com().transpose() << "\n";
  }
  return ::testing::AssertionSuccess();
}

// Note: These tests mostly rely on UnitInertia::SolidFoo() methods to validate.
// The *implementations* use the same logic. So, these tests would seem to be
// a tautology. As such, these tests serve as regression tests -- the shapes
// are supported as we expect and provide equivalent values to what one
// would expect from the UnitInertia APIs.

GTEST_TEST(ShapeMassPropertiesTest, Box) {
  const double Lx = 1;
  const double Ly = 2;
  const double Lz = 3;
  const double volume = 6;  // Lx * Ly * Lz
  const Vector3d p_SScm = Vector3d::Zero();
  const auto G_SScm_S = UnitInertia<double>::SolidBox(Lx, Ly, Lz);

  const geometry::Box box(Lx, Ly, Lz);
  // Test with specified density and default unit density.
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(box, kDensity),
                        volume * kDensity, p_SScm, G_SScm_S, kTol));
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(box),
                        volume, p_SScm, G_SScm_S, kTol));
}

GTEST_TEST(ShapeMassPropertiesTest, Capsule) {
  const double r = 1.5;
  const double L = 2;
  const double volume = 28.274333882308138;  // πr²L + 4/3πr³
  const Vector3d p_SScm = Vector3d::Zero();
  const auto G_SScm_S = UnitInertia<double>::SolidCapsule(r, L);

  const geometry::Capsule capsule(r, L);
  // Test with specified density and default unit density.
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(capsule, kDensity),
                        volume * kDensity, p_SScm, G_SScm_S, kTol));
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(capsule),
                        volume, p_SScm, G_SScm_S, 4 * kTol));
}

// Note: Convex can be found below in the MeshTypes test.

GTEST_TEST(ShapeMassPropertiesTest, Cylinder) {
  const double r = 1.5;
  const double L = 2;
  const double volume = 14.137166941154069;  // πr²L
  const Vector3d p_SScm = Vector3d::Zero();
  const auto G_SScm_S = UnitInertia<double>::SolidCylinder(r, L);

  const geometry::Cylinder cylinder(r, L);
  // Test with specified density and default unit density.
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(cylinder, kDensity),
                        volume * kDensity, p_SScm, G_SScm_S, kTol));
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(cylinder),
                        volume, p_SScm, G_SScm_S, kTol));
}

GTEST_TEST(ShapeMassPropertiesTest, Ellipsoid) {
  const double a = 1.5;
  const double b = 2.5;
  const double c = 3.5;
  const double volume = 54.97787143782138;  // 4/3πabc
  const Vector3d p_SScm = Vector3d::Zero();
  const auto G_SScm_S = UnitInertia<double>::SolidEllipsoid(a, b, c);

  const geometry::Ellipsoid ellipsoid(a, b, c);
  // Test with specified density and default unit density.
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(ellipsoid, kDensity),
                        volume * kDensity, p_SScm, G_SScm_S, kTol));
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(ellipsoid),
                        volume, p_SScm, G_SScm_S, kTol));
}

GTEST_TEST(ShapeMassPropertiesTest, HalfSpace) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CalcSpatialInertia(geometry::HalfSpace(), kDensity), ".*HalfSpace.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CalcSpatialInertia(geometry::HalfSpace()), ".*HalfSpace.*");
}

// Note: Mesh can be found below in the MeshTypes test.

GTEST_TEST(ShapeMassPropertiesTest, MeshcatCone) {
  const geometry::MeshcatCone cone(1, 2, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(CalcSpatialInertia(cone, kDensity),
                              ".*MeshcatCone.*");
  DRAKE_EXPECT_THROWS_MESSAGE(CalcSpatialInertia(cone), ".*MeshcatCone.*");
}

GTEST_TEST(ShapeMassPropertiesTest, Sphere) {
  const double r = 1.5;
  const double volume = 14.137166941154067;  // 4/3πr³
  const Vector3d p_SScm = Vector3d::Zero();
  const auto G_SScm_S = UnitInertia<double>::SolidSphere(r);

  const geometry::Sphere sphere(r);
  // Test with specified density and default unit density.
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(sphere, kDensity),
                        volume * kDensity, p_SScm, G_SScm_S, kTol));
  EXPECT_TRUE(MassEqual(CalcSpatialInertia(sphere),
                        volume, p_SScm, G_SScm_S, kTol));
}

// Exercises the common code paths for Mesh and Convex (i.e., "MeshTypes").
template <typename MeshType>
class MeshTypeMasPropertiesTest : public testing::Test {};

TYPED_TEST_SUITE_P(MeshTypeMasPropertiesTest);

/* Calling CalcSpatialInertia(Shape) on a MeshType delegates the math to
 CalcSpatialInertia(TriangleSurfaceMesh). However, this function *does* have the
 following administrative responsibilities:

   1. Vet the extension and throw for non-obj files.
   2. Make sure the mesh type's `scale` property is used.
   3. Make sure the given density value is used.

 Those responsibilities are tested here. The math is tested below. */
TYPED_TEST_P(MeshTypeMasPropertiesTest, Administrivia) {
  using MeshType = TypeParam;

  const std::string valid_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");
  const MeshType unit_scale(valid_path, 1.0);
  const MeshType double_scale(valid_path, 2.0);
  const MeshType nonexistant("nonexistant.stl", 1.0);

  {
    // Extension test; .obj doesn't throw, everything else does.
    // Note: this should be case-insensitve; .OBJ should also work. However,
    // we need *another* valid OBJ with the different capitalization of the
    // extension to test this. Rather than creating/copying such a file, we're
    // foregoing the test. The case insensitivity is *not* documented.

    EXPECT_NO_THROW(CalcSpatialInertia(unit_scale));
    DRAKE_EXPECT_THROWS_MESSAGE(
        CalcSpatialInertia(nonexistant),
        ".*only supports .obj .* given 'nonexistant.stl'.*");
  }

  {
    // Confirm that the scale is used.
    const SpatialInertia<double> M_SScm_S_small =
        CalcSpatialInertia(unit_scale);
    const SpatialInertia<double> M_SScm_S_large =
        CalcSpatialInertia(double_scale);
    EXPECT_DOUBLE_EQ(M_SScm_S_large.get_mass(), M_SScm_S_small.get_mass() * 8);
  }

  {
    // Confirm that the density is used.
    const SpatialInertia<double> M_SScm_S_small =
        CalcSpatialInertia(unit_scale, 1.5);
    const SpatialInertia<double> M_SScm_S_large =
        CalcSpatialInertia(unit_scale, 3.0);
    EXPECT_DOUBLE_EQ(M_SScm_S_large.get_mass(), M_SScm_S_small.get_mass() * 2);
  }
}

REGISTER_TYPED_TEST_SUITE_P(MeshTypeMasPropertiesTest, Administrivia);
using MeshTypes = ::testing::Types<geometry::Convex, geometry::Mesh>;
INSTANTIATE_TYPED_TEST_SUITE_P(All, MeshTypeMasPropertiesTest, MeshTypes);

/* Tests the math for a triangle surface mesh by using a mesh that perfectly
 reproduces a primitive: a Box.

   - When the mesh is posed in its frame like the corresponding primitive is,
     we should get identical SpatialInertia values.
   - We can transform the mesh and observe predictable changes to the
     resulting spatial inertia.  */
GTEST_TEST(TriangleSurfaceMassPropertiesTest, ExactPolyhedron) {
  // Note: this is the same box used in the Box test above.
  const double Lx = 2;
  const double Ly = 1;
  const double Lz = 3;
  const double volume = 6;  // Lx * Ly * Lz
  // Note: p_BoBcm = [0, 0, 0]. The box is defined such that its center of mass
  // is coincident with the box frame's origin.
  const Vector3d p_BBcm = Vector3d::Zero();
  const auto G_BBo_B = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const geometry::Box box(Lx, Ly, Lz);

  {
    // Mesh posed in its frame the same as the solid box is in its own frame.
    const geometry::TriangleSurfaceMesh<double> mesh =
        geometry::internal::MakeBoxSurfaceMesh<double>(box, 5);

    // Test with specified density and default unit density.
    EXPECT_TRUE(MassEqual(CalcSpatialInertia(mesh, kDensity), volume * kDensity,
                          p_BBcm, G_BBo_B, kTol));
    EXPECT_TRUE(
        MassEqual(CalcSpatialInertia(mesh), volume, p_BBcm, G_BBo_B, kTol));
  }

  {
    // We'll reposition the mesh's vertices so that the *mass* of the geometry
    // is positioned differently from the solid box (in each geometry's frame).
    // By translating and rotating the set of vertices *en masse*, we preserve
    // the total mass. The resulting SpatialInertia of the mesh M, is related to
    // the solid box's as follows:
    //
    //     p_MMcm = p_BBo + p_BcmMcm  // R_BM = I and Bo = Bcm.
    //            = p_BcmMcm          // Bo = Bcm --> p_BBo = [0, 0, 0].
    //     G_MMo_M = G_BBo_B re-expressed in M and then shifted to Mcm.

    const Vector3d p_BcmMcm(0.25, 2.3, -5.4);
    const RotationMatrixd R_MB(
        AngleAxisd(M_PI / 3, Vector3d(1, 2, 3).normalized()));
    const RigidTransformd X_MB(R_MB, p_BcmMcm);

    geometry::TriangleSurfaceMesh<double> mesh =
        geometry::internal::MakeBoxSurfaceMesh<double>(box, 5);
    mesh.TransformVertices(X_MB);

    const UnitInertia<double> G_BBo_M = G_BBo_B.ReExpress(R_MB);
    const UnitInertia<double> G_MMo_M =
        G_BBo_M.ShiftFromCenterOfMass(p_BcmMcm);

    // Test with specified density and default unit density.
    EXPECT_TRUE(MassEqual(CalcSpatialInertia(mesh, kDensity), volume * kDensity,
                          p_BcmMcm, G_MMo_M, kTol));
    EXPECT_TRUE(
        MassEqual(CalcSpatialInertia(mesh), volume, p_BcmMcm, G_MMo_M, kTol));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
