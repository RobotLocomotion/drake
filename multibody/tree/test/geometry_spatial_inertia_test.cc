#include "drake/multibody/tree/geometry_spatial_inertia.h"

#include <algorithm>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
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

// Arbitrary, non-unit density.
constexpr double kDensity = 1.5;
constexpr double kTol = std::numeric_limits<double>::epsilon();

// Asserts that the given spatial inertia is equivalent to the given mass
// properties, i.e., mass, position vector, and unit inertia.
::testing::AssertionResult SpatialInertiasEqual(
    const SpatialInertia<double>& dut,
    const SpatialInertia<double>& M_expected,
    double tolerance = 0.0) {
  const double mass = M_expected.get_mass();
  const Vector3<double> p_BoBcm_B = M_expected.get_com();
  const UnitInertia<double> G_BBo_B = M_expected.get_unit_inertia();
  if (std::abs(mass - dut.get_mass()) > tolerance) {
    return ::testing::AssertionFailure()
           << "Expected equal masses\n"
           << "  expected:   " << mass << "\n"
           << "  tested:     " << dut.get_mass() << "\n"
           << "  difference: " << std::abs(mass - dut.get_mass()) << "\n"
           << "  is greater than tolerance: " << tolerance << "\n";
  }
  ::testing::AssertionResult result =
      CompareMatrices(dut.get_com(), p_BoBcm_B, tolerance);
  if (!result) return result;
  if (!dut.get_unit_inertia().IsNearlyEqualTo(G_BBo_B, tolerance)) {
    return ::testing::AssertionFailure()
           << "Expected equal unit inertias\n"
           << "  expected\n"
           << G_BBo_B << "\n"
           << "  tested\n"
           << dut.get_unit_inertia() << "\n"
           << "  with tolerance: " << tolerance << "\n"
           << "(with mass: " << dut.get_mass() << "\n"
           << " and com: "
           << fmt::to_string(fmt_eigen(dut.get_com().transpose())) << "\n";
  }
  return ::testing::AssertionSuccess();
}


// Note: Some tests below validate using SpatialInertia::SolidFooWithDensity()
// or UnitInertia::SolidFoo(). The *implementations* use similar logic, so,
// these tests would seem to be a tautology. This is intentional. The goal is
// that the spatial inertia (or unit inertia) generated via this API should
// provide the same value as passing through the SpatialInertia APIs.
// This supports that goal as a regression test.

GTEST_TEST(GeometrySpatialInertaTest, Box) {
  const double Lx = 1;
  const double Ly = 2;
  const double Lz = 3;
  const geometry::Box box(Lx, Ly, Lz);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidBoxWithDensity(kDensity, Lx, Ly, Lz);
  EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(box, kDensity),
                                   M_BBo_B, /* tolerance = */ 0.0));
}

GTEST_TEST(GeometrySpatialInertaTest, Capsule) {
  const double r = 1.5;
  const double L = 2;
  const geometry::Capsule capsule(r, L);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidCapsuleWithDensity(
          kDensity, r, L, Vector3<double>::UnitZ());
  EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(capsule, kDensity),
                                   M_BBo_B, /* tolerance = */ 0.0));
}

// Note: Convex can be found below in the MeshTypes test.

GTEST_TEST(GeometrySpatialInertaTest, Cylinder) {
  const double r = 1.5;
  const double L = 2;
  const geometry::Cylinder cylinder(r, L);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidCylinderWithDensity(
          kDensity, r, L, Vector3<double>::UnitZ());
  EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(cylinder, kDensity),
                                   M_BBo_B, /* tolerance = */ 0.0));
}

GTEST_TEST(GeometrySpatialInertaTest, Ellipsoid) {
  const double a = 1.5;
  const double b = 2.5;
  const double c = 3.5;
  const geometry::Ellipsoid ellipsoid(a, b, c);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidEllipsoidWithDensity(kDensity, a, b, c);
  EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(ellipsoid, kDensity),
                                   M_BBo_B, /* tolerance = */ 0.0));
}

GTEST_TEST(GeometrySpatialInertaTest, HalfSpace) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CalcSpatialInertia(geometry::HalfSpace(), kDensity), ".*HalfSpace.*");
}

// Note: Mesh can be found below in the MeshTypes test.

GTEST_TEST(GeometrySpatialInertaTest, MeshcatCone) {
  const geometry::MeshcatCone cone(1, 2, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(CalcSpatialInertia(cone, kDensity),
                              ".*MeshcatCone.*");
}

GTEST_TEST(GeometrySpatialInertaTest, Sphere) {
  const double r = 1.5;
  const geometry::Sphere sphere(r);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidSphereWithDensity(kDensity, r);
  EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(sphere, kDensity),
                                   M_BBo_B, /* tolerance = */ 0.0));
}

// Exercises the common code paths for Mesh and Convex (i.e., "MeshTypes").
template <typename MeshType>
class MeshTypeSpatialInertaTest : public testing::Test {};

TYPED_TEST_SUITE_P(MeshTypeSpatialInertaTest);

/* Calling CalcSpatialInertia(Shape) on a MeshType delegates the math to
 CalcSpatialInertia(TriangleSurfaceMesh). However, this function *does* have the
 following administrative responsibilities:

   1. Vet the extension and throw for non-obj files.
   2. Make sure the mesh type's `scale` property is used.
   3. Make sure the given density value is used.

 Those responsibilities are tested here. The math is tested below. */
TYPED_TEST_P(MeshTypeSpatialInertaTest, Administrivia) {
  using MeshType = TypeParam;

  const std::string valid_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");
  const MeshType unit_scale(valid_path, 1.0);
  const MeshType double_scale(valid_path, 2.0);
  const MeshType nonexistent("nonexistent.stl", 1.0);

  {
    // Extension test; .obj doesn't throw, everything else does.
    // Note: this should be case-insensitive; .OBJ should also work. However,
    // we need *another* valid OBJ with the different capitalization of the
    // extension to test this. Rather than creating/copying such a file, we're
    // foregoing the test. The case insensitivity is *not* documented.

    EXPECT_NO_THROW(CalcSpatialInertia(unit_scale, kDensity));
    DRAKE_EXPECT_THROWS_MESSAGE(
        CalcSpatialInertia(nonexistent, kDensity),
        ".*only supports .obj .* given '.*nonexistent.stl'.*");
  }

  {
    // Confirm that the scale is used.
    const SpatialInertia<double> M_SScm_S_small =
        CalcSpatialInertia(unit_scale, kDensity);
    const SpatialInertia<double> M_SScm_S_large =
        CalcSpatialInertia(double_scale, kDensity);
    EXPECT_DOUBLE_EQ(M_SScm_S_large.get_mass(), M_SScm_S_small.get_mass() * 8);
  }

  {
    // Confirm that the density is used.
    const SpatialInertia<double> M_SScm_S_small =
        CalcSpatialInertia(unit_scale, kDensity);
    const SpatialInertia<double> M_SScm_S_large =
        CalcSpatialInertia(unit_scale, kDensity * 2);
    EXPECT_DOUBLE_EQ(M_SScm_S_large.get_mass(), M_SScm_S_small.get_mass() * 2);
  }
}

REGISTER_TYPED_TEST_SUITE_P(MeshTypeSpatialInertaTest, Administrivia);
using MeshTypes = ::testing::Types<geometry::Convex, geometry::Mesh>;
INSTANTIATE_TYPED_TEST_SUITE_P(All, MeshTypeSpatialInertaTest, MeshTypes);

/* Tests the math for a triangle surface mesh by using a mesh that perfectly
 reproduces a primitive: a Box.

   - When the mesh is posed in its frame like the corresponding primitive is,
     we should get identical SpatialInertia values.
   - We can transform the mesh and observe predictable changes to the
     resulting spatial inertia.  */
GTEST_TEST(TriangleSurfaceMassPropertiesTest, ExactPolyhedron) {
  // Note: this is the same box B used in the Box test above.
  // Note: p_BoBcm = [0, 0, 0]. The box is defined such that its center of mass
  // is coincident with the box frame's origin.
  const double Lx = 2;
  const double Ly = 1;
  const double Lz = 3;
  const geometry::Box box(Lx, Ly, Lz);
  const SpatialInertia<double> M_BBo_B =
    SpatialInertia<double>::SolidBoxWithDensity(kDensity, Lx, Ly, Lz);

  {
    // Mesh posed in its frame the same as the solid box is in its own frame.
    // We use a coarse resolution hint simply to reduce the cost of the test.
    const double resolution_hint = std::max({Lx, Ly, Lz});
    const geometry::TriangleSurfaceMesh<double> mesh =
        geometry::internal::MakeBoxSurfaceMesh<double>(box, resolution_hint);

    EXPECT_TRUE(SpatialInertiasEqual(CalcSpatialInertia(mesh, kDensity),
                                     M_BBo_B, kTol));
  }

  {
    // We'll reposition the mesh's vertices so that the "bulk" of the geometry
    // is posed differently from the solid box (in each geometry's frame). By
    // transforming the set of vertices rigidly, we preserve the total volume
    // (and, therefore, mass) but its center of mass and unit inertia, in its
    // frame M, will be different from the box's in its own frame B. With
    // X_BM = I, the two bodies' quantities are related as follows:
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

    const double mass = M_BBo_B.get_mass();
    const UnitInertia<double> G_BBo_B = M_BBo_B.get_unit_inertia();
    const UnitInertia<double> G_BBo_M = G_BBo_B.ReExpress(R_MB);
    const UnitInertia<double> G_MMo_M =
        G_BBo_M.ShiftFromCenterOfMass(p_BcmMcm);

    // The vertex transformation introduces precision loss, requiring a larger
    // tolerance.
    EXPECT_TRUE(SpatialInertiasEqual(
        CalcSpatialInertia(mesh, kDensity),
        SpatialInertia<double>(mass, p_BcmMcm, G_MMo_M), 8 * kTol));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
