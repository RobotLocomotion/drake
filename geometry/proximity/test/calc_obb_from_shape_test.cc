#include "drake/geometry/proximity/calc_obb_from_shape.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(CalcObbFromShapeTest, CalcObb) {
  constexpr double kTol = 1.0e-13;
  // Box
  {
    const Box box(1, 2, 3);
    const std::optional<Obb> obb = CalcObb(box);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(CompareMatrices(obb->half_width(),
                                Vector3<double>(0.5, 1.0, 1.5), kTol));
  }

  // Capsule
  {
    const double radius = 1.23;
    const double length = 2.4;
    const Capsule capsule(radius, length);
    const std::optional<Obb> obb = CalcObb(capsule);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    const double half_length = length / 2.0 + radius;
    EXPECT_TRUE(CompareMatrices(
        obb->half_width(), Vector3<double>(radius, radius, half_length), kTol));
  }

  // Cylinder
  {
    const double radius = 1.3;
    const double length = 2.1;
    const Cylinder cylinder(radius, length);
    const std::optional<Obb> obb = CalcObb(cylinder);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    const double half_length = length / 2.0;
    EXPECT_TRUE(CompareMatrices(
        obb->half_width(), Vector3<double>(radius, radius, half_length), kTol));
  }

  // Ellipsoid
  {
    const Ellipsoid ellipsoid(1, 2, 3);
    const std::optional<Obb> obb = CalcObb(ellipsoid);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 2, 3), kTol));
  }

  // Sphere
  {
    const double radius = 3.0;
    const Sphere sphere(radius);
    const std::optional<Obb> obb = CalcObb(sphere);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(CompareMatrices(obb->half_width(),
                                Vector3<double>(radius, radius, radius), kTol));
  }

  // HalfSpace
  {
    const HalfSpace hs;
    const std::optional<Obb> obb = CalcObb(hs);
    EXPECT_FALSE(obb.has_value());
  }

  // MeshcatCone
  {
    const MeshcatCone cone(4, 2, 3);
    const std::optional<Obb> obb = CalcObb(cone);
    EXPECT_FALSE(obb.has_value());
  }

  // For Mesh and Convex, we'll use a simple cube and confirm that the OBB is
  // as expected.
  {
    const std::string cube_obj =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    const Mesh mesh(cube_obj);
    const std::optional<Obb> obb = CalcObb(mesh);
    ASSERT_TRUE(obb.has_value());
    // The quad_cube is an axis-aligned 2x2x2 cube centered at the origin.
    // So, its OBB should be an axis-aligned box with half_width of (1, 1, 1).
    // PCA may lead to arbitrary axis directions. We check that the translation
    // is zero and that the rotation is a signed permutation matrix.
    const auto& pose = obb->pose();
    EXPECT_TRUE(
        CompareMatrices(pose.translation(), Vector3<double>::Zero(), kTol));
    const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
      EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
    }
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 1, 1), kTol));
  }
  {
    const std::string cube_obj =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    const Convex convex(cube_obj);
    const std::optional<Obb> obb = CalcObb(convex);
    ASSERT_TRUE(obb.has_value());
    const auto& pose = obb->pose();
    EXPECT_TRUE(
        CompareMatrices(pose.translation(), Vector3<double>::Zero(), kTol));
    const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
      EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
    }
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 1, 1), kTol));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
