#include "drake/geometry/proximity/calc_obb.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
constexpr double kTol = 1.0e-13;

GTEST_TEST(CalcObbFromShapeTest, Box) {
  const Box box(1, 2, 3);
  const std::optional<Obb> obb = CalcObb(box);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(obb->pose().IsExactlyIdentity());
  EXPECT_TRUE(
      CompareMatrices(obb->half_width(), Vector3d(0.5, 1.0, 1.5), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Capsule) {
  const double radius = 1.0;
  const double length = 2.4;
  const Capsule capsule(radius, length);
  const std::optional<Obb> obb = CalcObb(capsule);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(obb->pose().IsExactlyIdentity());
  EXPECT_TRUE(
      CompareMatrices(obb->half_width(), Vector3d(1.0, 1.0, 2.2), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Cylinder) {
  const double radius = 1.3;
  const double length = 2.1;
  const Cylinder cylinder(radius, length);
  const std::optional<Obb> obb = CalcObb(cylinder);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(obb->pose().IsExactlyIdentity());
  EXPECT_TRUE(
      CompareMatrices(obb->half_width(), Vector3d(1.3, 1.3, 1.05), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Ellipsoid) {
  const Ellipsoid ellipsoid(1, 2, 3);
  const std::optional<Obb> obb = CalcObb(ellipsoid);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(obb->pose().IsExactlyIdentity());
  EXPECT_TRUE(CompareMatrices(obb->half_width(), Vector3d(1, 2, 3), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Sphere) {
  const double radius = 3.0;
  const Sphere sphere(radius);
  const std::optional<Obb> obb = CalcObb(sphere);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(obb->pose().IsExactlyIdentity());
  EXPECT_TRUE(CompareMatrices(obb->half_width(),
                              Vector3d(radius, radius, radius), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, HalfSpace) {
  const HalfSpace hs;
  const std::optional<Obb> obb = CalcObb(hs);
  EXPECT_FALSE(obb.has_value());
}

GTEST_TEST(CalcObbFromShapeTest, MeshcatCone) {
  const MeshcatCone cone(4, 2, 3);
  const std::optional<Obb> obb = CalcObb(cone);
  ASSERT_TRUE(obb.has_value());
  EXPECT_TRUE(
      CompareMatrices(obb->pose().translation(), Vector3d(0, 0, 2.0), kTol));
  EXPECT_TRUE(CompareMatrices(obb->pose().rotation().matrix(),
                              Matrix3<double>::Identity(), kTol));
  EXPECT_TRUE(CompareMatrices(obb->half_width(), Vector3d(2, 3, 2), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Mesh) {
  const std::string cube_obj =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const Mesh mesh(cube_obj);
  const std::optional<Obb> obb = CalcObb(mesh);
  ASSERT_TRUE(obb.has_value());
  // The quad_cube is an axis-aligned 2x2x2 cube centered at the origin.
  // So, its OBB should be an axis-aligned box with half_width of (1, 1, 1).
  // PCA may lead to arbitrary axis directions. We check that the translation
  // is zero and that the rotation is a signed permutation matrix.
  const RigidTransformd& pose = obb->pose();
  EXPECT_TRUE(CompareMatrices(pose.translation(), Vector3d::Zero(), kTol));
  const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
    EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
  }
  EXPECT_TRUE(CompareMatrices(obb->half_width(), Vector3d(1, 1, 1), kTol));
}

GTEST_TEST(CalcObbFromShapeTest, Convex) {
  const std::string cube_obj =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const Convex convex(cube_obj);
  const std::optional<Obb> obb = CalcObb(convex);
  ASSERT_TRUE(obb.has_value());
  const RigidTransformd& pose = obb->pose();
  EXPECT_TRUE(CompareMatrices(pose.translation(), Vector3d::Zero(), kTol));
  const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
    EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
  }
  EXPECT_TRUE(CompareMatrices(obb->half_width(), Vector3d(1, 1, 1), kTol));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
