#include "drake/geometry/spatial_pose.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace {

using Iso3d = Isometry3<double>;
using Quatd = Quaternion<double>;
using SpatialPosed = SpatialPose<double>;
using Vector3d = Vector3<double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

GTEST_TEST(SpatialPose, Constructor) {
  // Case: with assert armed, all values are NaN.
#ifndef DRAKE_ASSERT_IS_DISARMED
  using std::isnan;
  SpatialPosed pose1;
  const auto& q1 = pose1.rotational().coeffs();
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(isnan(q1[i]));
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(isnan(pose1.translational()[i]));
  }
#endif

  const double w = 0.5, q_x = 0.1, q_y = 0.2, q_z = 0.3;
  const double x = 10, y = 20, z = 30;

  // Case: initialization with orientation and position.
  Quatd q(w, q_x, q_y, q_z);
  Vector3d p(x, y, z);
  SpatialPosed pose2(q, p);

  // Confirms that the data isn't *shared*, but the values are the same.
  EXPECT_NE(&pose2.rotational(), &q);
  EXPECT_TRUE(CompareMatrices(pose2.rotational().coeffs(), q.coeffs()));
  EXPECT_NE(&pose2.translational(), &p);
  EXPECT_TRUE(CompareMatrices(pose2.translational(), p));

  // Case: initialization with isometry.
  Iso3d iso;
  iso.translation() << x, y, z;
  // A rotation of the axes: z -> y, y -> x, x -> z, expressed in column-major
  // order.
  iso.linear() << 0, 1, 0, 0, 0, 1, 1, 0, 0;
  SpatialPosed pose3(iso);
  // There are two quaternions which map to the axis rotation. Simply test
  // against both of them (where one is the negation of the other).
  Quatd q3(0.5, -0.5, -0.5, -0.5);
  EXPECT_TRUE(CompareMatrices(pose3.rotational().coeffs(), q3.coeffs()) ||
              CompareMatrices(pose3.rotational().coeffs(), -q3.coeffs()));
  EXPECT_TRUE(CompareMatrices(pose3.translational(), p));
}

GTEST_TEST(SpatialPose, IsometryFromPose) {
  // A rotation of the axes; the basis goes from x-y-z to y-z-x.
  Quatd q(0.5, -0.5, -0.5, -0.5);
  Vector3d p(10, 20, 30);
  SpatialPosed pose(q, p);

  Iso3d expected;
  expected.linear() << 0, 1, 0, 0, 0, 1, 1, 0, 0;
  expected.translation() = p;

  auto derived_iso = pose.get_isometry();
  // Note: Because the Isometry is "AffineCompact", the bottom row is
  // effectively garbage; ignore the bad row in comparison.
  EXPECT_TRUE(CompareMatrices(derived_iso.matrix().template block<3, 4>(0, 0),
                              expected.matrix().template block<3, 4>(0, 0)));
}

GTEST_TEST(SpatialPose, Setters) {
  // A rotation of the axes; the basis goes from x-y-z to y-z-x.
  Quatd q(0.5, -0.5, -0.5, -0.5);
  Vector3d p(10, 20, 30);
  SpatialPosed pose(q, p);
  SpatialPosed cached_pose = pose;

  Vector3d new_p(-10, -20, -30);
  Quatd new_q(0.5, 0.5, 0.5, 0.5); // rotation in the opposite direction.
  Iso3d new_expected;
  new_expected.linear() << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  new_expected.translation() = new_p;

  // Confirm initial conditions.
  EXPECT_FALSE(CompareMatrices(pose.rotational().coeffs(), new_q.coeffs()));
  EXPECT_FALSE(CompareMatrices(pose.translational(), new_p));
  EXPECT_FALSE(CompareMatrices(
      pose.get_isometry().matrix().template block<3, 4>(0, 0),
      new_expected.matrix().template block<3, 4>(0, 0)));

  // Set rotational and translational components.
  pose.set_translational(new_p);
  EXPECT_TRUE(CompareMatrices(pose.translational(), new_p));
  pose.set_rotational(new_q);
  EXPECT_TRUE(CompareMatrices(pose.rotational().coeffs(), new_q.coeffs()));
  EXPECT_TRUE(CompareMatrices(
      pose.get_isometry().matrix().template block<3, 4>(0, 0),
      new_expected.matrix().template block<3, 4>(0, 0)));

  // Set isometry.
  pose = cached_pose;
  // Confirm initial conditions.
  EXPECT_FALSE(CompareMatrices(pose.rotational().coeffs(), new_q.coeffs()));
  EXPECT_FALSE(CompareMatrices(pose.translational(), new_p));
  EXPECT_FALSE(CompareMatrices(
      pose.get_isometry().matrix().template block<3, 4>(0, 0),
      new_expected.matrix().template block<3, 4>(0, 0)));

  pose.set_pose(new_expected);
  EXPECT_TRUE(CompareMatrices(pose.translational(), new_p));
  EXPECT_TRUE(CompareMatrices(pose.rotational().coeffs(), new_q.coeffs()));
  EXPECT_TRUE(CompareMatrices(
      pose.get_isometry().matrix().template block<3, 4>(0, 0),
      new_expected.matrix().template block<3, 4>(0, 0)));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
