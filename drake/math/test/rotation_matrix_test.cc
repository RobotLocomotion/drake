#include "drake/math/rotation_matrix.h"

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace drake {
namespace math {
namespace {

// Test XRotation, YRotation, ZRotation
GTEST_TEST(RotationMatrixTest, TestXYZ) {
  Vector3d i(1, 0, 0), j(0, 1, 0), k(0, 0, 1);

  double tol = 1e-12;
  EXPECT_TRUE(CompareMatrices(XRotation(M_PI_4) * i, i, tol));
  EXPECT_TRUE(CompareMatrices(XRotation(M_PI_4) * j,
                              Vector3d(0, M_SQRT1_2, M_SQRT1_2), tol));
  EXPECT_TRUE(CompareMatrices(XRotation(M_PI_4) * k,
                              Vector3d(0, -M_SQRT1_2, M_SQRT1_2), tol));

  EXPECT_TRUE(CompareMatrices(YRotation(M_PI_4) * i,
                              Vector3d(M_SQRT1_2, 0, -M_SQRT1_2), tol));
  EXPECT_TRUE(CompareMatrices(YRotation(M_PI_4) * j, j, tol));
  EXPECT_TRUE(CompareMatrices(YRotation(M_PI_4) * k,
                              Vector3d(M_SQRT1_2, 0, M_SQRT1_2), tol));

  EXPECT_TRUE(CompareMatrices(ZRotation(M_PI_4) * i,
                              Vector3d(M_SQRT1_2, M_SQRT1_2, 0), tol));
  EXPECT_TRUE(CompareMatrices(ZRotation(M_PI_4) * j,
                              Vector3d(-M_SQRT1_2, M_SQRT1_2, 0), tol));
  EXPECT_TRUE(CompareMatrices(ZRotation(M_PI_4) * k, k, tol));

  // Test that rotations by PI do not change the rotation axis, and flip the
  // signs on the other two axes.
  EXPECT_TRUE(CompareMatrices(
      XRotation(M_PI + 0.3),
      Eigen::DiagonalMatrix<double, 3>(1, -1, -1) * XRotation(0.3), tol));
  EXPECT_TRUE(CompareMatrices(
      YRotation(M_PI + 0.3),
      Eigen::DiagonalMatrix<double, 3>(-1, 1, -1) * YRotation(0.3), tol));
  EXPECT_TRUE(CompareMatrices(
      ZRotation(M_PI + 0.3),
      Eigen::DiagonalMatrix<double, 3>(-1, -1, 1) * ZRotation(0.3), tol));
}

GTEST_TEST(RotationMatrixTest, TestProjection) {
  double tol = 1e-12;

  // Identity => Identity
  EXPECT_TRUE(CompareMatrices(ProjectMatToRotMat(Matrix3d::Identity()),
                              Matrix3d::Identity(), tol));

  // R1 (a valid rotation matrix) => R1
  Matrix3d R1 = rpy2rotmat(Vector3d(0.1, 0.2, 0.3));
  EXPECT_TRUE(CompareMatrices(ProjectMatToRotMat(R1), R1, tol));

  // 2*R1 => R1
  EXPECT_TRUE(CompareMatrices(ProjectMatToRotMat(2 * R1), R1, tol));

  // Non-rotation matrix in gets an orthonormal matrix out.
  Matrix3d M2;
  M2 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Matrix3d R2 = ProjectMatToRotMat(M2);
  EXPECT_TRUE(CompareMatrices(R2.transpose(), R2.inverse(), tol));

  // Determinant could be -1 or 1.
  EXPECT_NEAR(std::abs(R2.determinant()), 1.0, tol);
}

}  // namespace
}  // namespace math
}  // namespace drake
