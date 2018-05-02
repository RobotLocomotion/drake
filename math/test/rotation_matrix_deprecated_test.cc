// Note: Although this file tests functionality in rotation_matrix_deprecated.h,
// that file cannot be directly included (see rotation_matrix_deprecated.h for
// details). Instead, rotation_matrix.h needs to be included and that #include
// statement must be placed below to avoid erroneous drakeLint.py errors.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"

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

// Take many possible samples of the rotation angle θ, make sure the rotation
// matrix as R[θ] = AngleAxis(θ, axis) has larger error than the projected
// matrix R, namely
// (R(i,j) - M(i,j))² <= (R[θ](i,j) - M(i,j))² ∀ θ: angle_lb <= θ <= angle_ub
void CheckProjectionWithAxis(const Eigen::Matrix3d& M,
                             const Eigen::Vector3d& axis, double angle_lb,
                             double angle_ub) {
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(ProjectMatToRotMatWithAxis(M, axis, angle_lb, angle_ub),
                        axis)
          .toRotationMatrix();
  const double R_error = (R - M).squaredNorm();
  const int kNumAngles = 100;
  double theta_lb{};
  double theta_ub{};
  // Depending on the value of angle_lb and angle_ub, we choose the range for
  // the sampled theta. If angle_lb and/or angle_ub is inf, then the theta_lb
  // and/or theta_ub will be set to a finite value.
  if (!std::isinf(angle_lb) && !std::isinf(angle_ub)) {
    theta_lb = angle_lb;
    theta_ub = angle_ub;
  } else if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta_lb = -2 * M_PI;
    theta_ub = 2 * M_PI;
  } else if (std::isinf(angle_lb)) {
    theta_lb = angle_ub - 2 * M_PI;
    theta_ub = angle_ub;
  } else {
    theta_lb = angle_lb;
    theta_ub = angle_lb + 2 * M_PI;
  }
  const Eigen::Matrix<double, kNumAngles, 1> theta =
      Eigen::Matrix<double, kNumAngles, 1>::LinSpaced(theta_lb, theta_ub);

  for (int i = 0; i < kNumAngles; ++i) {
    const Eigen::Matrix3d Ri =
        Eigen::AngleAxisd(theta(i), axis).toRotationMatrix();
    const double Ri_error = (Ri - M).squaredNorm();
    EXPECT_GE(Ri_error, R_error - 1E-10);
  }
}

GTEST_TEST(RotationMatrixTest, TestProjectionWithAxis) {
  // For a matrix on SO(3) with the desired axis, the projected matrix should
  // be the same, if the angle falls inside the bound.
  const Eigen::Vector3d axis(1.0 / 3.0, 2.0 / 3.0, -2.0 / 3.0);

  Eigen::Matrix3d M = Eigen::AngleAxisd(0.2, axis).toRotationMatrix();

  EXPECT_NEAR(ProjectMatToRotMatWithAxis(M, axis, 0, 1), 0.2, 1e-6);

  // If the angle of `M` falls outside of the angle bounds, then the optimal
  // projection is to one of the bound.
  EXPECT_NEAR(ProjectMatToRotMatWithAxis(M, axis, 0.3, 1), 0.3, 1e-6);

  // If the angle bounds include infinity, the the maximal angle is to shift 0.2
  // by 2kπ
  EXPECT_NEAR(ProjectMatToRotMatWithAxis(
                  M, axis, 0.3, std::numeric_limits<double>::infinity()),
              0.2 + 2 * M_PI, 1e-6);
  EXPECT_NEAR(ProjectMatToRotMatWithAxis(
                  M, axis, -std::numeric_limits<double>::infinity(), 0.1),
              0.2 - 2 * M_PI, 1e-6);
  EXPECT_NEAR(ProjectMatToRotMatWithAxis(
                  M, axis, -std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()),
              0.2, 1e-6);

  EXPECT_NEAR(ProjectMatToRotMatWithAxis(M, axis, -4, 0.1), 0.1, 1e-6);

  M = 2 * Eigen::AngleAxisd(M_PI_2, axis).toRotationMatrix();
  CheckProjectionWithAxis(M, axis, 0.1, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, -M_PI);

  M = 0.2 * Eigen::AngleAxisd(M_PI / 3, axis).toRotationMatrix();
  CheckProjectionWithAxis(M, axis, 0.1, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, -M_PI);

  // A random matrix.
  M << 0.1, 0.4, 1.2,
      -0.4, 2.3, 1.5,
       1.3, -.4, -0.2;
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, 0);
  CheckProjectionWithAxis(M, axis, 0.1, 0.2);
  CheckProjectionWithAxis(M, axis, -std::numeric_limits<double>::infinity(),
                          2 * M_PI);
  CheckProjectionWithAxis(M, axis, -M_PI,
                          std::numeric_limits<double>::infinity());
  CheckProjectionWithAxis(M, axis, -2 * M_PI, 4 * M_PI);
}
}  // namespace
}  // namespace math
}  // namespace drake
