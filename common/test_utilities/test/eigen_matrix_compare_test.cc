#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace {

// Tests the ability for two identical matrices to be compared.
GTEST_TEST(MatrixCompareTest, CompareIdentical) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, 2, 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1, 2, 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 100, 200, 300, 400;

  const double tolerance = 1e-8;

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::relative));

  EXPECT_FALSE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute));

  EXPECT_FALSE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::relative));
}

// Tests absolute tolerance with real numbers.
GTEST_TEST(MatrixCompareTest, AbsoluteCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, 2, 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1 - 1e-10, 2, 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 0, 1, 2 - 1e-8, 3;

  Eigen::MatrixXd m4(2, 2);
  m4 << 0, 1, 2, 3 - 1e-6;

  const double tolerance = 1e-8;

  // The difference between m1 and m2 is less than the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute));

  // The difference between m1 and m3 is exactly equal to the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute));

  // The difference between m1 and m4 is greater than the tolerance.
  // They should be considered different.
  EXPECT_FALSE(CompareMatrices(m1, m4, tolerance, MatrixCompareType::absolute));
}

// Tests absolute tolerance with NaN values
GTEST_TEST(MatrixCompareTest, AbsoluteNaNCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 0, 1 - 1e-10, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m4(2, 2);
  m4 << 0, 1, 2, 3;

  const double tolerance = 1e-8;

  // The difference between m1 and m2 is less than the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute));

  // The difference between m1 and m3 is exactly equal to the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute));

  // The difference between m1 and m4 is greater than the tolerance.
  // They should be considered different.
  EXPECT_FALSE(CompareMatrices(m1, m4, tolerance, MatrixCompareType::absolute));
}

// Tests absolute tolerance with real numbers.
GTEST_TEST(MatrixCompareTest, RelativeCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 100, 100, 100, 100;

  Eigen::MatrixXd m2(2, 2);
  m2 << 100, 100 * 0.9, 100, 100;

  // The difference between m1 and m2 is more than 1%.
  // They should be considered not equal.
  EXPECT_FALSE(CompareMatrices(m1, m2, 0.01, MatrixCompareType::relative));

  // The difference between m1 and m2 is equal to 10%.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, 0.1, MatrixCompareType::relative));

  // The difference between m1 and m4 is less than 20%.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, 0.2, MatrixCompareType::relative));
}

GTEST_TEST(MatrixCompareTest, RotationMatrix) {
  const Vector3<double> Bx(1, 0, 0);
  const Vector3<double> By(0, 0, -1);
  const Vector3<double> Bz(0, 1, 0);
  const math::RotationMatrix<double> R =
      math::RotationMatrix<double>::MakeFromOrthonormalColumns(Bx, By, Bz);
  const math::RotationMatrix<double> R_inv = R.inverse();
  const math::RotationMatrix<double> R_inv_inv = R_inv.inverse();

  // The matrix we've defined has no precision loss through inversion; so we
  // can expect perfect matches.
  EXPECT_TRUE(CompareMatrices(R, R_inv_inv));
  EXPECT_FALSE(CompareMatrices(R, R_inv));

  // Confirm that the additional flags (tolerance and relative/absolute error)
  // get properly exercised.
  const math::RotationMatrix<double> R2 =
      R * math::RotationMatrix<double>::MakeXRotation(1e-10);
  EXPECT_FALSE(CompareMatrices(R, R2));
  EXPECT_TRUE(CompareMatrices(R, R2, 1e-10));

  // A valid rotation matrix essentially always has the same "magnitude"; it's
  // an orthonormal matrix. So, it can be tricky to detect the difference
  // between a relative and absolute comparision. We won't stress about the
  // distinction in this test, safe in the assumption that no one will ever
  // provide that argument and expect a different outcome.
  EXPECT_TRUE(CompareMatrices(R, R2, 1e-10, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(R, R2, 1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(MatrixCompareTest, RigidTransform) {
  const Vector3<double> Bx(1, 0, 0);
  const Vector3<double> By(0, 0, -1);
  const Vector3<double> Bz(0, 1, 0);
  const math::RotationMatrix<double> R =
      math::RotationMatrix<double>::MakeFromOrthonormalColumns(Bx, By, Bz);

  const math::RigidTransform<double> X(R, Vector3<double>(10, 20, 30));
  const math::RigidTransform<double> X_inv = X.inverse();
  const math::RigidTransform<double> X_inv_inv = X_inv.inverse();

  // The matrix we've defined has no precision loss through inversion; so we
  // can expect perfect matches.
  EXPECT_TRUE(CompareMatrices(X, X_inv_inv));
  EXPECT_FALSE(CompareMatrices(X, X_inv));

  // Confirm that the additional flags (tolerance and relative/absolute error)
  // get properly exercised.
  const double delta = 1e-10;
  const math::RigidTransform<double> X2 =
      X * math::RigidTransform<double>(Vector3<double>(delta, delta, delta));
  EXPECT_FALSE(CompareMatrices(X, X2));
  EXPECT_TRUE(CompareMatrices(X, X2, 2 * delta));

  // Relative vs absolute error is accounted for; the same tolerance fails in
  // one mode, but passes in the other.
  EXPECT_TRUE(CompareMatrices(X, X2, delta, MatrixCompareType::relative));
  EXPECT_FALSE(CompareMatrices(X, X2, delta, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake
