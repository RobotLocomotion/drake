#include "drake/math/quadratic_form.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {
void CheckDecomposePSDmatrixIntoXtransposeTimesX(
    const Eigen::Ref<const Eigen::MatrixXd>& Y, double zero_tol,
    double check_tol = 1E-14) {
  const Eigen::MatrixXd X = DecomposePSDmatrixIntoXtransposeTimesX(Y, zero_tol);
  EXPECT_TRUE(CompareMatrices(X.transpose() * X, Y, check_tol,
                              MatrixCompareType::absolute));
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Y);
  qr.setThreshold(check_tol);
  EXPECT_EQ(qr.rank(), X.rows());
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test0) {
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Eigen::Matrix3d::Identity(),
                                              1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test1) {
  const Eigen::Matrix3d Y = Eigen::Vector3d(1, 4, 0).asDiagonal();
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, 1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test2) {
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 1, 0,
       1, 1, 0,
       0, 0, 0;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, 1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test3) {
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 2, 0,
       2, 4, 0,
       0, 0, 9;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, 1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test4) {
  // Y is a rank 1 psd matrix.
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 2, -3,
       2, 4, -6,
       -3, -6, 9;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, 1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test5) {
  // Y is a rank 2 psd matrix.
  Eigen::Matrix3d Y1;
  Eigen::Matrix3d Y2;
  // clang-format off
  Y1 << 1, 2, -3,
        2, 4, -6,
       -3, -6, 9;
  Y2 << 0, 0, 0,
        0, 1, -1,
        0, -1, 1;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y1 + Y2, 1E-15);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, negativeY) {
  // Y is a negative definite matrix.
  EXPECT_THROW(
      DecomposePSDmatrixIntoXtransposeTimesX(-Eigen::Matrix3d::Identity(), 0),
      std::runtime_error);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, indefiniteY) {
  // Y is an indefinite matrix.
  Eigen::Matrix4d Y;
  // clang-format off
  Y << 1, 2, 0, 1,
       2, 4, 0, 2,
       0, 0, 0, 1,
       1, 2, 1, 1;
  // clang-format on
  EXPECT_THROW(DecomposePSDmatrixIntoXtransposeTimesX(Y, 0),
               std::runtime_error);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, almost_psd_Y) {
  // Y is almost PSD.
  Eigen::Matrix3d U =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d Y =
      U * Eigen::Vector3d(1, -1E-10, 2).asDiagonal() * U.transpose();
  // With tolerance being 0, DecomposePSDmatrixIntoXtransposeTimesX should
  // detect Y is not PSD.
  EXPECT_THROW(DecomposePSDmatrixIntoXtransposeTimesX(Y, 0),
               std::runtime_error);
  // With tolerance being 1E-10, it should regard Y as a PSD matrix.
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, 2E-10, 1E-9);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, negative_tol) {
  EXPECT_THROW(DecomposePSDmatrixIntoXtransposeTimesX(
                   Eigen::Matrix3d::Identity(), -1E-10),
               std::runtime_error);
}

void CheckDecomposePositiveQuadraticForm(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c, double tol_psd = 0) {
  Eigen::MatrixXd R;
  Eigen::VectorXd d;
  const double tol_check = 1E-10;
  std::tie(R, d) = DecomposePositiveQuadraticForm(Q, b, c, tol_psd);
  EXPECT_TRUE(CompareMatrices(R.transpose() * R, Q, tol_check,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(R.transpose() * d, b / 2, tol_check,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(d.squaredNorm(), c, tol_check);
  Eigen::MatrixXd Y(Q.rows() + 1, Q.rows() + 1);
  Y << Q, b/2, b.transpose() / 2, c;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Y);
  EXPECT_EQ(qr.rank(), R.rows());
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test0) {
  // Decomposes a positive quadratic form without linear terms.
  // The quadratic form is 4x² + 4y² + 9.
  // This quadratic form is the same as zᵀ*z, where z is the vector
  // [2x]
  // [2y]
  // [ 3]
  Eigen::Matrix2d Q = 4 * Eigen::Matrix2d::Identity();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
  double c = 9;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test1) {
  // Decomposes a positive quadratic form with linear terms.
  //   x² + 4xy + 4y² + 2x + 4y + 2
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 4;
  Eigen::Vector2d b(2, 4);
  double c = 2;
  CheckDecomposePositiveQuadraticForm(Q, b, c, 1E-15);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test2) {
  // Decomposes a positive quadratic form with both linear and cross
  // terms.
  // x² + 2xy + 4y² + 4y +4
  Eigen::Matrix2d Q;
  Q << 1, 1, 1, 4;
  Eigen::Vector2d b(0, 4);
  double c = 2;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test3) {
  // Decomposes a positive form with no constant term.
  // x² + 4xy + 4y²
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 4;
  Eigen::Vector2d b(0, 0);
  double c = 0;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test4) {
  Eigen::Matrix2d Q;
  Q << 1, 1.5, 1.5, 1;
  Eigen::Vector2d b(0, 0);
  double c = 0;
  EXPECT_THROW(DecomposePositiveQuadraticForm(Q, b, c), std::runtime_error);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test5) {
  Eigen::Matrix3d Q;
  // clang-format off
  Q << 1, 2, 0,
       2, 4, 0,
       0, 0, 0;
  // clang-format on
  Eigen::Vector3d b(2, 4, -1);
  double c = 1;
  EXPECT_THROW(DecomposePositiveQuadraticForm(Q, b, c), std::runtime_error);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test6) {
  Eigen::Matrix3d Q;
  // clang-format off
  Q << 1, 2, 0,
       2, 4, 0,
       0, 0, 0;
  // clang-format on
  Eigen::Vector3d b(2, 4, -1);
  double c = 1;
  // tolerance has to be non-negative.
  EXPECT_THROW(DecomposePositiveQuadraticForm(Q, b, c, -1E-15),
               std::runtime_error);
}
}  // namespace
}  // namespace math
}  // namespace drake
