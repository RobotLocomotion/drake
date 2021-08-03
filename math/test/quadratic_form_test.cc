#include "drake/math/quadratic_form.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
namespace {

const double kDefaultZeroTol = 2E-15;

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
                                              kDefaultZeroTol);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test1) {
  const Eigen::Matrix3d Y = Eigen::Vector3d(1, 4, 0).asDiagonal();
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, kDefaultZeroTol);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test2) {
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 1, 0,
       1, 1, 0,
       0, 0, 0;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, kDefaultZeroTol);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test3) {
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 2, 0,
       2, 4, 0,
       0, 0, 9;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, kDefaultZeroTol);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, Test4) {
  // Y is a rank 1 psd matrix.
  Eigen::Matrix3d Y;
  // clang-format off
  Y << 1, 2, -3,
       2, 4, -6,
       -3, -6, 9;
  // clang-format on
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y, kDefaultZeroTol);
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
  CheckDecomposePSDmatrixIntoXtransposeTimesX(Y1 + Y2, kDefaultZeroTol);
}

GTEST_TEST(TestDecomposePSDmatrixIntoXtransposeTimesX, negativeY) {
  // Y is a negative definite matrix.
  DRAKE_EXPECT_THROWS_MESSAGE(
      DecomposePSDmatrixIntoXtransposeTimesX(-Eigen::Matrix3d::Identity(), 0),
      "Y is not positive definite. It has an eigenvalue -1.* that is more "
      "negative than the tolerance 0.*.");
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
  const RotationMatrixd U = RotationMatrixd::MakeYRotation(M_PI / 2);
  const Eigen::Matrix3d Y = U.matrix()
                          * Eigen::Vector3d(1, -1E-10, 2).asDiagonal()
                          * U.transpose().matrix();
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
  CheckDecomposePositiveQuadraticForm(Q, b, c, kDefaultZeroTol);
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
  // Decomposes a positive form with no constant or linear term, Q is not full
  // rank. x² + 4xy + 4y²
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 4;
  Eigen::Vector2d b(0, 0);
  double c = 0;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
  Eigen::MatrixXd R;
  Eigen::VectorXd d;
  // Make sure that R.rows() = rank(Q) (since b and c = 0).
  std::tie(R, d) = DecomposePositiveQuadraticForm(Q, b, c);
  EXPECT_EQ(R.rows(), 1);
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
  EXPECT_THROW(DecomposePositiveQuadraticForm(Q, b, c, -kDefaultZeroTol),
               std::runtime_error);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test7) {
  // Decomposes a positive form with no constant or linear term, Q is full rank.
  // x² + 4xy + 5y²
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 5;
  Eigen::Vector2d b(0, 0);
  double c = 0;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
  Eigen::MatrixXd R;
  Eigen::VectorXd d;
  // Make sure that R.rows() = rank(Q) (since b and c = 0).
  std::tie(R, d) = DecomposePositiveQuadraticForm(Q, b, c);
  EXPECT_EQ(R.rows(), 2);
}

void CheckBalancing(const Eigen::Matrix3d& S, const Eigen::Matrix3d& P,
                   const Eigen::MatrixXd& T) {
  const Eigen::MatrixXd D = T.transpose() * S * T;
  const Eigen::MatrixXd Dinv = (T.transpose() * P * T).cwiseAbs();

  // Check that D and Dinv are diagonal.
  EXPECT_TRUE(CompareMatrices(D, Eigen::MatrixXd(D.diagonal().asDiagonal()),
                              1e-11));
  EXPECT_TRUE(
      CompareMatrices(Dinv, Eigen::MatrixXd(Dinv.diagonal().asDiagonal()),
                      1e-11));

  // Check that Dinv is, in fact, the inverse of D.
  EXPECT_TRUE(CompareMatrices(D.inverse(), Dinv, 1e-10));
}

GTEST_TEST(MatrixUtilTest, BalanceQuadraticFormsTest) {
  Eigen::Matrix3d A, B;
  // clang-format off
  A << 1, 2, 4,
      2, 3, 5,
      4, 5, 6;
  B << 7,  8,  9,
      8, 10, 11,
      9, 11, 12;
  // clang-format on

  const Eigen::Matrix3d S = A * A.transpose();
  const Eigen::Matrix3d P = B * B.transpose();
  const Eigen::MatrixXd T = BalanceQuadraticForms(S, P);
  CheckBalancing(S, P, T);

  const Eigen::MatrixXd T2 = BalanceQuadraticForms(P, S);
  CheckBalancing(P, S, T2);

  // Now confirm that P need not have been PSD.
  EXPECT_FALSE(math::IsPositiveDefinite(A));
  const Eigen::MatrixXd T3 = BalanceQuadraticForms(P, A);
  CheckBalancing(P, A, T3);
  EXPECT_FALSE(math::IsPositiveDefinite(B));
  const Eigen::MatrixXd T4 = BalanceQuadraticForms(S, B);
  CheckBalancing(S, B, T4);
}

}  // namespace
}  // namespace math
}  // namespace drake
