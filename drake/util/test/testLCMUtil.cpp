#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/random_polynomial_matrix.h"
#include "drake/common/trajectories/test/random_piecewise_polynomial.h"
#include "drake/math/random_rotation.h"
#include "drake/util/lcmUtil.h"

#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;

namespace drake {
namespace {

// TODO(jwnimmer-tri) Unit tests should not use unseeded randomness.

GTEST_TEST(TestLcmUtil, testPolynomial) {
  default_random_engine generator;
  int max_num_coefficients = 5;
  uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  int num_coefficients = int_distribution(generator);
  VectorXd coefficients = VectorXd::Random(num_coefficients);
  Polynomial<double> poly(coefficients);
  drake::lcmt_polynomial msg;
  encodePolynomial(poly, msg);
  auto poly_back = decodePolynomial(msg);
  EXPECT_TRUE(poly.IsApprox(poly_back, 1e-8));
}

GTEST_TEST(TestLcmUtil, testPolynomialMatrix) {
  auto poly_matrix = drake::test::RandomPolynomialMatrix<double>(6, 5, 8);
  drake::lcmt_polynomial_matrix msg;
  encodePolynomialMatrix<Eigen::Dynamic, Eigen::Dynamic>(poly_matrix, msg);
  EXPECT_EQ(static_cast<int>(msg.rows), static_cast<int>(poly_matrix.rows()));
  EXPECT_EQ(static_cast<int>(msg.cols), static_cast<int>(poly_matrix.cols()));
  auto poly_matrix_back = decodePolynomialMatrix<Dynamic, Dynamic>(msg);
  EXPECT_EQ(poly_matrix.rows(), poly_matrix_back.rows());
  EXPECT_EQ(poly_matrix.cols(), poly_matrix_back.cols());
  for (int row = 0; row < msg.rows; ++row) {
    for (int col = 0; col < msg.cols; ++col) {
      EXPECT_TRUE(
          poly_matrix(row, col).IsApprox(poly_matrix_back(row, col), 1e-8));
    }
  }
}

GTEST_TEST(TestLcmUtil, testPiecewisePolynomial) {
  default_random_engine generator;
  int num_segments = 6;
  int rows = 4;
  int cols = 7;
  int num_coefficients = 3;
  std::vector<double> segment_times =
      PiecewiseFunction::randomSegmentTimes(num_segments, generator);
  PiecewisePolynomial<double> piecewise_polynomial =
      test::MakeRandomPiecewisePolynomial<double>(
          rows, cols, num_coefficients, segment_times);
  drake::lcmt_piecewise_polynomial msg;
  encodePiecewisePolynomial(piecewise_polynomial, msg);
  auto piecewise_polynomial_back = decodePiecewisePolynomial(msg);
  EXPECT_TRUE(piecewise_polynomial_back.isApprox(piecewise_polynomial, 1e-10));
}

GTEST_TEST(TestLcmUtil, testVector3d) {
  const int kVectorSize = 3;
  const Eigen::Vector3d vec = Vector3d::LinSpaced(Sequential, 0, kVectorSize);
  bot_core::vector_3d_t msg;
  EncodeVector3d(vec, msg);
  Eigen::Vector3d vec_back = DecodeVector3d(msg);
  EXPECT_TRUE(CompareMatrices(vec, vec_back, 0.0, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testQuaternion) {
  default_random_engine generator;
  generator.seed(0);
  const auto quaternion = drake::math::UniformlyRandomQuat(generator);
  bot_core::quaternion_t msg;
  EncodeQuaternion(quaternion, msg);
  auto quat_back = DecodeQuaternion(msg);
  EXPECT_TRUE(
      CompareMatrices(quaternion, quat_back, 0.0, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testPose) {
  default_random_engine generator;
  generator.seed(0);
  Eigen::Isometry3d pose;
  pose.linear() = drake::math::UniformlyRandomRotmat(generator);
  pose.translation().setLinSpaced(0, drake::kSpaceDimension);
  pose.makeAffine();
  const Eigen::Isometry3d& const_pose = pose;
  bot_core::position_3d_t msg;
  EncodePose(const_pose, msg);
  Eigen::Isometry3d pose_back = DecodePose(msg);
  EXPECT_TRUE(CompareMatrices(pose.matrix(), pose_back.matrix(), 1e-12,
                              MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testTwist) {
  const drake::TwistVector<double> twist =
      drake::TwistVector<double>::LinSpaced(Sequential, 0, drake::kTwistSize);
  bot_core::twist_t msg;
  EncodeTwist(twist, msg);
  auto twist_back = DecodeTwist(msg);
  EXPECT_TRUE(
      CompareMatrices(twist, twist_back, 0.0, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake

