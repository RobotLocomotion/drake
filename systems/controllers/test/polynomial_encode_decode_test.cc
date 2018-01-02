#include "drake/systems/controllers/polynomial_encode_decode.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/random_polynomial_matrix.h"
#include "drake/common/trajectories/test/random_piecewise_polynomial.h"
#include "drake/math/random_rotation.h"

using Eigen::Dynamic;
using Eigen::VectorXd;

namespace drake {
namespace {

// TODO(jwnimmer-tri) Unit tests should not use unseeded randomness.

GTEST_TEST(TestLcmUtil, testPolynomial) {
  std::default_random_engine generator;
  int max_num_coefficients = 5;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);
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
  std::default_random_engine generator;
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

}  // namespace
}  // namespace drake

