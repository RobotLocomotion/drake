#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include <cmath>
#include <random>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/trajectories/test/random_piecewise_polynomial.h"

using std::default_random_engine;
using std::uniform_real_distribution;

namespace drake {
namespace {

template <typename CoefficientType>
void testSimpleCase() {
  typedef ExponentialPlusPiecewisePolynomial<CoefficientType>
      ExponentialPlusPiecewisePolynomialType;
  typedef typename ExponentialPlusPiecewisePolynomialType::MatrixX MatrixX;

  int num_coefficients = 5;
  int num_segments = 1;

  MatrixX K = MatrixX::Random(1, 1);
  MatrixX A = MatrixX::Random(1, 1);
  MatrixX alpha = MatrixX::Random(1, 1);

  default_random_engine generator;
  auto segment_times =
      PiecewiseFunction::randomSegmentTimes(num_segments, generator);
  auto polynomial_part = test::MakeRandomPiecewisePolynomial<CoefficientType>(
      1, 1, num_coefficients, segment_times);

  ExponentialPlusPiecewisePolynomial<CoefficientType> expPlusPp(
      K, A, alpha, polynomial_part);
  ExponentialPlusPiecewisePolynomial<CoefficientType> derivative =
      expPlusPp.derivative();

  uniform_real_distribution<CoefficientType> uniform(expPlusPp.getStartTime(),
                                                     expPlusPp.getEndTime());
  double t = uniform(generator);
  auto check =
      K(0) * std::exp(A(0) * (t - expPlusPp.getStartTime())) * alpha(0) +
      polynomial_part.scalarValue(t);
  auto derivative_check =
      K(0) * A(0) * std::exp(A(0) * (t - expPlusPp.getStartTime())) * alpha(0) +
      polynomial_part.derivative().scalarValue(t);

  EXPECT_NEAR(check, expPlusPp.value(t)(0), 1e-8);
  EXPECT_NEAR(derivative_check, derivative.value(t)(0), 1e-8);
}

GTEST_TEST(testExponentialPlusPiecewisePolynomial, BasicTest) {
  testSimpleCase<double>();
}

}  // namespace
}  // namespace drake
