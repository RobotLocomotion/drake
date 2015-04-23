#include "PiecewisePolynomial.h"
#include <Eigen/Core>
#include <random>
#include <vector>
#include "testUtil.h"
#include "trajectoryTestUtil.h"
#include <iostream>

using namespace std;
using namespace Eigen;

default_random_engine generator;
uniform_real_distribution<double> uniform;

template <typename CoefficientType>
void testIntegralAndDerivative() {
  int num_coefficients = 5;
  int num_segments = 3;
  int rows = 3;
  int cols = 5;

  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;
  typedef typename PiecewisePolynomialType::CoefficientMatrix CoefficientMatrix;

  vector<double> segment_times = generateRandomSegmentTimes(num_segments, generator);
  PiecewisePolynomialType piecewise = generateRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);

  // differentiate integral, get original back
  PiecewisePolynomialType piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10))
    throw runtime_error("wrong");

  // check value at start time
  CoefficientMatrix desired_value_at_t0 = PiecewisePolynomialType::CoefficientMatrix::Random(piecewise.rows(), piecewise.cols());
  PiecewisePolynomialType integral = piecewise.integral(desired_value_at_t0);
  auto value_at_t0 = integral.matrixValue(piecewise.getStartTime());
  valuecheck(desired_value_at_t0, value_at_t0, 1e-10);

  // check continuity at knot points
  for (int i = 0; i < piecewise.getNumberOfSegments() - 1; ++i) {
    valuecheck(integral.getPolynomial(i).value(integral.getDuration(i)), integral.getPolynomial(i + 1).value(0.0));
  }
}

template <typename CoefficientType>
void testOperators() {
  int max_num_coefficients = 6;
  int num_tests = 100;
  default_random_engine generator;
  uniform_int_distribution<> int_distribution(1, max_num_coefficients);

  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;
  typedef typename PiecewisePolynomialType::CoefficientMatrix CoefficientMatrix;

  for (int i = 0; i < num_tests; ++i) {
    int num_coefficients = int_distribution(generator);
    int num_segments = int_distribution(generator);
    int rows = int_distribution(generator);
    int cols = int_distribution(generator);

    vector<double> segment_times = generateRandomSegmentTimes(num_segments, generator);
    PiecewisePolynomialType piecewise1 = generateRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise2 = generateRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);

    PiecewisePolynomialType sum = piecewise1 + piecewise2;

    uniform_real_distribution<double> uniform(piecewise1.getStartTime(), piecewise1.getEndTime());
    double t = uniform(generator);

    valuecheck(sum.value(t), piecewise1.value(t) + piecewise2.value(t), 1e-8);
  }
}

int main(int argc, char **argv) {
  testIntegralAndDerivative<double>();
  testOperators<double>();

  std::cout << "test passed";

  return 0;
}
