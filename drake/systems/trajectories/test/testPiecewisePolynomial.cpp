#include "PiecewisePolynomial.h"
#include <Eigen/Core>
#include <random>
#include <vector>
#include "testUtil.h"
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

  vector<double> segment_times = PiecewiseFunction::randomSegmentTimes(num_segments, generator);
  PiecewisePolynomialType piecewise = PiecewisePolynomial<CoefficientType>::random(rows, cols, num_coefficients, segment_times);

  // differentiate integral, get original back
  PiecewisePolynomialType piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10))
    throw runtime_error("wrong");

  // check value at start time
  CoefficientMatrix desired_value_at_t0 = PiecewisePolynomialType::CoefficientMatrix::Random(piecewise.rows(), piecewise.cols());
  PiecewisePolynomialType integral = piecewise.integral(desired_value_at_t0);
  auto value_at_t0 = integral.value(piecewise.getStartTime());
  valuecheckMatrix(desired_value_at_t0, value_at_t0, 1e-10);

  // check continuity at knot points
  for (int i = 0; i < piecewise.getNumberOfSegments() - 1; ++i) {
    valuecheck(integral.getPolynomial(i).value(integral.getDuration(i)), integral.getPolynomial(i + 1).value(0.0));
  }
}

template <typename CoefficientType>
void testBasicFunctionality() {
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

    vector<double> segment_times = PiecewiseFunction::randomSegmentTimes(num_segments, generator);
    PiecewisePolynomialType piecewise1 = PiecewisePolynomial<CoefficientType>::random(rows, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise2 = PiecewisePolynomial<CoefficientType>::random(rows, cols, num_coefficients, segment_times);

    normal_distribution<double> normal;
    double shift = normal(generator);
    CoefficientMatrix offset = CoefficientMatrix::Random(piecewise1.rows(), piecewise1.cols());

    PiecewisePolynomialType sum = piecewise1 + piecewise2;
    PiecewisePolynomialType difference = piecewise2 - piecewise1;
    PiecewisePolynomialType piecewise1_plus_offset = piecewise1 + offset;
    PiecewisePolynomialType piecewise1_minus_offset = piecewise1 - offset;
    PiecewisePolynomialType piecewise1_shifted = piecewise1;
    piecewise1_shifted.shiftRight(shift);

    uniform_real_distribution<double> uniform(piecewise1.getStartTime(), piecewise1.getEndTime());
    double t = uniform(generator);

    valuecheckMatrix(sum.value(t), piecewise1.value(t) + piecewise2.value(t), 1e-8);
    valuecheckMatrix(difference.value(t), piecewise2.value(t) - piecewise1.value(t), 1e-8);
    valuecheckMatrix(piecewise1_plus_offset.value(t), piecewise1.value(t) + offset, 1e-8);
    valuecheckMatrix(piecewise1_minus_offset.value(t), piecewise1.value(t) - offset, 1e-8);
    valuecheckMatrix(piecewise1_shifted.value(t), piecewise1.value(t - shift), 1e-8);
  }
}

template <typename CoefficientType>
void testValueOutsideOfRange() {
  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;

  default_random_engine generator;
  vector<double> segment_times = PiecewiseFunction::randomSegmentTimes(6, generator);
  PiecewisePolynomialType piecewise = PiecewisePolynomial<CoefficientType>::random(3, 4, 5, segment_times);
  valuecheckMatrix(piecewise.value(piecewise.getStartTime()), piecewise.value(piecewise.getStartTime() - 1.0), 1e-10);
  valuecheckMatrix(piecewise.value(piecewise.getEndTime()), piecewise.value(piecewise.getEndTime() + 1.0), 1e-10);
}

int main(int argc, char **argv) {
  testIntegralAndDerivative<double>();
  testBasicFunctionality<double>();
  testValueOutsideOfRange<double>();

  std::cout << "test passed";

  return 0;
}
