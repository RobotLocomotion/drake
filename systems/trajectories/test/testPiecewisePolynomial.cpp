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

vector<double> generateSegmentTimes(int num_segments) {
  vector<double> segment_times;
  double t0 = uniform(generator);
  segment_times.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = uniform(generator);
    segment_times.push_back(segment_times[i] + duration);
  }
  return segment_times;
}

template<typename CoefficientType>
PiecewisePolynomial<CoefficientType> createRandomPiecewisePolynomial(int rows, int cols, int num_coefficients, const vector<double>& segment_times)
{
  int num_segments = segment_times.size() - 1;
  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;
  typedef typename PiecewisePolynomialType::PolynomialType PolynomialType;
  typedef typename PiecewisePolynomialType::PolynomialMatrix PolynomialMatrix;
  typedef typename PiecewisePolynomialType::CoefficientMatrix CoefficientMatrix;
  vector<PolynomialMatrix> polynomials;
  int size = rows * cols;
  typedef typename PolynomialType::CoefficientsType CoefficientsType;
  for (int segment_index = 0; segment_index < num_segments; ++segment_index) {
    PolynomialMatrix matrix(rows, cols);
    for (int i = 0; i < size; i++) {
      CoefficientsType coefficients = CoefficientsType::Random(num_coefficients);
      matrix(i) = PolynomialType(coefficients);
    }
    polynomials.push_back(matrix);
  }
  PiecewisePolynomialType piecewise(polynomials, segment_times);
  return piecewise;
}

template <typename CoefficientType>
void testIntegralAndDerivative() {
  int num_coefficients = 5;
  int num_segments = 3;
  int rows = 3;
  int cols = 5;

  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;
  typedef typename PiecewisePolynomialType::CoefficientMatrix CoefficientMatrix;

  vector<double> segment_times = generateSegmentTimes(num_segments);
  PiecewisePolynomialType piecewise = createRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);

  // differentiate integral, get original back
  PiecewisePolynomialType piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10))
    throw runtime_error("wrong");

  // check value at start time
  CoefficientMatrix desired_value_at_t0 = decltype(piecewise)::CoefficientMatrix::Random(piecewise.rows(), piecewise.cols());
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

    vector<double> segment_times = generateSegmentTimes(num_segments);
    PiecewisePolynomialType piecewise1 = createRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise2 = createRandomPiecewisePolynomial<CoefficientType>(rows, cols, num_coefficients, segment_times);

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
