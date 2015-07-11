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

template <typename CoefficientType>
void testIntegralAndDerivative() {
  vector<Polynomial<CoefficientType>> polynomials;
  int num_coefficients = 5;
  int num_segments = 3;
  typedef Matrix<CoefficientType,Dynamic,1> CoefficientsType;
  for (int i = 0; i < num_segments; ++i) {
    CoefficientsType coefficients = CoefficientsType::Random(num_coefficients);
    polynomials.push_back(Polynomial<CoefficientType>(coefficients));
  }

  // differentiate integral, get original back
  PiecewisePolynomial<CoefficientType> piecewise(polynomials, generateSegmentTimes(num_segments));
  PiecewisePolynomial<CoefficientType> piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10))
    throw runtime_error("wrong");

  // check value at start time
  double value_at_t0 = uniform(generator);
  PiecewisePolynomial<CoefficientType> integral = piecewise.integral(value_at_t0);
  valuecheck(value_at_t0, integral.value(piecewise.getStartTime()), 1e-10);

  // check continuity at knot points
  for (int i = 0; i < piecewise.getNumberOfSegments() - 1; ++i) {
    valuecheck(integral.getPolynomial(i).value(integral.getDuration(i)), integral.getPolynomial(i + 1).value(0.0));
  }
}

int main(int argc, char **argv) {
  testIntegralAndDerivative<double>();

  std::cout << "test passed";

  return 0;
}
