#include "splineGeneration.h"
#include <random>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iostream>

using namespace std;

void valueCheck(double val, double desired_val, double tol) {
  if (abs(val - desired_val) > tol) {
    stringstream msg;
    msg << "Expected: " << desired_val << ", but got: " << val << endl;
    throw runtime_error(msg.str().c_str());
  }
}

SplineInformation createSplineRequirements(const vector<int>& segment_polynomial_orders, const vector<double>& segment_times, double x0, double xd0, double xf, double xdf, double x1, double x2)
{
  SplineInformation spline_information(segment_polynomial_orders, segment_times);
  int num_segments = spline_information.getNumberOfSegments();
  spline_information.addValueConstraint(0, ValueConstraint(0, spline_information.getStartTime(0), x0));
  spline_information.addValueConstraint(0, ValueConstraint(1, spline_information.getStartTime(0), xd0));
  spline_information.addValueConstraint(num_segments - 1, ValueConstraint(0, spline_information.getEndTime(num_segments - 1), xf));
  spline_information.addValueConstraint(num_segments - 1, ValueConstraint(1, spline_information.getEndTime(num_segments - 1), xdf));
  spline_information.addValueConstraint(1, ValueConstraint(0, spline_information.getStartTime(1), x1));
  spline_information.addValueConstraint(2, ValueConstraint(0, spline_information.getStartTime(2), x2));

  int num_knots = num_segments - 1;
  for (int i = 0; i < num_knots; i++) {
    for (int derivative_order = 0; derivative_order < 3; derivative_order++) {
      spline_information.addContinuityConstraint(ContinuityConstraint(derivative_order, i, i + 1));
    }
  }
  return spline_information;
}

int main(int argc, char **argv) {

  default_random_engine generator;
  uniform_real_distribution<double> uniform;

  int num_segments = 3;
  int polynomial_order = 3;

  vector<int> segment_polynomial_orders;
  for (int i = 0; i < num_segments; ++i) {
    segment_polynomial_orders.push_back(polynomial_order);
  }

  vector<double> segment_times;
  double t0 = 0.0; // uniform(generator);
  segment_times.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = 1.0; // uniform(generator);
    segment_times.push_back(segment_times[i] + duration);
  }


  double x0 = uniform(generator);
  double xd0 = uniform(generator);

  double xf = uniform(generator);
  double xdf = uniform(generator);

  double x1 = uniform(generator);
  double x2 = uniform(generator);

  SplineInformation spline_information = createSplineRequirements(segment_polynomial_orders, segment_times, x0, xd0, xf, xdf, x1, x2);
  PiecewisePolynomial result = generateSpline(spline_information);

  // check value constraints
  double tol = 1e-10;
  PiecewisePolynomial derivative = result.derivative();
  PiecewisePolynomial second_derivative = derivative.derivative();

  valueCheck(result.value(spline_information.getStartTime(0)), x0, tol);
  valueCheck(derivative.value(spline_information.getStartTime(0)), xd0, tol);

  valueCheck(result.value(spline_information.getEndTime(num_segments - 1)), xf, tol);
  valueCheck(derivative.value(spline_information.getEndTime(num_segments - 1)), xdf, tol);

  valueCheck(result.value(spline_information.getStartTime(1)), x1, tol);
  valueCheck(derivative.value(spline_information.getStartTime(2)), x2, tol);

  // check continuity constraints
  double eps = 1e-10;
  int num_knots = num_segments - 1;
  for (int i = 0; i < num_knots; i++) {
    double t_knot = spline_information.getEndTime(i);
    valueCheck(result.value(t_knot - eps), result.value(t_knot + eps), 1e-8);
    valueCheck(derivative.value(t_knot - eps), derivative.value(t_knot + eps), 1e-8);
    valueCheck(second_derivative.value(t_knot - eps), second_derivative.value(t_knot + eps), 1e-8);
  }

  return 0;
}
