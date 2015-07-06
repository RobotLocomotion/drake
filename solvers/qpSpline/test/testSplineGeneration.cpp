#include "splineGeneration.h"
#include <random>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include "testUtil.h"

using namespace std;

default_random_engine generator;
uniform_real_distribution<double> uniform;

vector<double> generateSegmentTimes(int num_segments) {
  vector<double> segment_times;
  double t0 = 0.0; // uniform(generator);
  segment_times.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = 1.0; // uniform(generator);
    segment_times.push_back(segment_times[i] + duration);
  }
  return segment_times;
}

double randomSpeedTest(int ntests) {
  double ret = 0.0;
  int num_segments = 3;
  vector<double> segment_times = generateSegmentTimes(num_segments);
  for (int i = 0; i < ntests; i++) {

    double x0 = uniform(generator);
    double xd0 = uniform(generator);

    double xf = uniform(generator);
    double xdf = uniform(generator);

    double x1 = uniform(generator);
    double x2 = uniform(generator);

    PiecewisePolynomial<double> result = twoWaypointCubicSpline(segment_times, x0, xd0, xf, xdf, x1, x2);
    ret += result.value(0.0);
  }
  return ret;
}

int main(int argc, char **argv) {
  int num_segments = 3;

  vector<double> segment_times = generateSegmentTimes(num_segments);

  double x0 = uniform(generator);
  double xd0 = uniform(generator);

  double xf = uniform(generator);
  double xdf = uniform(generator);

  double x1 = uniform(generator);
  double x2 = uniform(generator);


  PiecewisePolynomial<double> result = twoWaypointCubicSpline(segment_times, x0, xd0, xf, xdf, x1, x2);

  for (int i = 0; i < num_segments; i++) {
    valuecheck(segment_times[i], result.getStartTime(i));
  }
  valuecheck(segment_times[num_segments], result.getEndTime(num_segments - 1));

  // check value constraints
  double tol = 1e-10;
  PiecewisePolynomial<double> derivative = result.derivative();
  PiecewisePolynomial<double> second_derivative = derivative.derivative();


  valuecheck(result.value(result.getStartTime(0)), x0, tol);
  valuecheck(derivative.value(result.getStartTime(0)), xd0, tol);


  valuecheck(result.value(result.getEndTime(num_segments - 1)), xf, tol);
  valuecheck(derivative.value(result.getEndTime(num_segments - 1)), xdf, tol);

  valuecheck(result.value(result.getStartTime(1)), x1, tol);
  valuecheck(result.value(result.getStartTime(2)), x2, tol);

  // check continuity constraints
  double eps = 1e-10;
  int num_knots = num_segments - 1;
  for (int i = 0; i < num_knots; i++) {
    double t_knot = result.getEndTime(i);
    valuecheck(result.value(t_knot - eps), result.value(t_knot + eps), 1e-8);
    valuecheck(derivative.value(t_knot - eps), derivative.value(t_knot + eps), 1e-8);
    valuecheck(second_derivative.value(t_knot - eps), second_derivative.value(t_knot + eps), 1e-8);
  }

#if !defined(WIN32) && !defined(WIN64)
  int ntests = 1000;
  cout << "time: " << measure<chrono::microseconds>::execution(randomSpeedTest, ntests) / (double) ntests << " microseconds." << endl;
#endif

  cout << "test passed" << endl;

  return 0;
}
