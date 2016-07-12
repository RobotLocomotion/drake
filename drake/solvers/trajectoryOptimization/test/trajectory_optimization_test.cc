#include <iostream>
#include <random>
#include <vector>

#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "gtest/gtest.h"

using namespace std;
using std::default_random_engine;
using std::vector;

namespace drake {
namespace solvers {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

default_random_engine generator;

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  int num_coefficients = 5;
  int num_segments = 3;
  int rows = 3;
  int cols = 5;
  // TODO(lgibson) kNumTimeSamples should be 21, using smaller # now for tests.
  const size_t kNumTimeSamples(5);
  
  vector<double> segment_times =
      PiecewiseFunction::randomSegmentTimes(num_segments, generator);
  const PiecewisePolynomialType piecewise =
      PiecewisePolynomialType::random(rows, cols, num_coefficients, 
                                      segment_times);
  DirectTrajectoryOptimization directTraj(1, 2, kNumTimeSamples, 2, 6);
  directTraj.GetInitialVars(4, piecewise, piecewise);
  
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
