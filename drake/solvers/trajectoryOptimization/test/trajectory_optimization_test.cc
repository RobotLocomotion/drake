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
  const size_t kNumInputs(1);
  const size_t kNumStates(2);
  // TODO(lgibson) kNumTimeSamples should be 21, using smaller # now for tests.
  const size_t kNumTimeSamples(5);  // aka N. Segment for each time sample.
  DirectTrajectoryOptimization directTraj(kNumInputs, kNumStates,
      kNumTimeSamples, 2, 6);

  const size_t t_init_in(7);
  VectorXd s_times{VectorXd::LinSpaced(kNumTimeSamples, 0, t_init_in)};
  vector<double> segment_times(s_times.data(), s_times.data() +
    s_times.rows() * s_times.cols());
//  cout << "segmentTimes= " << endl;
//  std::copy(segment_times.begin(), segment_times.end(),
//    std::ostream_iterator<double>(std::cout, " "));

  const PiecewisePolynomialType inputs_u =
      PiecewisePolynomialType::random(kNumInputs, 1, 4, segment_times);
  const PiecewisePolynomialType states_x =
      PiecewisePolynomialType::random(kNumStates, 1, 6, segment_times);

  directTraj.GetInitialVars(t_init_in, inputs_u, states_x);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
