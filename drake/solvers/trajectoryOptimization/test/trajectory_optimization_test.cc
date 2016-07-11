#include <iostream>

#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"

#include "gtest/gtest.h"

using namespace std;

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  // TODO(lgibson) kNumTimeSamples should be 21, using smaller # now for tests.
  const size_t kNumTimeSamples(5);
  DirectTrajectoryOptimization directTraj(1, 2, kNumTimeSamples, 2, 6);
  directTraj.GetInitialVars(4);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
