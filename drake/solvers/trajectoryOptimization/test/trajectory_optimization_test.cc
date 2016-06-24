#include <iostream>

#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"

#include "gtest/gtest.h"

using namespace std;

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  // TODO(lgibson) 3rd arg should be 21, using smaller # now for testing.
  DirectTrajectoryOptimization directTraj(1, 2, 5, 2, 6);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
