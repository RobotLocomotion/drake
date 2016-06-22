#include <iostream>

#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"

#include "gtest/gtest.h"

using namespace std;

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  DirectTrajectoryOptimization directTraj(1, 2, 21, 2, 6);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
