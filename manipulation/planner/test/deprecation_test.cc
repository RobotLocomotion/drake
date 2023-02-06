// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/manipulation/planner/robot_plan_interpolator.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

GTEST_TEST(ManipulationPlannerPathsDeprecation, ConstraintRelaxingIk) {
  // Exercise the *single* type declared in the header.
  EXPECT_NO_THROW(
      ConstraintRelaxingIk(FindResourceOrThrow(kIiwaUrdf), "iiwa_link_7"));
}

GTEST_TEST(ManipulationPlannerPathsDeprecation, RobotPlanInterpolator) {
  // Include *both* types declared in the header file.
  EXPECT_NO_THROW(RobotPlanInterpolator(FindResourceOrThrow(kIiwaUrdf),
                                        InterpolatorType::ZeroOrderHold));
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
