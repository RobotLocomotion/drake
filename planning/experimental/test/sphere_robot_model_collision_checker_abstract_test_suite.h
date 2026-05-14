#pragma once

#include <memory>
#include <ostream>

#include <gtest/gtest.h>

#include "drake/planning/experimental/sphere_robot_model_collision_checker.h"

namespace drake {
namespace planning {
namespace experimental {
namespace test {

/* These parameters are passed to the abstract test cases to make them concrete,
for now just a collision checker of some derived type */
struct SphereRobotModelCollisionCheckerTestParams {
  std::shared_ptr<CollisionChecker> checker;
};

std::ostream& operator<<(std::ostream& out,
                         const SphereRobotModelCollisionCheckerTestParams& p);

// Even though the static type of the parameterized test value is just
// CollisionChecker, the runtime param value should always be a
// subclass of SphereRobotModelCollisionChecker.
class SphereRobotModelCollisionCheckerAbstractTestSuite
    : public testing::TestWithParam<
          SphereRobotModelCollisionCheckerTestParams> {};

}  // namespace test
}  // namespace experimental
}  // namespace planning
}  // namespace drake

// testing::PrintToString
