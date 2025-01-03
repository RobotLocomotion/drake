#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/planning/dev/sphere_robot_model_collision_checker.h"

namespace drake {
namespace planning {
namespace test {

// Even though the static type of the parameterized test value is just
// CollisionChecker, the runtime param value should always be a
// subclass of SphereRobotModelCollisionChecker.
class SphereRobotModelCollisionCheckerAbstractTestSuite
    : public testing::TestWithParam<std::shared_ptr<CollisionChecker>> {};

}  // namespace test
}  // namespace planning
}  // namespace drake
