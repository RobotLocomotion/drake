#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "planning/sphere_robot_model_collision_checker.h"

namespace anzu {
namespace planning {
namespace test {

// Even though the static type of the parameterized test value is just
// CollisionChecker, the runtime param value should always be a
// subclass of SphereRobotModelCollisionChecker.
class SphereRobotModelCollisionCheckerAbstractTestSuite
    : public testing::TestWithParam<
          std::shared_ptr<drake::planning::CollisionChecker>> {};

}  // namespace test
}  // namespace planning
}  // namespace anzu
