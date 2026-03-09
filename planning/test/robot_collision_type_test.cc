#include "drake/planning/robot_collision_type.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace drake {
namespace planning {
namespace {

constexpr RobotCollisionType kNoCollision = RobotCollisionType::kNoCollision;
constexpr RobotCollisionType kEnvironmentCollision =
    RobotCollisionType::kEnvironmentCollision;
constexpr RobotCollisionType kSelfCollision =
    RobotCollisionType::kSelfCollision;
constexpr RobotCollisionType kEnvironmentAndSelfCollision =
    RobotCollisionType::kEnvironmentAndSelfCollision;

struct Case {
  RobotCollisionType input{RobotCollisionType::kNoCollision};
  bool in_collision{};
  RobotCollisionType output{RobotCollisionType::kNoCollision};

  std::string to_string() const {
    return fmt::format("input {}, in_collision {}, output {}",
                       static_cast<int>(input), in_collision,
                       static_cast<int>(output));
  }
};

GTEST_TEST(CollisionCheckerTest, EnvCollisionBits) {
  Case cases[] = {
      {kNoCollision, false, kNoCollision},
      {kNoCollision, true, kEnvironmentCollision},

      {kEnvironmentCollision, false, kNoCollision},
      {kEnvironmentCollision, true, kEnvironmentCollision},

      {kSelfCollision, false, kSelfCollision},
      {kSelfCollision, true, kEnvironmentAndSelfCollision},

      {kEnvironmentAndSelfCollision, false, kSelfCollision},
      {kEnvironmentAndSelfCollision, true, kEnvironmentAndSelfCollision},
  };
  for (const auto& a_case : cases) {
    SCOPED_TRACE(a_case.to_string());
    EXPECT_EQ(SetInEnvironmentCollision(a_case.input, a_case.in_collision),
              a_case.output);
  }
}

GTEST_TEST(CollisionCheckerTest, SelfCollisionBits) {
  Case cases[] = {
      {kNoCollision, false, kNoCollision},
      {kNoCollision, true, kSelfCollision},

      {kEnvironmentCollision, false, kEnvironmentCollision},
      {kEnvironmentCollision, true, kEnvironmentAndSelfCollision},

      {kSelfCollision, false, kNoCollision},
      {kSelfCollision, true, kSelfCollision},

      {kEnvironmentAndSelfCollision, false, kEnvironmentCollision},
      {kEnvironmentAndSelfCollision, true, kEnvironmentAndSelfCollision},
  };
  for (const auto& a_case : cases) {
    SCOPED_TRACE(a_case.to_string());
    EXPECT_EQ(SetInSelfCollision(a_case.input, a_case.in_collision),
              a_case.output);
  }
}

}  // namespace
}  // namespace planning
}  // namespace drake
