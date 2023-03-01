#pragma once

#include <cstdint>

namespace drake {
namespace planning {

/** Enumerates these predicates (and their combinations):
 - is the robot in collision with itself?
 - is the robot in collision with something in the environment?

 @ingroup planning_collision_checker */
enum class RobotCollisionType : uint8_t {
  kNoCollision = 0x00,
  kEnvironmentCollision = 0x01,
  kSelfCollision = 0x02,
  kEnvironmentAndSelfCollision = kEnvironmentCollision | kSelfCollision
};

/** @returns a RobotCollisionType where the environment-collision value is that
 asserted by `in_environment_collision`, and the self-collision value is that
 asserted by `collision_type`. */
inline RobotCollisionType SetInEnvironmentCollision(
    RobotCollisionType collision_type, bool in_environment_collision) {
  constexpr uint8_t mask =
      static_cast<uint8_t>(RobotCollisionType::kEnvironmentCollision);
  return static_cast<RobotCollisionType>(
      (static_cast<uint8_t>(collision_type) & ~mask) |
      (in_environment_collision * mask));
}

/** @returns a RobotCollisionType where the self-collision value is that
 asserted by `in_self_collision`, and the environment-collision value is that
 asserted by `collision_type`. */
inline RobotCollisionType SetInSelfCollision(RobotCollisionType collision_type,
                                             bool in_self_collision) {
  constexpr uint8_t mask =
      static_cast<uint8_t>(RobotCollisionType::kSelfCollision);
  return static_cast<RobotCollisionType>(
      (static_cast<uint8_t>(collision_type) & ~mask) |
      (in_self_collision * mask));
}

}  // namespace planning
}  // namespace drake
