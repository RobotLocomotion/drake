#pragma once

#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace franka_panda {

const int kPandaArmNumJoints = 7;

/** Control modes for the Panda robot. These can be bitwise OR'd together
 * to enable multiple control modes simultaneously.
 * Values match lcmt_panda_status::CONTROL_MODE_* constants. */
namespace PandaControlMode {
constexpr int kPosition = 1;
constexpr int kVelocity = 2;
constexpr int kTorque = 4;
}  // namespace PandaControlMode

// Ensure our constants match the LCM message definition
static_assert(PandaControlMode::kPosition ==
                  drake::lcmt_panda_status::CONTROL_MODE_POSITION,
              "PandaControlMode::kPosition must match LCM definition");
static_assert(PandaControlMode::kVelocity ==
                  drake::lcmt_panda_status::CONTROL_MODE_VELOCITY,
              "PandaControlMode::kVelocity must match LCM definition");
static_assert(PandaControlMode::kTorque ==
                  drake::lcmt_panda_status::CONTROL_MODE_TORQUE,
              "PandaControlMode::kTorque must match LCM definition");

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
