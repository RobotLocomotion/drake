#pragma once

#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace franka_panda {

const int kPandaArmNumJoints = 7;

/** Type alias for Panda control mode bitfields. Control modes can be bitwise
 * OR'd together to enable multiple control modes simultaneously.
 * Values match lcmt_panda_status::CONTROL_MODE_* constants. */
using PandaControlMode = uint64_t;

/** Control mode constants for the Panda robot. */
namespace PandaControlModes {
constexpr PandaControlMode kNone = 0;
constexpr PandaControlMode kPosition = 1;
constexpr PandaControlMode kVelocity = 2;
constexpr PandaControlMode kTorque = 4;
}  // namespace PandaControlModes

// Helper to convert to int for use in comparisons and LCM messages
constexpr int to_int(PandaControlMode mode) {
  return static_cast<int>(mode);
}

// Ensure our constants match the LCM message definition
static_assert(to_int(PandaControlModes::kPosition) ==
                  drake::lcmt_panda_status::CONTROL_MODE_POSITION,
              "PandaControlModes::kPosition must match LCM definition");
static_assert(to_int(PandaControlModes::kVelocity) ==
                  drake::lcmt_panda_status::CONTROL_MODE_VELOCITY,
              "PandaControlModes::kVelocity must match LCM definition");
static_assert(to_int(PandaControlModes::kTorque) ==
                  drake::lcmt_panda_status::CONTROL_MODE_TORQUE,
              "PandaControlModes::kTorque must match LCM definition");

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
