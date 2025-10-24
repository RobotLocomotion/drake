#pragma once

#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace franka_panda {

const int kPandaArmNumJoints = 7;

/** Control modes for the Panda robot. These can be bitwise OR'd together
 * to enable multiple control modes simultaneously.
 * Values match lcmt_panda_status::CONTROL_MODE_* constants. */
enum class PandaControlMode : int {
  kNone = 0,
  kPosition = 1,
  kVelocity = 2,
  kTorque = 4,
};

// Enable bitwise operations for PandaControlMode
constexpr PandaControlMode operator|(PandaControlMode a, PandaControlMode b) {
  return static_cast<PandaControlMode>(static_cast<int>(a) |
                                       static_cast<int>(b));
}

constexpr PandaControlMode operator&(PandaControlMode a, PandaControlMode b) {
  return static_cast<PandaControlMode>(static_cast<int>(a) &
                                       static_cast<int>(b));
}

constexpr PandaControlMode operator~(PandaControlMode a) {
  return static_cast<PandaControlMode>(~static_cast<int>(a));
}

constexpr PandaControlMode& operator|=(PandaControlMode& a,
                                       PandaControlMode b) {
  return a = a | b;
}

constexpr PandaControlMode& operator&=(PandaControlMode& a,
                                       PandaControlMode b) {
  return a = a & b;
}

// Helper to convert to int for use in comparisons and LCM messages
constexpr int to_int(PandaControlMode mode) {
  return static_cast<int>(mode);
}

// Ensure our constants match the LCM message definition
static_assert(to_int(PandaControlMode::kPosition) ==
                  drake::lcmt_panda_status::CONTROL_MODE_POSITION,
              "PandaControlMode::kPosition must match LCM definition");
static_assert(to_int(PandaControlMode::kVelocity) ==
                  drake::lcmt_panda_status::CONTROL_MODE_VELOCITY,
              "PandaControlMode::kVelocity must match LCM definition");
static_assert(to_int(PandaControlMode::kTorque) ==
                  drake::lcmt_panda_status::CONTROL_MODE_TORQUE,
              "PandaControlMode::kTorque must match LCM definition");

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
