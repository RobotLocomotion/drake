/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf.
 */

#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

constexpr int kSchunkWsgNumActuators = 2;
constexpr int kSchunkWsgNumPositions = 2;
constexpr int kSchunkWsgNumVelocities = 2;

constexpr int kSchunkWsgPositionIndex = 0;
constexpr int kSchunkWsgVelocityIndex =
    kSchunkWsgNumPositions + kSchunkWsgPositionIndex;

// TODO(sammy): need to double check this.
constexpr double kSchunkWsgLcmStatusPeriod = 0.05;

/**
 * Returns the position vector corresponding to the open position of the
 * gripper.
 */
template <typename T>
VectorX<T> GetSchunkWsgOpenPosition() {
  // clang-format off
  return (VectorX<T>(kSchunkWsgNumPositions) <<
      -0.0550667,
       0.0550667).finished();
  // clang-format on
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
