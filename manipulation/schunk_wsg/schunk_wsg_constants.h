/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf.
 */

#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

constexpr int kSchunkWsgNumActuators = 2;
constexpr int kSchunkWsgNumPositions = 2;
constexpr int kSchunkWsgNumVelocities = 2;

// TODO(russt): These constants are predicated on the idea that we can map
// one finger's state => gripper state.  We should prefer using
// MakeMultibodyStateToWsgState and MakeMultibodyForceToWsgForce instead.
// But I cannot deprecate them yet without performing surgery on the old
// controller.
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

/// Extract the distance between the fingers (and its time derivative) out
/// of the plant model which pretends the two fingers are independent.
template <typename T>
std::unique_ptr<systems::MatrixGain<T>>
MakeMultibodyStateToWsgStateSystem() {
  Eigen::Matrix<double, 2, 4> D;
  // clang-format off
  D << 1, -1, 0,  0,
      0,  0, 1, -1;
  // clang-format on
  return std::make_unique<systems::MatrixGain<T>>(D);
}

/// Extract the gripper measured force from the generalized forces on the two
/// fingers.
template <typename T>
std::unique_ptr<systems::MatrixGain<T>>
MakeMultibodyForceToWsgForceSystem() {
  // gripper force = -finger0 + finger1.
  return std::make_unique<systems::MatrixGain<T>>(Eigen::RowVector2d(-1, 1));
}


}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
