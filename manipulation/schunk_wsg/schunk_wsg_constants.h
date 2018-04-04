/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf. Although the gripper only has one actuator, the
 * model has a number of constrained linkages to mimic two fingers moving
 * in a coordinated manner. This results in more states than actuators, and
 * a need for a selection matrix for state feedback control.
 */

#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

constexpr int kSchunkWsgNumActuators = 1;
constexpr int kSchunkWsgNumPositions = 5;
constexpr int kSchunkWsgNumVelocities = 5;

constexpr int kSchunkWsgPositionIndex = 0;
constexpr int kSchunkWsgVelocityIndex =
    kSchunkWsgNumPositions + kSchunkWsgPositionIndex;

// TODO(sammy): need to double check this.
constexpr double kSchunkWsgLcmStatusPeriod = 0.05;

/**
 * Returns the selection matrix that selects the position and velocity indices
 * that correspond to the `left_finger_sliding_joint` in the model. The
 * returned matrix projects the full state to two dimensions (position and
 * velocity).
 */
template <typename T>
MatrixX<T> GetSchunkWsgFeedbackSelector() {
  MatrixX<T> selector =
      MatrixX<T>::Zero(2 * kSchunkWsgNumActuators,
                       kSchunkWsgNumPositions + kSchunkWsgNumVelocities);
  selector(0, kSchunkWsgPositionIndex) = 1;
  selector(1, kSchunkWsgVelocityIndex) = 1;
  return selector;
}

/**
 * Returns the position vector corresponding to the open position of the
 * gripper. This is more complicated than one might expect due to the linkage in
 * our model of the gripper.
 */
template <typename T>
VectorX<T> GetSchunkWsgOpenPosition() {
  // clang-format off
  return (VectorX<T>(kSchunkWsgNumPositions) <<
      -0.0550667,
       0.009759,
       1.27982,
       0.0550667,
       0.009759) .finished();
  // clang-format on
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
