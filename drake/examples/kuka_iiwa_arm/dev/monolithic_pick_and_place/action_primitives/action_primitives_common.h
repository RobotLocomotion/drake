#pragma once

#include <vector>
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// Different states for the pick and place task.
typedef enum ActionPrimitiveState {
  // Primitive is currently under execution.
  RUNNING,
  // Primitive is waiting for new set of inputs (&/or last action done).
  WAITING,
  // Primitive has been aborted due to some kind of failure.
  ABORTED
} ActionPrimitiveState;
// TODO(naveenoid) : This needs to be expanded to become the base class for
// all of the outputs that ActionPrimitives send.

/// A struct that contains all the variables needed for the abstract input
/// of the `ActionPrimitive` of type `IiwaMove`.
struct IiwaActionInput {
  IiwaActionInput() {}

  /// A flag used to define if this instance of the `IiwaActionInput` is
  /// valid.
  bool is_valid{false};

  /// `time` and `q` are used to generate the joint space spline for
  /// interpolation.
  std::vector<double> time;
  std::vector<VectorX<double>> q;
};

/// An enumeration that defined the abstract input of the of the
/// `ActionPrimitive` of type `GripperAction`.
typedef enum GripperActionInput { UNDEFINED, CLOSE, OPEN } GripperActionInput;

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
