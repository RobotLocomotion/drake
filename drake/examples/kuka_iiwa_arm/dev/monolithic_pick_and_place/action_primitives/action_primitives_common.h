#pragma once

#include <vector>
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

// Different states for the pick and place task.
typedef enum ActionPrimitiveState {
  // Primitive is currently under execution.
  RUNNING,
  // Primitive is waiting for new set of inputs (&/or last action done).
  WAITING,
  // Primitive has been aborted due to some kind of failure.
  ABORTED
} ActionPrimitiveState;

struct IiwaActionInput {
  IiwaActionInput() {}

  bool is_valid{false};
  std::vector<double> time;
  std::vector<VectorX<double>> q;
};

typedef enum GripperActionInput { UNDEFINED, CLOSE, OPEN } GripperActionInput;

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
