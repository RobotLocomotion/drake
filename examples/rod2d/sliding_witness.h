#pragma once

#include <limits>
#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod2d_witness_function.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Determines whether the tangent velocity crosses the sliding velocity
/// threshold. This witness is used for two purposes: (1) it determines when
/// a contact has moved from non-sliding-to-sliding-transition to proper sliding
/// and (2) it determines when a contact has moved from sliding to non-sliding.
template <class T>
class SlidingWitness : public Rod2dWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingWitness)

  /// Constructs the sliding witness function to track the specified rod
  /// endpoint for checking sliding velocities along the positive x-axis
  /// (`pos_direction = true`) or along the negative x-axis
  /// (`pos_direction = false`).
  SlidingWitness(
      const Rod2D<T>* rod,
      RodEndpoint endpoint,
      bool pos_direction) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          endpoint) {
    std::ostringstream oss;
    oss << "Sliding ";
    if (pos_direction) {
      oss << "+";
    } else {
      oss << "-";
    }
    oss << " (" << endpoint << ")";
    this->set_name(oss.str());
    positive_ = pos_direction;
  }

 private:
  T DoCalcWitnessValue(const systems::Context<T>&) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // introduced.
    DRAKE_ABORT();
    return 0;
  }

  // If 'true', witness function triggers when the sliding velocity has
  // sufficient magnitude along the positive x-axis. Otherwise, it triggers when
  // the sliding velocity has sufficient magnitude along the negative x-axis.
  bool positive_{false};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

