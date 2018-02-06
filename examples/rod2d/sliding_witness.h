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

  SlidingWitness(
      const Rod2D<T>& rod,
      int contact_index,
      bool pos_direction,
      double sliding_velocity_threshold) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          contact_index) {
    std::ostringstream oss;
    oss << "Sliding ";
    if (pos_direction) {
      oss << "+";
    } else {
      oss << "-";
    }
    oss << " (" << contact_index << ")";
    this->set_name(oss.str());
    positive_ = pos_direction;
    velocity_threshold_ = sliding_velocity_threshold;
  }

  typename Rod2dWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return Rod2dWitnessFunction<T>::WitnessType::kSlidingWitness;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // introduced.
    DRAKE_ABORT();
    return 0;
  }

  // If 'true', witness function triggers when the sliding velocity is
  // sufficiently positive. Otherwise, it triggers when the sliding velocity
  // is sufficiently negative.
  bool positive_{false};

  // The contact is only to be considered as properly sliding once this
  // threshold has been met.
  double velocity_threshold_{10 * std::numeric_limits<double>::epsilon()};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

