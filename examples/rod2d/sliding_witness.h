#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

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
class SlidingWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingWitness)

  SlidingWitness(
      const Rod2D<T>* rod,
      int contact_index,
      bool pos_direction,
      double sliding_velocity_threshold) :
      RodWitnessFunction<T>(
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

  typename RodWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return RodWitnessFunction<T>::WitnessType::kSlidingWitness;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Get the rod system.
    const Rod2D<T>& rod = this->get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const int contact_index = this->get_contact_index();
    const auto& contact = rod.get_contacts_used_in_force_calculations(
        context.get_state())[contact_index];

    // Verify rod is undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.sliding_type ==
        multibody::constraint::SlidingModeType::kSliding || 
        contact.sliding_type ==
        multibody::constraint::SlidingModeType::kTransitioning);

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = rod.CalcContactVelocity(context,
                                                    contact_index);

    // Return the tangent velocity.
    if (positive_) {
      return velocity_threshold_ - pdot[0];
    } else {
      return -pdot[0] - velocity_threshold_;
    }
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

