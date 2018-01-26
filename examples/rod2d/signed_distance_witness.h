#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Computes the signed distance between a point of contact and the half-space
/// representing the ground.
template <class T>
class SignedDistanceWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignedDistanceWitness)

  SignedDistanceWitness(const Rod2D<T>* rod, int contact_index) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          contact_index) {
    std::ostringstream oss;
    oss << "SignedDistance (" << contact_index << ")";
    this->set_name(oss.str());
  }

  typename RodWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return RodWitnessFunction<T>::WitnessType::kSignedDistance;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Get the rod system.
    const Rod2D<T>& rod = this->get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact index. 
    const int contact_index = this->get_contact_index();

    // Get the relevant parts of the state.
    const Vector3<T> q = context.get_continuous_state().
        get_generalized_position().CopyToVector();

    // Get the contact candidate in the world frame.
    const Vector2<T>& u = rod.get_contact_candidate(contact_index);
    const Vector2<T> p = rod.GetPointInWorldFrame(q, u);

    // Return the vertical location.
    return p[1];
  }
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

