#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod2d_witness_function.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Computes the signed distance between a point of contact and the half-space
/// representing the ground.
template <class T>
class SignedDistanceWitness : public Rod2dWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignedDistanceWitness)

  SignedDistanceWitness(const Rod2D<T>* rod, int contact_index) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          contact_index) {
    std::ostringstream oss;
    oss << "SignedDistance (" << contact_index << ")";
    this->set_name(oss.str());
  }

  typename Rod2dWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return Rod2dWitnessFunction<T>::WitnessType::kSignedDistance;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // introduced.
    DRAKE_ABORT();
    return 0;
  }
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

