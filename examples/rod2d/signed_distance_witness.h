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

  SignedDistanceWitness(const Rod2D<T>* rod, RodEndpoint endpoint) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          endpoint) {
    std::ostringstream oss;
    oss << "SignedDistance (" << endpoint << ")";
    this->set_name(oss.str());
  }

 private:
  T DoCalcWitnessValue(const systems::Context<T>&) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // introduced.
    DRAKE_ABORT();
    return 0;
  }
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

