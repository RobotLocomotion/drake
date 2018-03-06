#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod2d_witness_function.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Witness function for determining whether a point of contact is moving.
/// away from the half-space.
template <class T>
class NormalVelWitness : public Rod2dWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NormalVelWitness)

  NormalVelWitness(const Rod2D<T>* rod, RodEndpoint endpoint) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          endpoint) {
    std::ostringstream oss;
    oss << "NormalVel (" << endpoint << ")";
    this->set_name(oss.str());
    solver_ = &rod->solver_;
  }

 private:
  T DoCalcWitnessValue(const systems::Context<T>&) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // introduced.
    DRAKE_ABORT();
    return 0;
  }

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

