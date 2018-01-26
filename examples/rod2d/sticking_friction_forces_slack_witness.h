#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod2d_witness_function.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// A witness function that is used to determine when the forces used to
/// enforce non-sliding contact move outside of the friction cone.
template <class T>
class StickingFrictionForcesSlackWitness : public Rod2dWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StickingFrictionForcesSlackWitness)

  StickingFrictionForcesSlackWitness(const Rod2D<T>* rod, int contact_index) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    std::ostringstream oss;
    oss << "StickingFrictionForcesSlack (" << contact_index << ")";
    this->set_name(oss.str());
    solver_ = &rod->solver_;
  }

  typename Rod2dWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return Rod2dWitnessFunction<T>::WitnessType::kStickingFrictionForceSlack;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // TODO(edrumwri): Flesh out this stub once PointContact class has been
    // added.
    DRAKE_ABORT();
    return 0;
  }

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

