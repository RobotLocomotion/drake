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

/// Witness function using the acceleration of a point away from the halfspace. 
template <class T>
class NormalAccelWitness : public Rod2dWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NormalAccelWitness)

  NormalAccelWitness(const Rod2D<T>* rod, int contact_index) :
      Rod2dWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    std::ostringstream oss;
    oss << "NormalAccel (" << contact_index << ")";
    this->set_name(oss.str());
    solver_ = &rod->solver_;
  }

  typename Rod2dWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return Rod2dWitnessFunction<T>::WitnessType::kNormalAccel;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
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

