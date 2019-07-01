#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System where the state at (scalar) time t corresponds to the quadratic
///  equation St² + St + 3, where S is a user-defined Scalar (4 by default).
class QuadraticScalarSystem : public LeafSystem<double> {
 public:
  explicit QuadraticScalarSystem(double S = 4) : S_(S) {
    this->DeclareContinuousState(1);
  }

  /// Evaluates the system at time t.
  double Evaluate(double t) const {
    return 3 + S_ * t * (t + 1);
  }

 private:
  void SetDefaultState(
      const Context<double>& context, State<double>* state) const final {
    const double t0 = 0.0;
    state->get_mutable_continuous_state().get_mutable_vector()[0] =
        Evaluate(t0);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = S_ * (2 * t + 1);
  }

  // The scaling factor.
  double S_{};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
