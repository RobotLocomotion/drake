#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System where the state at (scalar) time t corresponds to the cubic equation
/// t³ + t² + 12t + 5.
class CubicScalarSystem : public LeafSystem<double> {
 public:
  CubicScalarSystem() { this->DeclareContinuousState(1); }

  /// Evaluates the system at time t.
  double Evaluate(double t) const {
    return 5 + t * (t * (t + 1) + 12);
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
    (*deriv)[0] = 3 * t * t + 2 * t + 12;
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
