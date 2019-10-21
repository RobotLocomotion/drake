#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System where the state at (scalar) time t corresponds to the quintic
/// equation t⁵ + 2t⁴ + 3t³ + 4t² + 5t + 6.
class QuinticScalarSystem : public LeafSystem<double> {
 public:
  QuinticScalarSystem() { this->DeclareContinuousState(1); }

  /// Evaluates the system at time t.
  double Evaluate(double t) const {
    return t * (t * (t * (t * (t + 2) + 3) + 4) + 5) + 6;
  }

 private:
  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const final {
    const double t0 = 0.0;
    state->get_mutable_continuous_state().get_mutable_vector()[0] =
        Evaluate(t0);
  }

  void DoCalcTimeDerivatives(const Context<double>& context,
                             ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = t * (t * (t * (5*t + 8) + 9) + 8) + 5;
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
