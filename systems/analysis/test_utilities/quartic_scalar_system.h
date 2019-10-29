#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System where the state at (scalar) time t corresponds to the quartic
/// equation t⁴ + 2t³ + 3t² + 4t + 5.
class QuarticScalarSystem : public LeafSystem<double> {
 public:
  QuarticScalarSystem() { this->DeclareContinuousState(1); }

  /// Evaluates the system at time t.
  double Evaluate(double t) const {
    return t * (t * (t * (t + 2) + 3) + 4) + 5;
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
    (*deriv)[0] = t * (t * (4*t + 6) + 6) + 4;
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
