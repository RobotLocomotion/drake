#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace test {

/// Robertson's stiff chemical reaction problem. This example is taken from
/// [Hairer, 1996] and is described in more detail in:
/// http://www.radford.edu/~thompson/vodef90web/problems/demosnodislin/Single/DemoRobertson/demorobertson.pdf
/// The original system is described in:
///
/// - [Robertson, 1966]  H. H. Robertson. "The solution of a system of reaction
///                      rate equations" in Numerical Analysis, An Introduction.
///                      Pages 178-182. Academic Press, 1966.
template <class T>
class RobertsonSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobertsonSystem)
  RobertsonSystem() {
    this->DeclareContinuousState(3);
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* deriv) const override {
    // Get state.
    const T& y1 = context.get_continuous_state_vector().GetAtIndex(0);
    const T& y2 = context.get_continuous_state_vector().GetAtIndex(1);
    const T& y3 = context.get_continuous_state_vector().GetAtIndex(2);

    // Compute derivatives.
    T y1_prime = -0.04 * y1 + 1e4 * y2 * y3;
    T y2_prime = 0.04 * y1 - 1e4 * y2 * y3 - 3e7 * y2 * y2;
    T y3_prime = 3e7 * y2 * y2;

    // Set the derivatives.
    deriv->get_mutable_vector().SetAtIndex(0, y1_prime);
    deriv->get_mutable_vector().SetAtIndex(1, y2_prime);
    deriv->get_mutable_vector().SetAtIndex(2, y3_prime);
  }

  /// Sets the initial conditions for the Robertson system.
  void SetDefaultState(
      const Context<T>& context, State<T>* state) const override {
    auto& xc = state->get_mutable_continuous_state().get_mutable_vector();
    xc.SetAtIndex(0, 1);
    xc.SetAtIndex(1, 0);
    xc.SetAtIndex(2, 0);
  }

  /// Gets the end time for integration.
  T get_end_time() const { return 1e11; }

  /// Gets the system solution. Only works for time 10^11.
  static Vector3<T> GetSolution(const T& t) {
    DRAKE_DEMAND(t == 1e11);
    Vector3<T> sol;
    sol(0) = 0.208334014970122e-7;
    sol(1) = 0.8333360770334713e-13;
    sol(2) = 0.9999999791665050;
    return sol;
  }
};

}  // namespace test
}  // namespace analysis
}  // namespace systems
}  // namespace drake
