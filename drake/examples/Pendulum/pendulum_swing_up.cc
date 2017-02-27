#include "drake/examples/Pendulum/pendulum_swing_up.h"

#include <cmath>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/Pendulum/gen/pendulum_state_vector.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/solvers/function.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::solvers::LinearEqualityConstraint;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

/**
 * Define a function to be evaluated as the running cost for a
 * pendulum trajectory (using the solvers::FunctionTraits style
 * interface).
 */
class PendulumRunningCost {
 public:
  static size_t numInputs() {
    return PendulumStateVectorIndices::kNumCoordinates + 2;
  }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(const VecIn<ScalarType>& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());

    // u represents the input vector.  Convention preserved here from
    // PendulumPlant.m.
    const auto u = x.tail(1);
    const double R = 10;  // From PendulumPlant.m, arbitrary AFAICT
    y = (R * u) * u;
  }
};

}  // anon namespace

void AddSwingUpTrajectoryParams(
    const Eigen::Vector2d& x0, const Eigen::Vector2d& xG,
    systems::DircolTrajectoryOptimization* dircol) {

  const int kTorqueLimit = 3;  // Arbitrary, taken from PendulumPlant.m.
  const drake::Vector1d umin(-kTorqueLimit);
  const drake::Vector1d umax(kTorqueLimit);
  dircol->AddInputBounds(umin, umax);

  // TODO: Simplify to e.g. state(0) == x0 once symbolic support arrives.
  dircol->AddLinearConstraint( dircol->initial_state().array() == x0.array() );
  dircol->AddLinearConstraint( dircol->final_state().array() == xG.array() );

  dircol->AddRunningCostFunc(PendulumRunningCost());
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
