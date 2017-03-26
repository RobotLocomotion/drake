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

void AddSwingUpTrajectoryParams(
    const Eigen::Vector2d& x0, const Eigen::Vector2d& xG,
    systems::DircolTrajectoryOptimization* dircol) {

  const int kTorqueLimit = 3;  // Arbitrary, taken from PendulumPlant.m.
  const drake::Vector1d umin(-kTorqueLimit);
  const drake::Vector1d umax(kTorqueLimit);
  dircol->AddInputBounds(umin, umax);

  // TODO: Simplify to e.g. state(0) == x0 once the required overloads arrive.
  dircol->AddLinearConstraint( dircol->initial_state().array() == x0.array() );
  dircol->AddLinearConstraint( dircol->final_state().array() == xG.array() );

  const double R = 10;  // Cost on input "effort".
  auto u = dircol->input();
  dircol->AddRunningCost( (R * u) * u );
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
