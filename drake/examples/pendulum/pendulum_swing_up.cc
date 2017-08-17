#include "drake/examples/pendulum/pendulum_swing_up.h"

#include <cmath>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/pendulum/gen/pendulum_state_vector.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/solvers/function.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::solvers::LinearEqualityConstraint;

namespace drake {
namespace examples {
namespace pendulum {

void AddSwingUpTrajectoryParams(const Eigen::Vector2d& x0,
                                const Eigen::Vector2d& xG,
                                systems::DircolTrajectoryOptimization* dircol) {
  auto u = dircol->input();

  const double kTorqueLimit = 3.0;  // N*m.
  dircol->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  dircol->AddLinearConstraint(dircol->initial_state() == x0);
  dircol->AddLinearConstraint(dircol->final_state() == xG);

  const double R = 10;  // Cost on input "effort".
  dircol->AddRunningCost((R * u) * u);
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
