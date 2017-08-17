// This file contains utility functions for swing-up trajectory
// optimization. It is based on pendulum_swing_up.cc.

#include "drake/examples/acrobot/acrobot_swing_up.h"

#include <cmath>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state_vector.h"
#include "drake/solvers/function.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::solvers::LinearEqualityConstraint;

namespace drake {
namespace examples {
namespace acrobot {

void AddSwingUpTrajectoryParams(
    const Eigen::Vector4d& x0, const Eigen::Vector4d& xG,
    systems::DircolTrajectoryOptimization* dircol) {

  // Current limit for MIT's acrobot is 7-9 Amps, according to Michael Posa.
  const double kTorqueLimit = 8;

  auto u = dircol->input();
  dircol->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  dircol->AddLinearConstraint(dircol->initial_state() == x0);
  dircol->AddLinearConstraint(dircol->final_state() == xG);

  const double R = 10;  // Cost on input "effort".
  dircol->AddRunningCost((R * u) * u);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
