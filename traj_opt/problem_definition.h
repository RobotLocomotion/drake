#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace traj_opt {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * A struct for specifying the optimization problem
 *
 *    min x_err(T)'*Qf*x_err(T) + sum{ x_err(t)'*Q*x_err(t) + u(t)'*R*u(t) }
 *    s.t. x(0) = x0
 *         multibody dynamics with contact
 *
 *  where x(t) = [q(t); v(t)], x_err(t) = x(t)-x_nom(t), and
 *  Q = [ Qq  0  ]
 *      [ 0   Qv ].
 */
struct ProblemDefinition {
  // Time horizon (number of steps) for the optimization problem
  int num_steps;

  // Initial generalized positions
  VectorXd q_init;

  // Initial generalized velocities
  VectorXd v_init;
};

}  // namespace traj_opt
}  // namespace drake
