#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace traj_opt {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * A struct for specifying the optimization problem
 *
 *    min x_err(T)'*Qf*x_err(T) + dt*sum{ x_err(t)'*Q*x_err(t) + u(t)'*R*u(t) }
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

  // Running cost coefficients for generalized positions
  // N.B. these weights are per unit of time
  // TODO(vincekurtz): consider storing these as VectorXd, assuming they are
  // diagonal, and using X.asDiagonal() when multiplying.
  MatrixXd Qq;

  // Running cost coefficients for generalized velocities
  // N.B. these weights are per unit of time
  MatrixXd Qv;

  // Terminal cost coefficients for generalized positions
  MatrixXd Qf_q;

  // Terminal cost coefficients for generalized velocities
  MatrixXd Qf_v;

  // Control cost coefficients
  // N.B. these weights are per unit of time
  MatrixXd R;

  // Target generalized positions
  VectorXd q_nom;

  // Target generalized velocities
  VectorXd v_nom;
};

}  // namespace traj_opt
}  // namespace drake
