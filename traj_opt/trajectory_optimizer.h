#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/problem_definition.h"

namespace drake {
namespace traj_opt {

using Eigen::VectorXd;
using multibody::MultibodyPlant;
using systems::Context;

class TrajectoryOptimizer {
 public:
  /**
   * Construct a new Trajectory Optimizer object.
   *
   * @param plant A model of the system that we're trying to find an optimal
   *              trajectory for.
   * @param prob Problem definition, including cost, initial and target states,
   *             etc.
   */
  TrajectoryOptimizer(const MultibodyPlant<double>* plant,
                      const ProblemDefinition& prob);

  /**
   * Convienience function to get the timestep of this optimization problem.
   *
   * @return double dt, the time step for this optimization problem
   */
  double time_step() const { return plant_->time_step(); }

  /**
   * Convienience function to get the time horizon (T) of this optimization problem.
   *
   * @return int the number of time steps in the optimal trajectory.
   */
  int num_steps() const { return prob_.num_steps; }

  /**
   * Compute a sequence of generalized velocities v from a sequence of
   * generalized positions, where
   *
   *     v_t = (q_t - q_{t-1})/dt            (1)
   *
   * v and q are each vectors of length num_steps+1,
   *
   *     v = [v(0), v(1), v(2), ..., v(num_steps)],
   *     q = [q(0), q(1), q(2), ..., q(num_steps)].
   *
   * Note that v0 = v_init is defined by the initial state of the optimization
   * problem, rather than Equation (1) above.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities
   */
  void CalcV(const std::vector<VectorXd>& q, std::vector<VectorXd>* v) const;

 private:
  // A model of the system that we are trying to find an optimal trajectory for.
  const MultibodyPlant<double>* plant_;

  // A context corresponding to plant_, to enable dynamics computations.
  std::unique_ptr<Context<double>> context_;

  // Stores the problem definition, including cost, time horizon, initial state,
  // target state, etc.
  const ProblemDefinition prob_;
};

}  // namespace traj_opt
}  // namespace drake
