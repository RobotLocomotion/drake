#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/inverse_dynamics_partials.h"
#include "drake/traj_opt/problem_definition.h"
#include "drake/traj_opt/trajectory_optimizer_state.h"
#include "drake/traj_opt/trajectory_optimizer_workspace.h"
#include "drake/traj_opt/velocity_partials.h"

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
   * Convienience function to get the time horizon (T) of this optimization
   * problem.
   *
   * @return int the number of time steps in the optimal trajectory.
   */
  int num_steps() const { return prob_.num_steps; }

  /**
   * Convienience function to get a const reference to the multibody plant that
   * we are optimizing over.
   *
   * @return const MultibodyPlant<double>&, the plant we're optimizing over.
   */
  const MultibodyPlant<double>& plant() const { return *plant_; }

  /**
   * Create a state object which contains the decision variables (generalized
   * positions at each timestep), along with a cache of other things that are
   * computed from positions, such as velocities, accelerations, forces, and
   * various derivatives.
   *
   * @return TrajectoryOptimizerState
   */
  TrajectoryOptimizerState CreateState() const {
    return TrajectoryOptimizerState(num_steps(), plant());
  }

  /**
   * Compute and return the total (unconstrained) cost of the optimization
   * problem,
   *
   *     L(q) = x_err(T)'*Qf*x_err(T)
   *                + dt*sum_{t=0}^{T-1} x_err(t)'*Q*x_err(t) + u(t)'*R*u(t),
   *
   * where:
   *      x_err(t) = x(t) - x_nom is the state error,
   *      T = num_steps() is the time horizon of the optimization problem,
   *      x(t) = [q(t); v(t)] is the system state at time t,
   *      u(t) are control inputs, and we assume (for now) that u(t) = tau(t),
   *      Q{f} = diag([Qq{f}, Qv{f}]) are a block diagonal PSD state-error
   *       weighting matrices,
   *      R is a PSD control weighting matrix.
   *
   * @param state optimizer state, including q, v, and tau
   * @return double, total cost
   */
  double CalcCost(const TrajectoryOptimizerState& state) const;

  /**
   * Compute the gradient of the unconstrained cost L(q).
   *
   * @param state optimizer state, including q, v, tau, gradients, etc.
   * @param g a single VectorXd containing the partials of L w.r.t. each
   *          decision variable (q_t[i]).
   */
  void CalcGradient(const TrajectoryOptimizerState& state,
                    EigenPtr<VectorXd> g) const;

 private:
  // Friend class to facilitate testing.
  friend class TrajectoryOptimizerTester;

  /**
   * Compute everything in the state's cache (v, a, tau, dv_dq, dtau_dq)
   * to correspond to the state's generalized positions q.
   *
   * @param state optimizer state to update.
   */
  void UpdateCache(const TrajectoryOptimizerState& state) const;

  /**
   * Compute the total cost of the unconstrained problem.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities (consistent with q)
   * @param tau sequence of generalized forces (consistent with q and v)
   * @param workspace scratch space for intermediate computations
   * @return double, total cost
   */
  double CalcCost(const std::vector<VectorXd>& q,
                  const std::vector<VectorXd>& v,
                  const std::vector<VectorXd>& tau,
                  TrajectoryOptimizerWorkspace* workspace) const;

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
  void CalcVelocities(const std::vector<VectorXd>& q,
                      std::vector<VectorXd>* v) const;

  /**
   * Compute a sequence of generalized accelerations a from a sequence of
   * generalized velocities,
   *
   *    a_t = (v_{t+1} - v_{t})/dt,
   *
   * where v is of length (num_steps+1) and a is of length num_steps:
   *
   *     v = [v(0), v(1), v(2), ..., v(num_steps)],
   *     a = [a(0), a(1), a(2), ..., a(num_steps-1)].
   *
   * @param v sequence of generalized velocities
   * @param a sequence of generalized accelerations
   */
  void CalcAccelerations(const std::vector<VectorXd>& v,
                         std::vector<VectorXd>* a) const;

  /**
   * Compute a sequence of generalized forces t from sequences of generalized
   * accelerations, velocities, and positions, where generalized forces are
   * defined by the inverse dynamics,
   *
   *    tau_t = M*(v_{t+1}-v_t})/dt + D*v_{t+1} - k(q_t,v_t)
   *                               - (1/dt) *J'*gamma(v_{t+1},q_t).
   *
   * Note that q and v have length num_steps+1,
   *
   *  q = [q(0), q(1), ..., q(num_steps)],
   *  v = [v(0), v(1), ..., v(num_steps)],
   *
   * while a and tau have length num_steps,
   *
   *  a = [a(0), a(1), ..., a(num_steps-1)],
   *  tau = [tau(0), tau(1), ..., tau(num_steps-1)],
   *
   * i.e., tau(t) takes us us from t to t+1.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities
   * @param a sequence of generalized accelerations
   * @param workspace scratch space for intermediate computations
   * @param tau sequence of generalized forces
   */
  void CalcInverseDynamics(const std::vector<VectorXd>& q,
                           const std::vector<VectorXd>& v,
                           const std::vector<VectorXd>& a,
                           TrajectoryOptimizerWorkspace* workspace,
                           std::vector<VectorXd>* tau) const;

  /**
   * Helper function for computing the inverse dynamics
   *
   *  tau = ID(a, v, q, f_ext)
   *
   * at a single timestep.
   *
   * @param q generalized position
   * @param v generalized velocity
   * @param a generalized acceleration
   * @param workspace scratch space for intermediate computations
   * @param tau generalized forces
   */
  void CalcInverseDynamicsSingleTimeStep(
      const VectorXd& q, const VectorXd& v, const VectorXd& a,
      TrajectoryOptimizerWorkspace* workspace, VectorXd* tau) const;

  /**
   * Compute partial derivatives of the inverse dynamics
   *
   *    tau_t = ID(q_{t-1}, q_t, q_{t+1})
   *
   * and store them in the given InverseDynamicsPartials struct.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities (computed from q)
   * @param a sequence of generalized accelerations (computed from q)
   * @param tau sequence of generalized forces (computed from q)
   * @param workspace scratch space for intermediate computations
   * @param id_partials struct for holding dtau/dq
   */
  void CalcInverseDynamicsPartials(const std::vector<VectorXd>& q,
                                   const std::vector<VectorXd>& v,
                                   const std::vector<VectorXd>& a,
                                   const std::vector<VectorXd>& tau,
                                   TrajectoryOptimizerWorkspace* workspace,
                                   InverseDynamicsPartials* id_partials) const;

  /**
   * Compute partial derivatives of the generalized velocities
   *
   *    v_t = N+(q_t) * (q_t - q_{t-1}) / dt
   *
   * and store them in the given VelocityPartials struct
   *
   * @param q sequence of generalized positions
   * @param v_partials struct for holding dv/dq
   */
  void CalcVelocityPartials(const std::vector<VectorXd>& q,
                            VelocityPartials* v_partials) const;

  /**
   * Compute partial derivatives of the inverse dynamics
   *
   *    tau_t = ID(q_{t-1}, q_t, q_{t+1})
   *
   * using finite differences.
   *
   * For testing purposes only - this is very inefficient.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities (computed from q)
   * @param a sequence of generalized accelerations (computed from q)
   * @param tau sequence of generalized forces (computed from q)
   * @param workspace scratch space for intermediate computations
   * @param id_partials struct for holding dtau/dq
   */
  void CalcInverseDynamicsPartialsFiniteDiff(
      const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
      const std::vector<VectorXd>& a, const std::vector<VectorXd>& tau,
      TrajectoryOptimizerWorkspace* workspace,
      InverseDynamicsPartials* id_partials) const;

  /**
   * Compute the gradient of the unconstrained cost L(q) using finite
   * differences.
   *
   * Uses central differences, so with a perturbation on the order of eps^(1/3),
   * we expect errors on the order of eps^(2/3).
   *
   * For testing purposes only.
   *
   * @param state optimizer state, including q, v, tau, gradients, etc.
   * @param g a single VectorXd containing the partials of L w.r.t. each
   *          decision variable (q_t[i]).
   */
  void CalcGradientFiniteDiff(const TrajectoryOptimizerState& state,
                              EigenPtr<VectorXd> g) const;

  // A model of the system that we are trying to find an optimal trajectory for.
  const MultibodyPlant<double>* plant_;

  // A context corresponding to plant_, to enable dynamics computations.
  std::unique_ptr<Context<double>> context_;

  // Stores the problem definition, including cost, time horizon, initial state,
  // target state, etc.
  const ProblemDefinition prob_;

  // Joint damping coefficients for the plant under consideration
  VectorXd joint_damping_;
};

}  // namespace traj_opt
}  // namespace drake
