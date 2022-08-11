#pragma once

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/inverse_dynamics_partials.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"
#include "drake/traj_opt/problem_definition.h"
#include "drake/traj_opt/solver_parameters.h"
#include "drake/traj_opt/trajectory_optimizer_solution.h"
#include "drake/traj_opt/trajectory_optimizer_state.h"
#include "drake/traj_opt/trajectory_optimizer_workspace.h"
#include "drake/traj_opt/velocity_partials.h"

namespace drake {
namespace traj_opt {

using internal::PentaDiagonalMatrix;
using multibody::MultibodyPlant;
using systems::Context;

template <typename T>
class TrajectoryOptimizer {
 public:
  /**
   * Construct a new Trajectory Optimizer object.
   *
   * @param plant A model of the system that we're trying to find an optimal
   *              trajectory for.
   * @param prob Problem definition, including cost, initial and target states,
   *             etc.
   * @param params solver parameters, including max iterations, linesearch
   *               method, etc.
   */
  TrajectoryOptimizer(const MultibodyPlant<T>* plant,
                      const ProblemDefinition& prob,
                      const SolverParameters& params = SolverParameters{});

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
   * @return const MultibodyPlant<T>&, the plant we're optimizing over.
   */
  const MultibodyPlant<T>& plant() const { return *plant_; }

  /**
   * Create a state object which contains the decision variables (generalized
   * positions at each timestep), along with a cache of other things that are
   * computed from positions, such as velocities, accelerations, forces, and
   * various derivatives.
   *
   * @return TrajectoryOptimizerState
   */
  TrajectoryOptimizerState<T> CreateState() const {
    return TrajectoryOptimizerState<T>(num_steps(), plant());
  }

  /**
   * Compute the gradient of the unconstrained cost L(q).
   *
   * @param state optimizer state, including q, v, tau, gradients, etc.
   * @param g a single VectorXd containing the partials of L w.r.t. each
   *          decision variable (q_t[i]).
   */
  void CalcGradient(const TrajectoryOptimizerState<T>& state,
                    EigenPtr<VectorX<T>> g) const;

  /**
   * Compute the Hessian of the unconstrained cost L(q) as a sparse
   * penta-diagonal matrix.
   *
   * @param state optimizer state, including q, v, tau, gradients, etc.
   * @param H a PentaDiagonalMatrix containing the second-order derivatives of
   *          the total cost L(q). This matrix is composed of (num_steps+1 x
   *          num_steps+1) blocks of size (nq x nq) each.
   */
  void CalcHessian(const TrajectoryOptimizerState<T>& state,
                   PentaDiagonalMatrix<T>* H) const;

  /**
   * Solve the optimization from the given initial guess, which may or may not
   * be dynamically feasible.
   *
   * @param q_guess a sequence of generalized positions corresponding to the
   * initial guess
   * @param solution a container for the optimal solution, including velocities
   * and torques
   * @param stats a container for other timing and iteration-specific
   * data regarding the solve process.
   * @return SolverFlag
   */
  SolverFlag Solve(const std::vector<VectorX<T>>& q_guess,
                   TrajectoryOptimizerSolution<T>* solution,
                   TrajectoryOptimizerStats<T>* stats) const;

  // Evaluator functions to get data from the state's cache, and update it if
  // necessary.
  const std::vector<VectorX<T>>& EvalV(
      const TrajectoryOptimizerState<T>& state) const;
  const std::vector<VectorX<T>>& EvalA(
      const TrajectoryOptimizerState<T>& state) const;
  const std::vector<VectorX<T>>& EvalTau(
      const TrajectoryOptimizerState<T>& state) const;

  const VelocityPartials<T>& EvalVelocityPartials(
      const TrajectoryOptimizerState<T>& state) const;
  const InverseDynamicsPartials<T>& EvalInverseDynamicsPartials(
      const TrajectoryOptimizerState<T>& state) const;

  const T EvalCost(const TrajectoryOptimizerState<T>& state) const;
  const PentaDiagonalMatrix<T>& EvalHessian(
      const TrajectoryOptimizerState<T>& state) const;
  const VectorX<T>& EvalGradient(
      const TrajectoryOptimizerState<T>& state) const;

 private:
  // Friend class to facilitate testing.
  friend class TrajectoryOptimizerTester;

  /**
   * Compute all of the "trajectory data" (velocities v, accelerations a,
   * torques tau) in the state's cache to correspond to the state's generalized
   * positions q.
   *
   * @param state optimizer state to update.
   */
  void CalcCacheTrajectoryData(
      const TrajectoryOptimizerState<T>& state) const;

  /**
   * Compute all of the "derivatives data" (dv/dq, dtau/dq) stored in the
   * state's cache to correspond to the state's generalized positions q.
   *
   * @param state optimizer state to update.
   */
  void CalcCacheDerivativesData(
      const TrajectoryOptimizerState<T>& state) const;

  /**
   * Return the total (unconstrained) cost of the optimization problem,
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
   * A cached version of this cost is stored in the state. If the cache is up to
   * date, simply return that cost.
   *
   * @param state optimizer state
   * @return double, total cost
   */
  T CalcCost(const TrajectoryOptimizerState<T>& state) const;

  /**
   * Compute the total cost of the unconstrained problem.
   *
   * @param q sequence of generalized positions
   * @param v sequence of generalized velocities (consistent with q)
   * @param tau sequence of generalized forces (consistent with q and v)
   * @param workspace scratch space for intermediate computations
   * @return double, total cost
   */
  T CalcCost(const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
             const std::vector<VectorX<T>>& tau,
             TrajectoryOptimizerWorkspace<T>* workspace) const;

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
  void CalcVelocities(const std::vector<VectorX<T>>& q,
                      std::vector<VectorX<T>>* v) const;

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
  void CalcAccelerations(const std::vector<VectorX<T>>& v,
                         std::vector<VectorX<T>>* a) const;

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
  void CalcInverseDynamics(const std::vector<VectorX<T>>& q,
                           const std::vector<VectorX<T>>& v,
                           const std::vector<VectorX<T>>& a,
                           TrajectoryOptimizerWorkspace<T>* workspace,
                           std::vector<VectorX<T>>* tau) const;

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
      const VectorX<T>& q, const VectorX<T>& v, const VectorX<T>& a,
      TrajectoryOptimizerWorkspace<T>* workspace, VectorX<T>* tau) const;

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
  void CalcInverseDynamicsPartials(
      const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
      const std::vector<VectorX<T>>& a, const std::vector<VectorX<T>>& tau,
      TrajectoryOptimizerWorkspace<T>* workspace,
      InverseDynamicsPartials<T>* id_partials) const;

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
  void CalcVelocityPartials(const std::vector<VectorX<T>>& q,
                            VelocityPartials<T>* v_partials) const;

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
      const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
      const std::vector<VectorX<T>>& a, const std::vector<VectorX<T>>& tau,
      TrajectoryOptimizerWorkspace<T>* workspace,
      InverseDynamicsPartials<T>* id_partials) const;

  /**
   * Compute the gradient of the unconstrained cost L(q) using finite
   * differences.
   *
   * Uses central differences, so with a perturbation on the order of eps^(1/3),
   * we expect errors on the order of eps^(2/3).
   *
   * For testing purposes only.
   *
   * @param q vector of generalized positions at each timestep
   * @param g a single VectorX<T> containing the partials of L w.r.t. each
   *          decision variable (q_t[i]).
   */
  void CalcGradientFiniteDiff(const TrajectoryOptimizerState<T>& state,
                              EigenPtr<VectorX<T>> g) const;

  /**
   * Compute the linesearch parameter alpha given a linesearch direction
   * dq. In other words, approximately solve the optimization problem
   *
   *      min_{alpha} L(q + alpha*dq).
   *
   * @param state the optimizer state containing q and everything that we
   *              compute from q
   * @param dq search direction, stacked as one large vector
   * @param scratch_state scratch state variable used for computing L(q +
   *                      alpha*dq)
   * @return double, the linesearch parameter alpha
   * @return int, the number of linesearch iterations
   */
  std::tuple<double, int> Linesearch(
      const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
      TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Debugging function which saves the line-search residual
   *
   *    phi(alpha) = L(q + alpha*dq)
   *
   * for various values of alpha to a file.
   *
   * This allows us to make a nice plot in python after the fact
   */
  void SaveLinesearchResidual(const TrajectoryOptimizerState<T>& state,
                              const VectorX<T>& dq,
                              TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Simple backtracking linesearch strategy to find alpha that satisfies
   *
   *    L(q + alpha*dq) < L(q) + c*g'*dq
   *
   * and is (approximately) a local minimizer of L(q + alpha*dq).
   */
  std::tuple<double, int> BacktrackingLinesearch(
      const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
      TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Simple backtracking linesearch strategy to find alpha that satisfies
   *
   *    L(q + alpha*dq) < L(q) + c*g'*dq
   */
  std::tuple<double, int> ArmijoLinesearch(
      const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
      TrajectoryOptimizerState<T>* scratch_state) const;

  // A model of the system that we are trying to find an optimal trajectory for.
  const MultibodyPlant<T>* plant_;

  // A context corresponding to plant_, to enable dynamics computations.
  std::unique_ptr<Context<T>> context_;

  // Stores the problem definition, including cost, time horizon, initial state,
  // target state, etc.
  const ProblemDefinition prob_;

  // Joint damping coefficients for the plant under consideration
  VectorX<T> joint_damping_;

  // Various parameters
  SolverParameters params_;
};

}  // namespace traj_opt
}  // namespace drake
