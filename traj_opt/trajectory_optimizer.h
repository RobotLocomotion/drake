#pragma once

#include <memory>
#include <optional>
#include <string>
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
namespace systems {
// Forward declaration to avoid polluting this namespace with systems:: stuff.
template <typename>
class Diagram;
}  // namespace systems

namespace traj_opt {

using internal::PentaDiagonalMatrix;
using multibody::MultibodyPlant;
using systems::Context;
using systems::Diagram;

template <typename T>
class TrajectoryOptimizer {
 public:
  /**
   * Construct a new Trajectory Optimizer object.
   *
   * @param plant A model of the system that we're trying to find an optimal
   *              trajectory for.
   * @param context A context for the plant, used to perform various multibody
   *                dynamics computations. Should be part of a larger Diagram
   *                context, and be connected to a scene graph.
   * @param prob Problem definition, including cost, initial and target states,
   *             etc.
   * @param params solver parameters, including max iterations, linesearch
   *               method, etc.
   */
  // TODO(amcastro-tri): Get rid of this constructor. Favor the new construction
  // below so that we can cache the context at each time step in the state.
  TrajectoryOptimizer(const MultibodyPlant<T>* plant, Context<T>* context,
                      const ProblemDefinition& prob,
                      const SolverParameters& params = SolverParameters{});

  /**
   * Construct a new Trajectory Optimizer object.
   *
   * @param diagram Diagram for the entire model that will include the plant and
   * SceneGraph for geometric queries. Used to allocate context resources.
   * @param plant A model of the system that we're trying to find an optimal
   *              trajectory for.
   * @param prob Problem definition, including cost, initial and target states,
   *             etc.
   * @param params solver parameters, including max iterations, linesearch
   *               method, etc.
   */
  TrajectoryOptimizer(const Diagram<T>* diagram, const MultibodyPlant<T>* plant,
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
    if (diagram_ != nullptr) {
      return TrajectoryOptimizerState<T>(num_steps(), *diagram_, plant());
    }
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

  // Evaluates the MultibodyPlant context at the t-th time.
  const Context<T>& EvalPlantContext(const TrajectoryOptimizerState<T>& state,
                                     int t) const;

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

  /* Evaluates the signed distance pairs for the t-th step stored in `state`. */
  const std::vector<geometry::SignedDistancePair<T>>& EvalSignedDistancePairs(
      const TrajectoryOptimizerState<T>& state, int t) const;

  /* Evaluates data storing contact Jacobians at all time steps in `state`. */
  const typename TrajectoryOptimizerCache<T>::ContactJacobianData&
  EvalContactJacobianData(const TrajectoryOptimizerState<T>& state) const;

 private:
  // Friend class to facilitate testing.
  friend class TrajectoryOptimizerTester;

  /**
   * Solve the optimization problem from the given initial guess using a
   * linesearch strategy.
   *
   * @param q_guess a sequence of generalized positions corresponding to the
   * initial guess
   * @param solution a container for the optimal solution, including velocities
   * and torques
   * @param stats a container for other timing and iteration-specific
   * data regarding the solve process.
   * @return SolverFlag
   */
  SolverFlag SolveWithLinesearch(const std::vector<VectorX<T>>& q_guess,
                                 TrajectoryOptimizerSolution<T>* solution,
                                 TrajectoryOptimizerStats<T>* stats) const;

  /**
   * Solve the optimization problem from the given initial guess using a trust
   * region strategy.
   *
   * @param q_guess a sequence of generalized positions corresponding to the
   * initial guess
   * @param solution a container for the optimal solution, including velocities
   * and torques
   * @param stats a container for other timing and iteration-specific
   * data regarding the solve process.
   * @return SolverFlag
   */
  SolverFlag SolveWithTrustRegion(const std::vector<VectorX<T>>& q_guess,
                                  TrajectoryOptimizerSolution<T>* solution,
                                  TrajectoryOptimizerStats<T>* stats) const;

  // Updates `cache` to store q and v from `state`.
  void CalcContextCache(
      const TrajectoryOptimizerState<T>& state,
      typename TrajectoryOptimizerCache<T>::ContextCache* cache) const;

  /**
   * Compute all of the "trajectory data" (velocities v, accelerations a,
   * torques tau) in the state's cache to correspond to the state's generalized
   * positions q.
   *
   * @param state optimizer state to update.
   */
  void CalcCacheTrajectoryData(const TrajectoryOptimizerState<T>& state) const;

  /**
   * Compute all of the "derivatives data" (dv/dq, dtau/dq) stored in the
   * state's cache to correspond to the state's generalized positions q.
   *
   * @param state optimizer state to update.
   */
  void CalcCacheDerivativesData(const TrajectoryOptimizerState<T>& state) const;

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
   * Calculate the force contribution from contacts for each body, and add them
   * into the given MultibodyForces object.
   *
   * @param forces total forces applied to the plant, which we will add into.
   * @pre generalized positions (q) and velocities (v) have been properly set in
   *      context_
   */
  void CalcContactForceContribution(MultibodyForces<T>* forces) const;

  /* Computes signed distance data for all time configurations in `state`. */
  void CalcSdfData(
      const TrajectoryOptimizerState<T>& state,
      typename TrajectoryOptimizerCache<T>::SdfData* sdf_data) const;  

  /* Helper to compute the contact Jacobian for the configuration stored in
  `context`. Signed distance pairs `sdf_pairs` must be consistent with
  `context`. */
  void CalcContactJacobian(
      const Context<T>& context,
      const std::vector<geometry::SignedDistancePair<T>>& sdf_pairs,
      MatrixX<T>* J, std::vector<math::RotationMatrix<T>>* R_WC,
      std::vector<std::pair<BodyIndex, BodyIndex>>* body_pairs) const;

  /* Computes the Jacobian data for all time step configurations stored in
   * `state`.*/
  void CalcContactJacobianData(
      const TrajectoryOptimizerState<T>& state,
      typename TrajectoryOptimizerCache<T>::ContactJacobianData*
          contact_jacobian_data) const;  

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
      const TrajectoryOptimizerState<T>& state,
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
      const TrajectoryOptimizerState<T>& state,
      TrajectoryOptimizerWorkspace<T>* workspace,
      InverseDynamicsPartials<T>* id_partials) const;

  void CalcInverseDynamicsPartialsAutoDiff(
      const TrajectoryOptimizerState<double>& state,
      InverseDynamicsPartials<double>* id_partials) const;

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
  void SaveLinesearchResidual(
      const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
      TrajectoryOptimizerState<T>* scratch_state,
      const std::string filename = "linesearch_data.csv") const;

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

  /**
   * Compute the trust ratio
   *
   *           L(q) - L(q + dq)
   *    rho =  ----------------
   *             m(0) - m(dq)
   *
   * which compares the actual reduction in cost to the reduction in cost
   * predicted by the quadratic model
   *
   *    m(dq) = L + g'*dq + 1/2 dq'*H*dq
   *
   * @param state optimizer state containing q and everything computed from q
   * @param dq change in q, stacked in one large vector
   * @param scratch_state scratch state variable used to compute L(q+dq)
   * @return T, the trust region ratio
   */
  T CalcTrustRatio(const TrajectoryOptimizerState<T>& state,
                   const VectorX<T>& dq,
                   TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Compute the dogleg step δq, which approximates the solution to the
   * trust-region sub-problem
   *
   *   min_{δq} L(q) + g(q)'*δq + 1/2 δq'*H(q)*δq
   *   s.t.     ‖ δq ‖ <= Δ
   *
   * @param state the optimizer state, containing q and the ability to compute
   * g(q) and H(q)
   * @param Delta the trust region size
   * @param dq  the dogleg step (change in decision variables)
   * @return true if the step intersects the trust region
   * @return false if the step is in the interior of the trust region
   */
  bool CalcDoglegPoint(const TrajectoryOptimizerState<T>& state,
                       const double Delta, VectorX<T>* dq) const;

  /**
   * Solve the scalar quadratic equation
   *
   *    a x² + b x + c = 0
   *
   * for the positive root. This problem arises from finding the intersection
   * between the trust region and the second leg of the dogleg path. Provided we
   * have properly checked that the trust region does intersect this seconds
   * leg, this quadratic equation has some special properties:
   *
   *     - a is strictly positive
   *     - there is exactly one positive root
   *     - this positive root is in (0,1)
   *
   * @param a the first coefficient
   * @param b the second coefficient
   * @param c the third coefficient
   * @return T the positive root
   */
  T SolveDoglegQuadratic(const T& a, const T& b, const T& c) const;

  /**
   * Save the cost L(q) for a variety of values of q so that we can make a
   * contour plot (later) in python.
   *
   * Only changes the first two values of q(t) at t=1, so we can plot in 2d.
   *
   * @param scratch_state State variable used to compute L(q) for a variety of
   * values of q.
   */
  void SaveContourPlotDataFirstTwoVariables(
      TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Save the cost, gradient, and Hessian accross a range of values of q(1)[0],
   * where q(1)[0] is the first state variable at timestep t=1.
   *
   * This data will be used later to make debugging plots of L, g, and H.
   *
   * @param scratch_state State variable used to compute L(q), g(q), and H(q).
   */
  void SaveLinePlotDataFirstVariable(
      TrajectoryOptimizerState<T>* scratch_state) const;

  /**
   * Clear the file `iteration_data.csv` and write a csv header so we can later
   * record iteration data with SaveIterationData().
   */
  void SetupIterationDataFile() const;

  /**
   * Save iteration-specific data (like cost, q, etc) to a file.
   *
   * @param iter_num iteration number
   * @param Delta trust region radius
   * @param dq change in first decision variable
   * @param rho trust ratio
   * @param state state variable used to store q and compute L(q), g(q), H(q),
   * etc
   */
  void SaveIterationData(const int iter_num, const double Delta,
                         const double dq, const double rho,
                         const TrajectoryOptimizerState<T>& state) const;

  /**
   * Clear the file `quadratic_data.csv` and write a csv header so we can later
   * record iteration data with SaveQuadraticDataFirstTwoVariables().
   */
  void SetupQuadraticDataFile() const;

  /**
   * Save the cost L(q), gradient g(q), and Hessian approximation H(q) for the
   * first two variables of the optimization problem at the given iteration.
   *
   * @warning this appends a row to `quadratic_data.csv`, without
   * establishing any csv header or clearning the file. Make sure to call
   * SetupQuadraticDataFile() first.
   *
   * @param iter_num iteration number that we're on
   * @param Delta trust region radius
   * @param dq variable step for this iteration.
   * @param state optimizer state containing q, from which we can compute L, g,
   * and H
   */
  void SaveQuadraticDataFirstTwoVariables(
      const int iter_num, const double Delta, const VectorX<T>& dq,
      const TrajectoryOptimizerState<T>& state) const;

  // Diagram of containing the plant_ model and scene graph. Needed to allocate
  // context resources.
  const Diagram<T>* diagram_{nullptr};

  // A model of the system that we are trying to find an optimal trajectory for.
  const MultibodyPlant<T>* plant_{nullptr};

  // A context corresponding to plant_, to enable dynamics computations. Must be
  // connected to a larger Diagram with a SceneGraph for systems with contact.
  Context<T>* context_{nullptr};

  // Temporary workaround for when context_ is not provided at construction.
  // TODO(amcastro-tri): Get rid of context_ and owned_context_.
  std::unique_ptr<Context<T>> owned_context_;

  // Stores the problem definition, including cost, time horizon, initial state,
  // target state, etc.
  const ProblemDefinition prob_;

  // Joint damping coefficients for the plant under consideration
  VectorX<T> joint_damping_;

  // Various parameters
  const SolverParameters params_;

  std::unique_ptr<Diagram<AutoDiffXd>> diagram_ad_;
  const MultibodyPlant<AutoDiffXd>* plant_ad_;
  std::unique_ptr<TrajectoryOptimizer<AutoDiffXd>> optimizer_ad_;
  std::unique_ptr<TrajectoryOptimizerState<AutoDiffXd>> state_ad_;
};

// Declare template specializations
template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithLinesearch(
    const std::vector<VectorXd>&, TrajectoryOptimizerSolution<double>*,
    TrajectoryOptimizerStats<double>*) const;

template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithTrustRegion(
    const std::vector<VectorXd>&, TrajectoryOptimizerSolution<double>*,
    TrajectoryOptimizerStats<double>*) const;

}  // namespace traj_opt
}  // namespace drake
