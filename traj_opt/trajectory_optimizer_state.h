#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/inverse_dynamics_partials.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"
#include "drake/traj_opt/trajectory_optimizer_workspace.h"
#include "drake/traj_opt/velocity_partials.h"

namespace drake {
namespace traj_opt {

using internal::PentaDiagonalMatrix;
using multibody::MultibodyPlant;

/**
 * Struct for holding quantities that are computed from the optimizer state (q),
 * such as generalized velocities (v), accelerations (a), and forces (tau), as
 * well as various derivatives.
 *
 * We separate the trajectory data (v, a, tau) and derivative data (v_partials,
 * id_partials), since we often need to just use the trajectory data, for
 * instance to compute the total cost, without performing expensive updates of
 * the derivative data.
 */
template <typename T>
struct TrajectoryOptimizerCache {
  TrajectoryOptimizerCache(const int num_steps, const int nv, const int nq)
      : derivatives_data(num_steps, nv, nq),
        gradient((num_steps + 1) * nq),
        hessian(num_steps + 1, nq) {
    trajectory_data.v.assign(num_steps + 1, VectorX<T>(nv));
    trajectory_data.a.assign(num_steps, VectorX<T>(nv));
    trajectory_data.tau.assign(num_steps, VectorX<T>(nv));
  }

  // Data used to compute the cost L(q)
  struct TrajectoryData {
    // Generalized velocities at each timestep
    // [v(0), v(1), ..., v(num_steps)]
    std::vector<VectorX<T>> v;

    // Generalized accelerations at each timestep
    // [a(0), a(1), ..., a(num_steps-1)]
    std::vector<VectorX<T>> a;

    // Generalized forces at each timestep
    // [tau(0), tau(1), ..., tau(num_steps-1)]
    std::vector<VectorX<T>> tau;

    bool up_to_date{false};
  } trajectory_data;

  // Data used to construct the gradient ∇L and Hessian ∇²L approximation
  struct DerivativesData {
    DerivativesData(const int num_steps, const int nv, const int nq)
        : v_partials(num_steps, nv, nq), id_partials(num_steps, nv, nq) {}

    // Storage for dv(t)/dq(t) and dv(t)/dq(t-1)
    VelocityPartials<T> v_partials;

    // Storage for dtau(t)/dq(t-1), dtau(t)/dq(t), and dtau(t)/dq(t+1)
    InverseDynamicsPartials<T> id_partials;

    bool up_to_date{false};
  } derivatives_data;

  // The total cost L(q)
  T cost;
  bool cost_up_to_date{false};

  // The gradient of the unconstrained cost ∇L
  VectorX<T> gradient;
  bool gradient_up_to_date{false};

  // Our Hessian approximation of the unconstrained cost ∇²L
  PentaDiagonalMatrix<T> hessian;
  bool hessian_up_to_date{false};
};

/**
 * Struct for storing the "state" of the trajectory optimizer.
 *
 * The only actual state is the sequence of generalized positions q at each
 * timestep. This class stores that directly, but also a "cache" of other values
 * computed from q, such as generalized velocities and forces at each timesteps,
 * relevant dynamics partials, etc.
 */
template <typename T>
class TrajectoryOptimizerState {
 public:
  /**
   * Constructor which allocates things of the proper sizes.
   *
   * @param num_steps number of timesteps in the optimization problem
   * @param nv number of multibody velocities
   * @param nq number of multipody positions
   */
  TrajectoryOptimizerState(const int num_steps, const MultibodyPlant<T>& plant)
      : workspace(num_steps, plant),
        num_steps_(num_steps),
        nq_(plant.num_positions()),
        cache_(num_steps, plant.num_velocities(), plant.num_positions()) {
    q_.assign(num_steps + 1, VectorX<T>(plant.num_positions()));
  }

  /**
   * Getter for the sequence of generalized positions.
   *
   * @return const std::vector<VectorX<T>>& q
   */
  const std::vector<VectorX<T>>& q() const { return q_; }

  /**
   * Setter for the sequence of generalized positions. Invalidates the cache.
   *
   * @param q sequence of generalized positions at each time step
   */
  void set_q(const std::vector<VectorX<T>>& q) {
    q_ = q;
    invalidate_cache();
  }

  /**
   * Update the sequence of generalized positions, q, by adding
   *
   *    q = q + dq,
   *
   * where dq is a large vector which stacks changes in each q[t].
   *
   * @param dq vector of changes in generalized positions
   */
  void AddToQ(const VectorX<T>& dq) {
    DRAKE_DEMAND(dq.size() == nq_ * (num_steps_ + 1));
    for (int t = 0; t <= num_steps_; ++t) {
      q_[t] += dq.segment(t * nq_, nq_);
    }
    invalidate_cache();
  }

  /**
   * Getter for the cache, containing other values computed from q, such as
   * generalized velocities, forces, and various dynamics derivatives.
   *
   * @return const TrajectoryOptimizerCache& cache
   */
  const TrajectoryOptimizerCache<T>& cache() const { return cache_; }

  /**
   * Get a mutable copy of the cache, containing other values computed from q,
   * such as generalized velocities, forces, and various dynamics derivatives.
   *
   * @return TrajectoryOptimizerCache&
   */
  TrajectoryOptimizerCache<T>& mutable_cache() const { return cache_; }

  /**
   * Scratch space for intermediate computations, to avoid expensive
   * allocations.
   */
  mutable TrajectoryOptimizerWorkspace<T> workspace;

 private:
  // Number of timesteps in the optimization problem
  const int num_steps_;

  // Number of multibody positions for this system
  const int nq_;

  // Sequence of generalized velocities at each timestep,
  // [q(0), q(1), ..., q(num_steps)]
  // TODO(vincekurtz): consider storing as a single VectorX<T> for better memory
  // layout.
  std::vector<VectorX<T>> q_;

  // Storage for all other quantities that are computed from q, and are useful
  // for our calculations
  mutable TrajectoryOptimizerCache<T> cache_;

  // Set all the cache invalidation flags to false
  void invalidate_cache() {
    cache_.trajectory_data.up_to_date = false;
    cache_.derivatives_data.up_to_date = false;
    cache_.cost_up_to_date = false;
    cache_.gradient_up_to_date = false;
    cache_.hessian_up_to_date = false;
  }
};

}  // namespace traj_opt
}  // namespace drake
