#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace traj_opt {

using multibody::MultibodyForces;
using multibody::MultibodyPlant;

/**
 * A container for scratch variables that we use in various intermediate
 * computations. Allows us to avoid extra allocations when speed is important.
 */
template <typename T>
struct TrajectoryOptimizerWorkspace {
  // Construct a workspace with size matching the given plant.
  TrajectoryOptimizerWorkspace(const int num_steps,
                               const MultibodyPlant<T>& plant)
      : f_ext(plant) {
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();

    // Set vector sizes
    q_size_tmp1.resize(nq);
    q_size_tmp2.resize(nq);

    v_size_tmp1.resize(nv);
    v_size_tmp2.resize(nv);

    tau_size_tmp1.resize(nv);
    tau_size_tmp2.resize(nv);
    tau_size_tmp3.resize(nv);

    a_size_tmp1.resize(nq);
    a_size_tmp2.resize(nq);
    a_size_tmp3.resize(nq);

    q_times_num_steps_size_tmp.resize(nq * (num_steps + 1));

    // Allocate sequences
    q_sequence_tmp1.assign(num_steps, VectorX<T>(nq));
    q_sequence_tmp2.assign(num_steps, VectorX<T>(nq));
  }

  // Storage for multibody forces
  MultibodyForces<T> f_ext;

  // Storage of size nq
  VectorX<T> q_size_tmp1;
  VectorX<T> q_size_tmp2;

  // Storage of size nv
  // These are named v, tau, and a, but this distinction is just for
  // convienience.
  VectorX<T> v_size_tmp1;
  VectorX<T> v_size_tmp2;

  VectorX<T> tau_size_tmp1;
  VectorX<T> tau_size_tmp2;
  VectorX<T> tau_size_tmp3;

  VectorX<T> a_size_tmp1;
  VectorX<T> a_size_tmp2;
  VectorX<T> a_size_tmp3;

  // Storage of sequence of q
  std::vector<VectorX<T>> q_sequence_tmp1;
  std::vector<VectorX<T>> q_sequence_tmp2;

  // Vector of all decision variables
  VectorX<T> q_times_num_steps_size_tmp;
};

}  // namespace traj_opt
}  // namespace drake
