#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace traj_opt {

using Eigen::VectorXd;
using multibody::MultibodyForces;
using multibody::MultibodyPlant;

/**
 * A container for scratch variables that we use in various intermediate
 * computations. Allows us to avoid extra allocations when speed is important.
 */
struct TrajectoryOptimizerWorkspace {
  // Construct a workspace with size matching the given plant.
  explicit TrajectoryOptimizerWorkspace(const MultibodyPlant<double>& plant)
      : f_ext(plant) {
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();

    // Set vector sizes
    q_size_tmp.resize(nq);

    v_size_tmp1.resize(nv);
    v_size_tmp2.resize(nv);

    tau_size_tmp1.resize(nv);
    tau_size_tmp2.resize(nv);
    tau_size_tmp3.resize(nv);

    a_size_tmp1.resize(nq);
    a_size_tmp2.resize(nq);
    a_size_tmp3.resize(nq);
  }

  // Storage for multibody forces
  MultibodyForces<double> f_ext;

  // Storage of size nq
  VectorXd q_size_tmp;

  // Storage of size nv
  // These are named v, tau, and a, but this distinction is just for
  // convienience.
  VectorXd v_size_tmp1;
  VectorXd v_size_tmp2;

  VectorXd tau_size_tmp1;
  VectorXd tau_size_tmp2;
  VectorXd tau_size_tmp3;

  VectorXd a_size_tmp1;
  VectorXd a_size_tmp2;
  VectorXd a_size_tmp3;
};

}  // namespace traj_opt
}  // namespace drake
