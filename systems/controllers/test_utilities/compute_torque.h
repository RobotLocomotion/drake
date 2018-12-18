#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"

namespace drake {
namespace systems {
namespace controllers_test {

// Computes torque predicted by inverse dynamics for use with inverse dynamics
// and inverse dynamics controller testing.
VectorX<double> ComputeTorque(
    const multibody::MultibodyPlant<double>& plant,
    const VectorX<double>& q,
    const VectorX<double>& v,
    const VectorX<double>& vd_d,
    systems::Context<double>* context) {
  // Populate the context.
  context->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q);
  context->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v);

  // Compute the caches.
  auto& tree = plant.tree();
  multibody::PositionKinematicsCache<double> pcache(tree.get_topology());
  multibody::VelocityKinematicsCache<double> vcache(tree.get_topology());
  tree.CalcPositionKinematicsCache(*context, &pcache);
  tree.CalcVelocityKinematicsCache(*context, pcache, &vcache);

  // Compute inverse dynamics.
  VectorX<double> tau_applied = tree.CalcGravityGeneralizedForces(*context);
  std::vector<multibody::SpatialAcceleration<double>> A_WB_array(
      tree.num_bodies());
  std::vector<multibody::SpatialForce<double>> F_BMo_W_array(
      tree.num_bodies());
  for (auto& f : F_BMo_W_array)
    f.SetZero();
  VectorX<double> tau_array(tree.num_velocities());

  tree.CalcInverseDynamics(*context, pcache, vcache, vd_d, {}, tau_applied,
                           &A_WB_array, &F_BMo_W_array, &tau_array);

  return tau_array;
}

}  // namespace controllers_test
}  // namespace systems
}  // namespace drake
