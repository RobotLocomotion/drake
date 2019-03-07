#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace controllers_test {

// Computes torque predicted by inverse dynamics for use with inverse dynamics
// and inverse dynamics controller testing.
VectorX<double> ComputeTorque(const RigidBodyTree<double>& tree,
                              const VectorX<double>& q,
                              const VectorX<double>& v,
                              const VectorX<double>& vd_d) {
  // Compute the expected torque.
  KinematicsCache<double> cache = tree.doKinematics(q, v);
  eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                  drake::TwistVector<double>> f_ext;

  return tree.massMatrix(cache) * vd_d + tree.dynamicsBiasTerm(cache, f_ext);
}

}  // namespace controllers_test
}  // namespace systems
}  // namespace drake
