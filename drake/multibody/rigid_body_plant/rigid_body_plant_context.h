#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/systems/framework/leaf_context.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace systems {

/// %RigidBodyPlantContext is a temporary fix providing a caching like
/// capability to RigidBodyPlant.
/// %RigidBodyPlantContext allows to save in the RigidBodyPlant context a number
/// of expensive to compute calculations. These include contact results,
/// kinematics and generalized forces.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
// TODO(amcastro-tri): Replace this hacky implementation by one actually
// using System 2.0's Cache.
template <typename T>
class RigidBodyPlantContext : public LeafContext<T> {
 public:
  explicit RigidBodyPlantContext(const RigidBodyPlant<T>& rbp);
  virtual ~RigidBodyPlantContext() {}

  // Context<T> overrides.
  void InvalidateContinuousStateVectorDependents() const final {
    is_kinematics_cache_valid_ = false;
    is_contact_results_valid_ = false;
  }

 private:
  // RigidBodyPlantContext objects are neither copyable nor moveable.
  RigidBodyPlantContext(const RigidBodyPlantContext& other) = delete;
  RigidBodyPlantContext& operator=(const RigidBodyPlantContext& other) = delete;
  RigidBodyPlantContext(RigidBodyPlantContext&& other) = delete;
  RigidBodyPlantContext& operator=(RigidBodyPlantContext&& other) = delete;

 public:
  // "Cached" entries in the RigidBodyPlantContext.
  mutable bool is_kinematics_cache_valid_{false};
  mutable KinematicsCache<T> kinematics_cache_;

  mutable bool is_contact_results_valid_{false};
  mutable ContactResults<T> contact_results_;
  mutable VectorX<T> generalized_contact_forces_;
};

}  // namespace systems
}  // namespace drake
