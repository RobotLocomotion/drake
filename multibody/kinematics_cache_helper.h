#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace plants {
/// Helper class to avoid recalculating a kinematics cache which is
/// going to be used repeatedly by multiple other classes.
template <typename Scalar>
class KinematicsCacheHelper {
 public:
  /**
   * Construct a cache for a tree.
   * @param tree tree is aliased and needs to live for the lifetime of the
   * object.
   */
  explicit KinematicsCacheHelper(const RigidBodyTree<double>& tree);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q,
      const RigidBodyTree<double>* tree);

 private:
  VectorX<Scalar> last_q_;
  const RigidBodyTree<double>* last_tree_;
  KinematicsCache<Scalar> kinematics_cache_;
};

/**
 * Stores and updates the kinematics cache for the rigid body tree.
 */
template <typename Scalar>
class KinematicsCacheWithVHelper {
 public:
  /**
   * Construct a cache for a tree.
   * The kinematics information includes the pose and spatial velocity, w.r.t
   * q, v. And Jdotv is computed.
   * @param tree tree is aliased and needs to live for the lifetime of the
   * object.
   */
  explicit KinematicsCacheWithVHelper(const RigidBodyTree<double>& tree);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q,
      const Eigen::Ref<const VectorX<Scalar>>& v);

 private:
  const RigidBodyTree<double>* tree_;
  VectorX<Scalar> last_q_;
  VectorX<Scalar> last_v_;
  KinematicsCache<Scalar> kinematics_cache_;
};
}  // namespace plants
}  // namespace systems
}  // namespace drake
