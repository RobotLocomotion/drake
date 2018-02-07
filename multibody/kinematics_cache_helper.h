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
  explicit KinematicsCacheHelper(const RigidBodyTree<double>& tree);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q,
      const RigidBodyTree<double>* tree);

 private:
  VectorX<Scalar> last_q_;
  const RigidBodyTree<double>* last_tree_;
  KinematicsCache<Scalar> kinsol_;
};

/**
 * Stores and updates the kinematics cache for the rigid body tree. 
 */
template <typename Scalar>
class KinematicsCacheWithVHelper {
 public:
  explicit KinematicsCacheWithVHelper(const RigidBodyTree<double>& tree)
      : tree_{&tree}, kinsol_(tree.CreateKinematicsCacheWithType<Scalar>()) {
    last_q_.resize(0);
    last_v_.resize(0);
  }

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q) {
    if (q.size() != last_q_.size() || q != last_q_) {
      last_q_ = q;
      kinsol_.initialize(q, last_v_);
      tree_->doKinematics(kinsol_, true);  // compute Jdotv
    }
    return kinsol_;
  }

 private:
  const RigidBodyTree<double>* tree_;
  VectorX<Scalar> last_q_;
  VectorX<Scalar> last_v_;
  KinematicsCache<Scalar> kinsol_;
};
}  // namespace plants
}  // namespace systems
}  // namespace drake

