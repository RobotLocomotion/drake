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
  KinematicsCache<Scalar> kinsol_;
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

  /**
   * Only updates the position q, but leave velocity v unchanged as the last
   * velocity passed in.
   * @throw a runtime_error if last_v_ is unset.
   * Note that by updating only q, KinematicsCacheWithVHelper behaves similarly
   * to KinematicsCachHelper, without updating v. The reason we provide this
   * functionality is that it is possible that we might need to evaluate
   * some kinematic result both with and without v, for example in manipulator
   * equation
   * Mv̇+c = Bu + Jᵀλ
   * The coriolis term c requires kinematics information with both q and v,
   * while the Jacobian J might only need the position q. To avoid providing
   * two helpers for this evaluator, one with v and one without, we can choose
   * to provide a single helper with both q and v, and update only q when
   * computing the Jacobian J. This trick avoids the redundant computation for
   * calling doKinematics for twice, if we were to use two kinematics cache
   * helpers, one with v and the other without.
   */
  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const VectorX<Scalar>>& q,
      const Eigen::Ref<const VectorX<Scalar>>& v);

 private:
  const RigidBodyTree<double>* tree_;
  VectorX<Scalar> last_q_;
  VectorX<Scalar> last_v_;
  KinematicsCache<Scalar> kinsol_;
};
}  // namespace plants
}  // namespace systems
}  // namespace drake
