#include "drake/multibody/kinematics_cache_helper.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
namespace plants {
template <typename Scalar>
KinematicsCacheHelper<Scalar>::KinematicsCacheHelper(
    const RigidBodyTree<double>& tree)
    : last_tree_{nullptr},
      kinematics_cache_(tree.CreateKinematicsCacheWithType<Scalar>()) {
  last_q_.resize(0);
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const VectorX<Scalar>>& q,
    const RigidBodyTree<double>* tree) {
  if ((q.size() != last_q_.size()) || (q != last_q_) || (last_tree_ != tree)) {
    last_q_ = q;
    last_tree_ = tree;
    kinematics_cache_.initialize(q);
    tree->doKinematics(kinematics_cache_);
  }
  return kinematics_cache_;
}

template <typename Scalar>
KinematicsCacheWithVHelper<Scalar>::KinematicsCacheWithVHelper(
    const RigidBodyTree<double>& tree)
    : tree_{&tree},
      kinematics_cache_(tree.CreateKinematicsCacheWithType<Scalar>()) {
  last_q_.resize(0);
  last_v_.resize(0);
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheWithVHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const VectorX<Scalar>>& q,
    const Eigen::Ref<const VectorX<Scalar>>& v) {
  if (q.size() != last_q_.size() || q != last_q_ ||
      v.size() != last_v_.size() || v != last_v_) {
    last_q_ = q;
    last_v_ = v;
    kinematics_cache_.initialize(q, v);
    tree_->doKinematics(kinematics_cache_, true);  // compute Jdotv
  }
  return kinematics_cache_;
}

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheHelper<double>;
template class KinematicsCacheHelper<AutoDiffXd>;

template class KinematicsCacheWithVHelper<double>;
template class KinematicsCacheWithVHelper<AutoDiffXd>;
}  // namespace plants
}  // namespace systems
}  // namespace drake
