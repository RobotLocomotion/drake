#include "drake/multibody/kinematics_cache_helper.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
namespace plants {
template <typename Scalar>
KinematicsCacheHelper<Scalar>::KinematicsCacheHelper(
    const RigidBodyTree<double>& tree)
    : last_tree_{nullptr},
      kinsol_(tree.CreateKinematicsCacheWithType<Scalar>()) {
  last_q_.resize(0);
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const VectorX<Scalar>>& q,
    const RigidBodyTree<double>* tree) {
  if ((q.size() != last_q_.size()) || (q != last_q_) || (last_tree_ != tree)) {
    last_q_ = q;
    last_tree_ = tree;
    kinsol_.initialize(q);
    tree->doKinematics(kinsol_);
  }
  return kinsol_;
}

template <typename Scalar>
KinematicsCacheWithVHelper<Scalar>::KinematicsCacheWithVHelper(
    const RigidBodyTree<double>& tree)
    : tree_{&tree}, kinsol_(tree.CreateKinematicsCacheWithType<Scalar>()) {
  last_q_.resize(0);
  last_v_.resize(0);
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheWithVHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const VectorX<Scalar>>& q) {
  if (last_v_.rows() == 0) {
    throw std::runtime_error("last_v_ is not set yet.");
  }
  return UpdateKinematics(q, last_v_);
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheWithVHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const VectorX<Scalar>>& q,
    const Eigen::Ref<const VectorX<Scalar>>& v) {
  if (q.size() != last_q_.size() || q != last_q_ ||
      v.size() != last_v_.size() || v != last_v_) {
    last_q_ = q;
    last_v_ = v;
    kinsol_.initialize(q, v);
    tree_->doKinematics(kinsol_, true);  // compute Jdotv
  }
  return kinsol_;
}

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheHelper<double>;
template class KinematicsCacheHelper<AutoDiffXd>;

template class KinematicsCacheWithVHelper<double>;
template class KinematicsCacheWithVHelper<AutoDiffXd>;
}  // namespace plants
}  // namespace systems
}  // namespace drake
