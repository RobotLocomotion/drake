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

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheHelper<double>;
template class KinematicsCacheHelper<AutoDiffXd>;
}  // namespace plants
}  // namespace systems
}  // namespace drake
