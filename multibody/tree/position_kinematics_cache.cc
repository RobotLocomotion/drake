#include "drake/multibody/tree/position_kinematics_cache.h"

#include <limits>

#include "drake/multibody/tree/frame_body_pose_cache.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PositionKinematicsCache<T>::PositionKinematicsCache(
    const SpanningForest& forest)
    : num_mobods_(forest.num_mobods()), num_links_(forest.num_links()) {
  Allocate();

  // Set known values.
  X_WB_pool_[world_mobod_index()] = RigidTransform<T>::Identity();
  X_WL_pool_[world_link_ordinal()] = RigidTransform<T>::Identity();
  p_BoLo_W_pool_[world_link_ordinal()].setZero();
  // p_BoLo_W is always zero for each mobod's active link (since B=L₀).
  for (MobodIndex mobod_index(1); mobod_index < num_mobods_; ++mobod_index) {
    const LinkOrdinal active_link =
        forest.mobods(mobod_index).active_link_ordinal();
    p_BoLo_W_pool_[active_link].setZero();
  }
}

template <typename T>
void PositionKinematicsCache<T>::ComputeWorldComposite(
    const SpanningForest& forest,
    const FrameBodyPoseCache<T>& frame_body_pose_cache) {
  const SpanningForest::Mobod& world_mobod = forest.mobods(MobodIndex(0));
  const std::vector<LinkOrdinal>& world_followers =
      world_mobod.follower_link_ordinals();
  for (size_t i = 1; i < world_followers.size(); ++i) {
    const LinkOrdinal link_ordinal = world_followers[i];
    const math::RigidTransform<T>& X_WL =
        frame_body_pose_cache.get_X_BL(link_ordinal);  // B(=W) to link L
    SetX_WL(link_ordinal, X_WL);
    // For World's followers, B is the World mobod so R_WB is identity.
    // Thus p_BoLo_W = R_WB * p_BoLo_B = p_BoLo_B = X_BL.translation().
    Set_p_BoLo_W(link_ordinal, X_WL.translation());
  }
}

// Initialize most things to NaN to catch bugs.
template <typename T>
void PositionKinematicsCache<T>::Allocate() {
  const Vector3<T> nan_vec =
      Vector3<T>::Constant(std::numeric_limits<double>::quiet_NaN());
  X_WB_pool_.resize(num_mobods_, NaNPose());
  X_WL_pool_.resize(num_links_, NaNPose());
  p_BoLo_W_pool_.resize(num_links_, nan_vec);
  X_PB_pool_.resize(num_mobods_, NaNPose());
  // Mobilizers expect to be able to count on X_FM having been initialized
  // to the identity matrix. For example, Weld mobilizers just leave it that
  // way and never write to X_FM. Other mobilizers make use of the known
  // structure of the identity transform.
  X_FM_pool_.resize(num_mobods_, RigidTransform<T>::Identity());
  p_PoBo_W_pool_.resize(num_mobods_, nan_vec);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PositionKinematicsCache);
