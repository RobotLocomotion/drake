#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {

/* This class is one of the cache entries in the Context. It holds the
precalculated body-relative poses X_BF of every Frame F. Since FixedOffsetFrame
body poses are parameterized, and given with respect to a parent frame P which
may itself be a parameterized FixedOffsetFrame, we need to precalculate X_BF
once the parameters have been set so that we don't have to do that calculation
repeatedly at runtime. We also precalculate the inverse X_FB since that
is often needed as well, and record whether X_BF (and of course X_FB) is the
identity transform, for use in runtime optimizations.

Every Frame is allocated one slot here and the index of that slot (which we
refer to as `body_pose_index` in this class) is stored in the Frame object for
fast retrieval (the indices are assigned in Finalize()). Since RigidBodyFrames
have identity poses by definition, they all share a single entry (the 0th) here
to permit getting the body pose of any Frame efficiently in cases where you
don't know what kind of Frame you have. Of course if you know you are working
with RigidBodyFrames you don't have to use this cache.
@tparam_default_scalar */
template <typename T>
class FrameBodyPoseCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameBodyPoseCache);

  explicit FrameBodyPoseCache(int num_mobods,
                              int num_frame_body_pose_slots_needed)
      : X_BF_pool_(num_frame_body_pose_slots_needed),
        X_FB_pool_(num_frame_body_pose_slots_needed),
        is_X_BF_identity_(num_frame_body_pose_slots_needed),
        M_BBo_B_pool_(num_mobods, SpatialInertia<T>::NaN()) {
    DRAKE_DEMAND(num_frame_body_pose_slots_needed > 0);

    // All RigidBodyFrames share this body pose.
    X_BF_pool_[0] = X_FB_pool_[0] = math::RigidTransform<T>::Identity();
    is_X_BF_identity_[0] = true;
  }

  const math::RigidTransform<T>& get_X_BF(int body_pose_index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= body_pose_index && body_pose_index < ssize(X_BF_pool_));
    return X_BF_pool_[body_pose_index];
  }

  const math::RigidTransform<T>& get_X_FB(int body_pose_index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= body_pose_index && body_pose_index < ssize(X_FB_pool_));
    return X_FB_pool_[body_pose_index];
  }

  // Returns true if F is a body frame or is coincident with a body frame,
  // unless T is symbolic, in which case we always return false. This should be
  // used only for performance optimization, so that a false negative
  // harmlessly leads to treating X_BF as a general transform.
  bool is_X_BF_identity(int body_pose_index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= body_pose_index &&
                 body_pose_index < ssize(is_X_BF_identity_));
    return static_cast<bool>(is_X_BF_identity_[body_pose_index]);
  }

  const SpatialInertia<T>& get_M_BBo_B(MobodIndex index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= index && index < ssize(M_BBo_B_pool_));
    return M_BBo_B_pool_[index];
  }

  void SetX_BF(int body_pose_index, const math::RigidTransform<T>& X_BF) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= body_pose_index && body_pose_index < ssize(X_BF_pool_));
    // RigidBodyFrames use pose index 0; we already know X_BF is identity.
    if (body_pose_index == 0) return;
    X_BF_pool_[body_pose_index] = X_BF;
    X_FB_pool_[body_pose_index] = X_BF.inverse();
    if constexpr (scalar_predicate<T>::is_bool) {
      is_X_BF_identity_[body_pose_index] = X_BF.IsExactlyIdentity();
    } else {
      is_X_BF_identity_[body_pose_index] = static_cast<uint8_t>(false);
    }
  }

  void SetM_BBo_B(MobodIndex index, const SpatialInertia<T>& M_BBo_B) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= index && index < ssize(M_BBo_B_pool_));
    M_BBo_B_pool_[index] = M_BBo_B;
  }

 private:
  // Sizes are set on construction.

  // These are indexed by Frame::get_body_pose_index_in_cache().
  std::vector<math::RigidTransform<T>> X_BF_pool_;
  std::vector<math::RigidTransform<T>> X_FB_pool_;
  std::vector<uint8_t> is_X_BF_identity_;  // fast vector<bool> equivalent

  // Spatial inertia of mobilized body B, about its body origin Bo, expressed
  // in B. These are indexed by MobodIndex.
  std::vector<SpatialInertia<T>> M_BBo_B_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::FrameBodyPoseCache);
