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
is often needed as well.

Every Frame is allocated one slot here and the index of that slot is stored in
the Frame object for fast retrieval. Since RigidBodyFrames have identity poses
by definition, they all share a single entry (the 0th) here to permit getting
the body pose of any Frame efficiently in cases where you don't know what kind
of Frame you have. Of course if you know you are working with RigidBodyFrames
you don't have to use this cache.
@tparam_default_scalar */
template <typename T>
class FrameBodyPoseCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameBodyPoseCache);

  explicit FrameBodyPoseCache(int num_mobods, int num_frames,
                              int num_frame_body_pose_slots_needed)
      : X_BF_pool_(num_frame_body_pose_slots_needed),
        X_FB_pool_(num_frame_body_pose_slots_needed),
        X_MbF_pool_(num_frames),
        X_FMb_pool_(num_frames),
        is_X_MbF_identity_(num_frames),
        M_BMo_M_pool_(num_mobods, SpatialInertia<T>::NaN()) {
    DRAKE_DEMAND(num_frame_body_pose_slots_needed > 0);

    // All RigidBodyFrames share this body pose.
    X_BF_pool_[0] = X_FB_pool_[0] = math::RigidTransform<T>::Identity();
  }

  const math::RigidTransform<T>& get_X_BF(int body_pose_index) const {
    DRAKE_ASSERT(0 <= body_pose_index && body_pose_index < ssize(X_BF_pool_));
    return X_BF_pool_[body_pose_index];
  }

  const math::RigidTransform<T>& get_X_FB(int body_pose_index) const {
    DRAKE_ASSERT(0 <= body_pose_index && body_pose_index < ssize(X_FB_pool_));
    return X_FB_pool_[body_pose_index];
  }

  // Returns true for body frames.
  bool is_X_BF_identity(int body_pose_index) const {
    return body_pose_index == 0;
  }

  const math::RigidTransform<T>& get_X_MbF(FrameIndex index) const {
    DRAKE_ASSERT(0 <= index && index < ssize(X_MbF_pool_));
    return X_MbF_pool_[index];
  }

  const math::RigidTransform<T>& get_X_FMb(FrameIndex index) const {
    DRAKE_ASSERT(0 <= index && index < ssize(X_FMb_pool_));
    return X_FMb_pool_[index];
  }

  // Returns true for any frame F that is the outboard frame M for some
  // mobilizer, as well as any frame exactly coincident with one of those.
  bool is_X_MbF_identity(FrameIndex index) const {
    DRAKE_ASSERT(0 <= index && index < ssize(is_X_MbF_identity_));
    return static_cast<bool>(is_X_MbF_identity_[index]);
  }

  const SpatialInertia<T>& get_M_BMo_M(MobodIndex index) const {
    DRAKE_ASSERT(0 <= index && index < ssize(M_BMo_M_pool_));
    return M_BMo_M_pool_[index];
  }

  void set_X_BF(int body_pose_index, const math::RigidTransform<T>& X_BF) {
    DRAKE_DEMAND(0 <= body_pose_index && body_pose_index < ssize(X_BF_pool_));
    X_BF_pool_[body_pose_index] = X_BF;
    X_FB_pool_[body_pose_index] = X_BF.inverse();
  }

  void set_X_MbF(FrameIndex index, const math::RigidTransform<T>& X_MbF) {
    DRAKE_DEMAND(0 <= index && index < ssize(X_MbF_pool_));
    X_MbF_pool_[index] = X_MbF;
    X_FMb_pool_[index] = X_MbF.inverse();
    if constexpr (scalar_predicate<T>::is_bool) {
      is_X_MbF_identity_[index] = X_MbF.IsExactlyIdentity();
    } else {
      is_X_MbF_identity_[index] = static_cast<uint8_t>(false);
    }
  }

  void set_M_BMo_M(MobodIndex index, const SpatialInertia<T>& M_BMo_M) {
    DRAKE_DEMAND(0 <= index && index < ssize(M_BMo_M_pool_));
    M_BMo_M_pool_[index] = M_BMo_M;
  }

 private:
  // Sizes are set on construction.

  // These are indexed by Frame::get_body_pose_index_in_cache().
  std::vector<math::RigidTransform<T>> X_BF_pool_;
  std::vector<math::RigidTransform<T>> X_FB_pool_;

  // These are indexed by Frame::index().
  std::vector<math::RigidTransform<T>> X_MbF_pool_;
  std::vector<math::RigidTransform<T>> X_FMb_pool_;
  std::vector<uint8_t> is_X_MbF_identity_;  // fast vector<bool> equivalent

  // Spatial inertia of mobilized body B, about its inboard joint frame M's
  // origin Mo, expressed in M. These are indexed by MobodIndex.
  std::vector<SpatialInertia<T>> M_BMo_M_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::FrameBodyPoseCache);
