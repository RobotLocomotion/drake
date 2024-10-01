#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"

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

  explicit FrameBodyPoseCache(int num_frame_body_pose_slots_needed)
      : X_BF_pool_(num_frame_body_pose_slots_needed),
        X_FB_pool_(num_frame_body_pose_slots_needed) {
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

  void set_X_BF(int body_pose_index, const math::RigidTransform<T>& X_BF) {
    DRAKE_ASSERT(0 <= body_pose_index && body_pose_index < ssize(X_BF_pool_));
    X_BF_pool_[body_pose_index] = X_BF;
    X_FB_pool_[body_pose_index] = X_BF.inverse();
  }

 private:
  // Size is set on construction.
  std::vector<math::RigidTransform<T>> X_BF_pool_;
  std::vector<math::RigidTransform<T>> X_FB_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::FrameBodyPoseCache);
