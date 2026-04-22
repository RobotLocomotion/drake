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

/* This class is one of the cache entries in the Context. It can be filled in
once parameters have known values (and must be recalculated when parameters
change). It holds the following items.

Frame & Link poses
------------------
 - the link-relative pose X_LF of every Frame F on the Link to which it is
   fixed. FixedOffsetFrame link poses are parameterized, and given with
   respect to a parent frame P which may itself be a parameterized
   FixedOffsetFrame. We need to precalculate X_LF so that we don't have to do
   that calculation repeatedly at runtime.
 - the pose X_BL of each link's frame L on its Mobod frame B. Note that frame B
   is always the link frame of the mobod's active (most inboard) link. X_BL is
   necessarily identity unless B is a composite mobod and L is not the active
   link.
 - since a frame is fixed to its link L, and L is fixed to its mobod B, we
   can calculate each frame's mobod-relative pose X_BF (= X_BL*X_LF). This
   can only differ from X_LF when mobod B is composite and L is not the active
   link of B.
 - the inverse X_FB since that is often needed as well.
 - whether X_BF (and of course X_FB) is the identity transform, for use in
   runtime optimizations.

Mass properties
---------------
 - the SpatialInertia M_LLo_L of every link L about its link origin Lo,
   expressed in L. Since mass properties can be parameterized, we need to
   precalculate these inertias so that we don't have to do that repeatedly at
   runtime.
 - the SpatialInertia M_BBo_B of every mobod B about its body origin Bo,
   expressed in B. This differs from M_LLo_L when B is a composite mobod.

@tparam_default_scalar */
template <typename T>
class FrameBodyPoseCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameBodyPoseCache);

  explicit FrameBodyPoseCache(int num_links, int num_frames, int num_mobods)
      : X_LF_pool_(num_frames, math::RigidTransform<T>::Identity()),
        X_BF_pool_(num_frames, math::RigidTransform<T>::Identity()),
        X_FB_pool_(num_frames, math::RigidTransform<T>::Identity()),
        is_X_BF_identity_(num_frames, true),
        X_BL_pool_(num_links, math::RigidTransform<T>::Identity()),
        is_X_BL_identity_(num_links, true),
        M_LLo_L_pool_(num_links, SpatialInertia<T>::NaN()),
        M_BBo_B_pool_(num_mobods, SpatialInertia<T>::NaN()) {
    // Initially all transforms are identity, mass props are NaN.
  }

  const math::RigidTransform<T>& get_X_LF(FrameIndex index) const {
    DRAKE_ASSERT(0 <= index && index < ssize(X_LF_pool_));
    return X_LF_pool_[index];
  }

  const math::RigidTransform<T>& get_X_BF(FrameIndex index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= index && index < ssize(X_BF_pool_));
    return X_BF_pool_[index];
  }

  const math::RigidTransform<T>& get_X_FB(FrameIndex index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= index && index < ssize(X_FB_pool_));
    return X_FB_pool_[index];
  }

  // We're given a frame F that is fixed to some link L, and L is fixed to
  // some mobod B. Denote B's active link as L₀. By definition, L₀'s
  // LinkFrame is also mobilized body B's body frame.
  //
  // This method returns true if
  // (1) F is B's body frame (that is, F is L's LinkFrame and L≡L₀), or
  // (2) T is nonsymbolic and F is currently coincident B's body frame.
  //
  // This should be used only for performance optimization, so that a false
  // negative harmlessly leads to treating X_BF as a general transform.
  bool is_X_BF_identity(FrameIndex index) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= index && index < ssize(is_X_BF_identity_));
    return static_cast<bool>(is_X_BF_identity_[index]);
  }

  const math::RigidTransform<T>& get_X_BL(LinkOrdinal ordinal) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= ordinal && ordinal < ssize(X_BL_pool_));
    return X_BL_pool_[ordinal];
  }

  bool is_X_BL_identity(LinkOrdinal ordinal) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= ordinal && ordinal < ssize(is_X_BL_identity_));
    return static_cast<bool>(is_X_BL_identity_[ordinal]);
  }

  const SpatialInertia<T>& get_M_LLo_L(LinkOrdinal ordinal) const {
    DRAKE_ASSERT(0 <= ordinal && ordinal < ssize(M_LLo_L_pool_));
    return M_LLo_L_pool_[ordinal];
  }

  const SpatialInertia<T>& get_M_BBo_B(MobodIndex ordinal) const {
    // This method must be very fast in Release.
    DRAKE_ASSERT(0 <= ordinal && ordinal < ssize(M_BBo_B_pool_));
    return M_BBo_B_pool_[ordinal];
  }

  void SetX_LF(FrameIndex index, const math::RigidTransform<T>& X_LF) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= index && index < ssize(X_LF_pool_));
    X_LF_pool_[index] = X_LF;
  }

  void SetX_BF(FrameIndex index, const math::RigidTransform<T>& X_BF) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= index && index < ssize(X_BF_pool_));
    X_BF_pool_[index] = X_BF;
    X_FB_pool_[index] = X_BF.inverse();
    if constexpr (scalar_predicate<T>::is_bool) {
      is_X_BF_identity_[index] = X_BF.IsExactlyIdentity();
    } else {
      is_X_BF_identity_[index] = static_cast<uint8_t>(false);
    }
  }

  void SetX_BL(LinkOrdinal ordinal, const math::RigidTransform<T>& X_BL) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= ordinal && ordinal < ssize(X_BL_pool_));
    X_BL_pool_[ordinal] = X_BL;
    if constexpr (scalar_predicate<T>::is_bool) {
      is_X_BL_identity_[ordinal] = X_BL.IsExactlyIdentity();
    } else {
      is_X_BL_identity_[ordinal] = static_cast<uint8_t>(false);
    }
  }

  void SetM_LLo_L(LinkOrdinal ordinal, const SpatialInertia<T>& M_LLo_L) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= ordinal && ordinal < ssize(M_LLo_L_pool_));
    M_LLo_L_pool_[ordinal] = M_LLo_L;
  }

  void SetM_BBo_B(MobodIndex index, const SpatialInertia<T>& M_BBo_B) {
    // This method is only called when parameters change.
    DRAKE_DEMAND(0 <= index && index < ssize(M_BBo_B_pool_));
    M_BBo_B_pool_[index] = M_BBo_B;
  }

 private:
  // Sizes are set on construction.

  // These are indexed by FrameIndex.
  std::vector<math::RigidTransform<T>> X_LF_pool_;
  std::vector<math::RigidTransform<T>> X_BF_pool_;
  std::vector<math::RigidTransform<T>> X_FB_pool_;
  std::vector<uint8_t> is_X_BF_identity_;  // fast vector<bool> equivalent

  // These are indexed by LinkOrdinal.
  std::vector<math::RigidTransform<T>> X_BL_pool_;
  std::vector<uint8_t> is_X_BL_identity_;
  std::vector<SpatialInertia<T>> M_LLo_L_pool_;

  // This is indexed by MobodIndex.
  std::vector<SpatialInertia<T>> M_BBo_B_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::FrameBodyPoseCache);
