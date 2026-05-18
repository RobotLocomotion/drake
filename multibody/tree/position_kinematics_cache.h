#pragma once

#include <limits>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* This class is one of the cache entries in the Context. It holds the
kinematics results of computations that only depend on the generalized
positions q of the system.

Position kinematics results are mostly per-Mobod. We also need to know the pose
of every Link (composite mobods carry multiple links). Note that every mobod B
(composite or not) has a distinguished "active" link L₀, which is the most
inboard link following that mobod (that is, the link with the joint that
connects the mobod to its inboard mobod). The body frame B of a mobod is always
coincident with the frame L₀ of its active link.

Results are indexed by MobodIndex unless otherwise specified:
 - X_WB: Pose of mobod B measured and expressed in the world frame W.
         Frame B is the same as frame L₀ of a mobod's active link.
 - X_WL: Pose of link L in W for every link. Indexed by LinkOrdinal. Same as
         X_WB if L is the active link of mobod B.
 - X_PB: Pose of mobod B measured and expressed in its parent (inboard) mobod's
         frame P.
 - p_PoBo_W:
         Position of mobod B's origin Bo measured in its parent (inboard) mobod
         P, expressed in world frame W.
 - X_FM: Pose of mobilizer's outboard frame M measured and expressed in
         its inboard frame F.
 - H_FM: Mobilizer's hinge matrix, the Jacobian ∂V_FM/∂v that maps the
         mobilizer's generalized velocities v to cross-mobilizer spatial
         velocities V_FM = H_FM * v.

@tparam_default_scalar */

// TODO(sherm1) X_WL mostly duplicates X_WB (they are identical if there are no
//  composite bodies), and always contains all the X_WB transforms (for the
//  active links). Claude's idea to avoid duplication with no overhead: assign
//  active link ordinals first, so that
//  LinkOrdinal(mobod.active_link) == MobodIndex(mobod). Then
//  X_WL(mobod.index()) == X_WB(mobod.index()) avoiding indirection.
//  Applies to the velocity_kinematics_cache also.

// TODO(sherm1) Currently we never form composites so each Mobod has only
//  its active link L₀.
template <typename T>
class PositionKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PositionKinematicsCache);

  template <typename U>
  using RigidTransform = drake::math::RigidTransform<U>;

  explicit PositionKinematicsCache(const SpanningForest& forest)
      : num_mobods_(forest.num_mobods()), num_links_(forest.num_links()) {
    Allocate();
  }

  // Returns a const reference to pose `X_WB` of the body B (associated with
  // mobilized body mobod_index) as measured and expressed in the world frame W.
  // @param[in] mobod_index The unique index for the computational
  //                        BodyNode object associated with body B.

  // @returns `X_WB` the pose of the body frame B measured and expressed in
  //                 the world frame W.
  const RigidTransform<T>& get_X_WB(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_WB_pool_[mobod_index];
  }

  // See documentation on the const version get_X_WB() for details.
  RigidTransform<T>& get_mutable_X_WB(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_WB_pool_[mobod_index];
  }

  const RigidTransform<T>& get_X_WL(LinkOrdinal ordinal) const {
    DRAKE_ASSERT(0 <= ordinal && ordinal < num_links_);
    return X_WL_pool_[ordinal];
  }

  void SetX_WL(LinkOrdinal ordinal, const math::RigidTransform<T>& X_WL) {
    DRAKE_DEMAND(0 <= ordinal && ordinal < num_links_);
    X_WL_pool_[ordinal] = X_WL;
  }

  // Returns a const reference to the rotation matrix `R_WB` that relates the
  // orientation of the world frame W with the body frame B.
  // @param[in] mobod_index The unique index for the computational
  //                        BodyNode object associated with body B.
  const math::RotationMatrix<T>& get_R_WB(MobodIndex mobod_index) const {
    const RigidTransform<T>& X_WB = get_X_WB(mobod_index);
    return X_WB.rotation();
  }

  // Returns a const reference to the pose `X_PB` of the body frame B
  // as measured and expressed in its parent body frame P.
  // @param[in] mobod_index The unique identifier for the computational
  //                        BodyNode object associated with body B.
  // @returns `X_PB` a const reference to the pose of the body frame B
  //                 measured and expressed in the parent body frame P.
  const RigidTransform<T>& get_X_PB(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_PB_pool_[mobod_index];
  }

  // See documentation on the const version get_X_PB() for details.
  RigidTransform<T>& get_mutable_X_PB(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_PB_pool_[mobod_index];
  }

  // For the mobilizer associated with the body node indexed by
  // `mobod_index`, this method returns a const reference to the pose
  // `X_FM` of the outboard frame M as measured and expressed in the inboard
  // frame F.
  //
  // @param[in] mobod_index The unique index for the computational BodyNode
  //                        object associated with the mobilizer of interest.
  // @returns A const reference to the pose `X_FM` of the outboard frame M
  //          as measured and expressed in the inboard frame F.
  const RigidTransform<T>& get_X_FM(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_FM_pool_[mobod_index];
  }

  // See documentation on the const version get_X_FM() for details.
  RigidTransform<T>& get_mutable_X_FM(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return X_FM_pool_[mobod_index];
  }

  // Position of node B, with index `mobod_index`, measured in the inboard
  // body frame P, expressed in the world frame W.
  const Vector3<T>& get_p_PoBo_W(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return p_PoBo_W_pool_[mobod_index];
  }

  // Mutable version of get_p_PoBo_W().
  Vector3<T>& get_mutable_p_PoBo_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return p_PoBo_W_pool_[mobod_index];
  }

 private:
  // Allocates resources for this position kinematics cache.
  void Allocate() {
    X_WB_pool_.resize(num_mobods_);
    // Even though RigidTransform defaults to identity, we make it explicit.
    // This pose will never change after this initialization.
    X_WB_pool_[world_mobod_index()] = RigidTransform<T>::Identity();

    X_WL_pool_.resize(num_links_);
    X_WL_pool_[world_link_ordinal()] = RigidTransform<T>::Identity();

    X_PB_pool_.resize(num_mobods_);
    X_PB_pool_[world_mobod_index()] = NaNPose();  // It should never be used.

    X_FM_pool_.resize(num_mobods_);
    X_FM_pool_[world_mobod_index()] = NaNPose();  // It should never be used.

    p_PoBo_W_pool_.resize(num_mobods_);
    // p_PoBo_W for the world body should never be used.
    p_PoBo_W_pool_[world_mobod_index()].setConstant(
        std::numeric_limits<
            typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  // Helper method to initialize poses to garbage values including NaNs.
  // This allow us to quickly verify some of the values stored in the pools are
  // never used (however we store them anyway to simplify the indexing).
  static RigidTransform<T> NaNPose() {
    // Note: RotationMatrix will throw in Debug builds if values are NaN. For
    // our purposes, it is enough the translation has NaN values.
    return RigidTransform<T>(
        math::RotationMatrix<T>::Identity(),
        Vector3<T>::Constant(Eigen::NumTraits<double>::quiet_NaN()));
  }

  // Number of Mobods in the multibody forest, including the World mobod.
  int num_mobods_{0};

  // Number of links in the multibody graph includes all user-defined links
  // (including the World link) and possibly some ephemeral links.
  int num_links_{0};

  // These are indexed by MobodIndex so are in depth-first order.
  std::vector<RigidTransform<T>> X_WB_pool_;
  std::vector<RigidTransform<T>> X_PB_pool_;
  std::vector<RigidTransform<T>> X_FM_pool_;
  std::vector<Vector3<T>> p_PoBo_W_pool_;

  // This pool is indexed by LinkOrdinal.
  std::vector<RigidTransform<T>> X_WL_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PositionKinematicsCache);
