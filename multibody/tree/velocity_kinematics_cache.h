#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Should cache qdot here.

/* This class is one of the cache entries in the Context. It holds the
kinematics results of computations that depend not only on the generalized
positions of the system, but also on its generalized velocities.

Velocity kinematics results are mostly per-Mobod. We also need to know the
velocity of every Link (composite mobods carry multiple links). Note that
every mobod B (composite or not) has a distinguished "active" link L₀, which
is the most inboard link following that mobod (that is, the link with the
joint that connects the mobod to its inboard mobod). The body frame B of
a mobod is always coincident with the frame L₀ of its active link.

- V_WB:   Spatial velocity of mobod B measured and expressed in the world
          frame W. Frame B is the same as frame L₀ of a mobod's active link.
- V_WL:   Spatial velocity link L in W for every link. Indexed by
          LinkOrdinal. Same as V_WB if L is the active link of mobod B.
- V_PB_W: Spatial velocity of mobod B measured in its parent (inboard)
          mobod's frame P, expressed in W.
- V_FM:   For a mobod's mobilizer, the spatial velocity of the mobilizer's
          outboard frame M (on the mobod) in its inboard frame F (on the mobod's
          inboard ("parent") mobod), expressed in F.

@tparam_default_scalar */

// TODO(sherm1) V_WL mostly duplicates V_WB (they are identical if there are no
//  composite bodies), and always contains all the V_WB spatial velocities (for
//  the active links). See position_kinematics_cache for how to avoid
//  duplication with no overhead.
template <typename T>
class VelocityKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VelocityKinematicsCache);

  // Constructs a velocity kinematics cache entry for the given
  // SpanningForest.
  // In Release builds specific entries are left uninitialized resulting in a
  // zero cost operation. However, in Debug builds those entries are set to NaN
  // so that operations using this uninitialized cache entry fail fast, easing
  // bug detection.
  explicit VelocityKinematicsCache(const SpanningForest& forest)
      : num_mobods_(forest.num_mobods()), num_links_(forest.num_links()) {
    Allocate();
    DRAKE_ASSERT_VOID(InitializeToNaN());
    // Sets defaults.
    V_WB_pool_[world_mobod_index()].SetZero();   // World's velocity is zero.
    V_WL_pool_[world_link_ordinal()].SetZero();  //           "
    V_FM_pool_[world_mobod_index()].SetNaN();    // It must never be used.
    V_PB_W_pool_[world_mobod_index()].SetNaN();  // It must never be used.
  }

  // Initializes `this` %VelocityKinematicsCache as if all generalized
  // velocities of the corresponding MultibodyTree model were zero.
  void InitializeToZero() {
    for (MobodIndex mobod_index(0); mobod_index < num_mobods_; ++mobod_index) {
      V_WB_pool_[mobod_index].SetZero();
      V_FM_pool_[mobod_index].SetZero();
      V_PB_W_pool_[mobod_index].SetZero();
    }
    for (LinkOrdinal link_ordinal(0); link_ordinal < num_links_;
         ++link_ordinal) {
      V_WL_pool_[link_ordinal].SetZero();
    }
  }

  // Returns V_WB, body B's spatial velocity in the world frame W.
  // @param[in] mobod_index The unique index for the computational
  //                        BodyNode object associated with body B.
  // @retval V_WB_W body B's spatial velocity in the world frame W,
  // expressed in W (for point Bo, the body frame's origin).
  const SpatialVelocity<T>& get_V_WB(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_WB_pool_[mobod_index];
  }

  // Mutable version of get_V_WB().
  SpatialVelocity<T>& get_mutable_V_WB(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_WB_pool_[mobod_index];
  }

  const SpatialVelocity<T>& get_V_WL(LinkOrdinal ordinal) const {
    DRAKE_ASSERT(0 <= ordinal && ordinal < num_links_);
    return V_WL_pool_[ordinal];
  }

  void SetV_WL(LinkOrdinal ordinal, const SpatialVelocity<T>& V_WL) {
    DRAKE_DEMAND(0 <= ordinal && ordinal < num_links_);
    V_WL_pool_[ordinal] = V_WL;
  }

  // Returns a const reference to the across-mobilizer (associated with
  // mobilized body `mobod_index`) spatial velocity `V_FM` of the outboard frame
  // M in the inboard frame F.
  const SpatialVelocity<T>& get_V_FM(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_FM_pool_[mobod_index];
  }

  // Mutable version of get_V_FM().
  SpatialVelocity<T>& get_mutable_V_FM(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_FM_pool_[mobod_index];
  }

  // Returns a const reference to the spatial velocity `V_PB_W` of the
  // body B associated with mobilized body `mobod_index` in the parent body's
  // frame P, expressed in the world frame W.
  const SpatialVelocity<T>& get_V_PB_W(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_PB_W_pool_[mobod_index];
  }

  // Mutable version of get_V_PB_W().
  SpatialVelocity<T>& get_mutable_V_PB_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return V_PB_W_pool_[mobod_index];
  }

 private:
  // Allocates resources for this velocity kinematics cache.
  void Allocate() {
    V_WB_pool_.resize(num_mobods_);
    V_FM_pool_.resize(num_mobods_);
    V_PB_W_pool_.resize(num_mobods_);
    V_WL_pool_.resize(num_links_);
  }

  // Initializes all pools to have NaN values to ease bug detection when entries
  // are accidentally left uninitialized.
  void InitializeToNaN() {
    for (MobodIndex mobod_index(0); mobod_index < num_mobods_; ++mobod_index) {
      V_WB_pool_[mobod_index].SetNaN();
      V_FM_pool_[mobod_index].SetNaN();
      V_PB_W_pool_[mobod_index].SetNaN();
    }
    for (LinkOrdinal link_ordinal(0); link_ordinal < num_links_;
         ++link_ordinal) {
      V_WL_pool_[link_ordinal].SetNaN();
    }
  }

  // Number of Mobods in the multibody forest, including the World mobod.
  int num_mobods_{0};

  // Number of links in the multibody graph includes all user-defined links
  // (including the World link) and possibly some ephemeral links.
  int num_links_{0};

  // These are indexed by MobodIndex so are in depth-first order.
  std::vector<SpatialVelocity<T>> V_WB_pool_;
  std::vector<SpatialVelocity<T>> V_FM_pool_;
  std::vector<SpatialVelocity<T>> V_PB_W_pool_;

  // This pool is indexed by LinkOrdinal.
  std::vector<SpatialVelocity<T>> V_WL_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::VelocityKinematicsCache);
