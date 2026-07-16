#include "drake/multibody/tree/velocity_kinematics_cache.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
VelocityKinematicsCache<T>::VelocityKinematicsCache(
    const SpanningForest& forest)
    : num_mobods_(forest.num_mobods()), num_links_(forest.num_links()) {
  Allocate();

  // Set values that won't ever change.
  V_WB_pool_[world_mobod_index()].SetZero();   // World's velocity is zero.
  V_WL_pool_[world_link_ordinal()].SetZero();  //           "
  V_FM_pool_[world_mobod_index()].SetNaN();    // It must never be used.
  V_PB_W_pool_[world_mobod_index()].SetNaN();  // It must never be used.

  // Any links that are part of the World composite also have known zero
  // velocities in World.
  const SpanningForest::Mobod& world_mobod = forest.mobods(MobodIndex(0));
  const std::vector<LinkOrdinal>& world_followers =
      world_mobod.follower_link_ordinals();
  DRAKE_DEMAND(world_followers[0] == LinkOrdinal(0));  // Already done above.
  for (size_t i = 1; i < world_followers.size(); ++i) {
    const LinkOrdinal link_ordinal = world_followers[i];
    V_WL_pool_[link_ordinal].SetZero();
  }
}

template <typename T>
void VelocityKinematicsCache<T>::SetToZero() {
  for (MobodIndex mobod_index(0); mobod_index < num_mobods_; ++mobod_index) {
    V_WB_pool_[mobod_index].SetZero();
    V_FM_pool_[mobod_index].SetZero();
    V_PB_W_pool_[mobod_index].SetZero();
  }
  for (LinkOrdinal link_ordinal(0); link_ordinal < num_links_; ++link_ordinal) {
    V_WL_pool_[link_ordinal].SetZero();
  }
}

template <typename T>
void VelocityKinematicsCache<T>::Allocate() {
  V_WB_pool_.resize(num_mobods_, SpatialVelocity<T>::NaN());
  V_PB_W_pool_.resize(num_mobods_, SpatialVelocity<T>::NaN());
  V_WL_pool_.resize(num_links_, SpatialVelocity<T>::NaN());
  // Mobilizers are entitled to assume V_FM has been initialized to zero.
  // For example, a Weld mobilizer can simply not write to V_FM at all, and
  // others can take advantage of known zeroes.
  V_FM_pool_.resize(num_mobods_, SpatialVelocity<T>::Zero());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::VelocityKinematicsCache);
