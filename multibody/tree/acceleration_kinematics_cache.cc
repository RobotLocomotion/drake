#include "drake/multibody/tree/acceleration_kinematics_cache.h"

#include <limits>

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
AccelerationKinematicsCache<T>::AccelerationKinematicsCache(
    const SpanningForest& forest) {
  Allocate(forest);

  // World's acceleration is always zero.
  A_WB_pool_[MobodIndex(0)].SetZero();
  A_WL_pool_[LinkOrdinal(0)].SetZero();

  // Any links that are part of the World composite also have known zero
  // accelerations.
  const SpanningForest::Mobod& world_mobod = forest.mobods(MobodIndex(0));
  const std::vector<LinkOrdinal>& world_followers =
      world_mobod.follower_link_ordinals();
  DRAKE_DEMAND(world_followers[0] == LinkOrdinal(0));  // Already done above.
  for (size_t i = 1; i < world_followers.size(); ++i) {
    const LinkOrdinal link_ordinal = world_followers[i];
    A_WL_pool_[link_ordinal].SetZero();
  }
}

template <typename T>
void AccelerationKinematicsCache<T>::SetToZero() {
  for (SpatialAcceleration<T>& acc : A_WB_pool_) acc.SetZero();
  for (SpatialAcceleration<T>& acc : A_WL_pool_) acc.SetZero();
  vdot_.setZero();
}

template <typename T>
void AccelerationKinematicsCache<T>::Allocate(const SpanningForest& forest) {
  A_WB_pool_.resize(forest.num_mobods(), SpatialAcceleration<T>::NaN());
  A_WL_pool_.resize(forest.num_links(), SpatialAcceleration<T>::NaN());
  vdot_ = VectorX<T>::Constant(forest.num_velocities(),
                               std::numeric_limits<double>::quiet_NaN());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::AccelerationKinematicsCache);
