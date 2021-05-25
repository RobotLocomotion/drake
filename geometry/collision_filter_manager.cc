#include "drake/geometry/collision_filter_manager.h"

#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

template <typename T>
CollisionFilterManager<T>::CollisionFilterManager(GeometryState<T>* state)
    : state_(state) {
  DRAKE_DEMAND(state != nullptr);
}

template <typename T>
void CollisionFilterManager<T>::ExcludeCollisionsWithin(
    const GeometrySet& set) {
  state_->ExcludeCollisionsWithin(set);
}

template <typename T>
void CollisionFilterManager<T>::ExcludeCollisionsBetween(
    const GeometrySet& setA, const GeometrySet& setB) {
  state_->ExcludeCollisionsBetween(setA, setB);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::CollisionFilterManager)
