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
void CollisionFilterManager<T>::Apply(
    const CollisionFilterDeclaration& declaration) {
  state_->ApplyFilterDeclaration(declaration);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::CollisionFilterManager)
