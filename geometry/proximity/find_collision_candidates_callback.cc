#include "drake/geometry/proximity/find_collision_candidates_callback.h"

#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace find_collision_candidates {

CallbackData::CallbackData(const CollisionFilter* collision_filter_in,
                           std::vector<SortedPair<GeometryId>>* pairs_in)
    : collision_filter(*collision_filter_in),
      pairs(*pairs_in) {
  DRAKE_DEMAND(collision_filter_in != nullptr);
  DRAKE_DEMAND(pairs_in != nullptr);
}

bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data) {
  auto& data = *static_cast<CallbackData*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.id(), encoding_b.id());
  if (can_collide) {
    data.pairs.emplace_back(encoding_a.id(), encoding_b.id());
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace find_collision_candidates
}  // namespace internal
}  // namespace geometry
}  // namespace drake
