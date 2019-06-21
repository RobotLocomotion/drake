#pragma once

#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/drake_optional.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace find_collision_candidates {

/** Supporting data for the broadphase callback (see Callback below).
   It includes:

    - A map from GeometryIndex to GeometryId (to facilitate reporting GeometryId
      values in the results).
    - A collision filter instance.
    - A vector of geometry pairs -- each pair of geometries are possibly in
      contact.
 */
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the %CallbackData instance.

   @param geometry_map_in         The index -> id map. Aliased.
   @param collision_filter_in     The collision filter system. Aliased.
   @param pairs_in                The output results. Aliased.  */
  CallbackData(const std::vector<GeometryId>* geometry_map_in,
               const CollisionFilterLegacy* collision_filter_in,
               std::vector<SortedPair<GeometryId>>* pairs_in)
      : geometry_map(*geometry_map_in),
        collision_filter(*collision_filter_in),
        pairs(*pairs_in) {
    DRAKE_DEMAND(geometry_map_in);
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(pairs_in);
  }

  /** The map from GeometryIndex to GeometryId.  */
  const std::vector<GeometryId>& geometry_map;

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The results of the broadphase query.  */
  std::vector<SortedPair<GeometryId>>& pairs;
};

/** The callback function that stores the geometry ids of two shapes identified
 as potentially being in contact by the broad-phase.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data to find collision candidates.
 @returns False; the broadphase should *not* terminate its process.
  */
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data) {
  auto& data = *static_cast<CallbackData*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.encoding(), encoding_b.encoding());
  if (can_collide) {
    const GeometryId id_a = encoding_a.id(data.geometry_map);
    const GeometryId id_b = encoding_b.id(data.geometry_map);
    data.pairs.emplace_back(id_a, id_b);
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace find_collision_candidates
}  // namespace internal
}  // namespace geometry
}  // namespace drake
