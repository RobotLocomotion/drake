#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace find_collision_candidates {

/** Supporting data for the collision candidates callback (see Callback below).
   It includes:

    - A collision filter instance.
    - A vector of geometry pairs -- each pair of geometries are possibly in
      contact.
 */
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the %CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param pairs_in                The output results. Aliased.  */
  CallbackData(const CollisionFilterLegacy* collision_filter_in,
               std::vector<SortedPair<GeometryId>>* pairs_in);

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The results of the collision candidates query.  */
  std::vector<SortedPair<GeometryId>>& pairs;
};

/** The callback function that stores the geometry ids of two shapes identified
 as potentially being in contact by the collision candidates query.

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
              void* callback_data);

}  // namespace find_collision_candidates
}  // namespace internal
}  // namespace geometry
}  // namespace drake
