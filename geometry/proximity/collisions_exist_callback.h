#pragma once

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {

/** Supporting data for the collisions exist callback (see Callback below).
   It includes:
    - A collision filter instance.
    - A boolean of whether any collisions exist.
 */
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.  */
  explicit CallbackData(const CollisionFilterLegacy* collision_filter_in);

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The parameters for the fcl object-object collision function.  */
  fcl::CollisionRequestd request;

  /** The result of the collisions exist query.  */
  bool collisions_exist{false};
};

/** The callback function for computing if there is a collision.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data for whether collisions exist.
 @returns true if there is a collision so that the broadphase can terminate its
          process early.
  */
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              void* callback_data);

}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
