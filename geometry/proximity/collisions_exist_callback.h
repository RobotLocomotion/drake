#pragma once

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"

namespace drake {
namespace geometry {
namespace internal {
namespace collisions_exist {

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
  explicit CallbackData(const CollisionFilterLegacy* collision_filter_in)
      : collision_filter(*collision_filter_in), exist(false) {
    DRAKE_DEMAND(collision_filter_in);
    request.num_max_contacts = 1;
    request.enable_contact = false;
    // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
    // superior to FCL's "independent" implementation. Furthermore, libccd
    // appears to behave badly if its gjk tolerance is much tighter than
    // 2e-12. Until this changes, we explicitly specify these parameters rather
    // than relying on FCL's defaults.
    request.gjk_tolerance = 2e-12;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  }

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The parameters for the fcl object-object collision function.  */
  fcl::CollisionRequestd request;

  /** The result of the collisions exist query.  */
  bool exist;
};

/** The callback function for computing if there is a collision.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data for whether collisions exist.
 @returns true if there is a collision so that the broadphase can terminate its
          process early, otherwise false.
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
  if (!can_collide) return false;

  // Unpack the callback data
  const fcl::CollisionRequestd& request = data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  fcl::CollisionResultd result;

  // Perform nearphase collision detection
  fcl::collide(object_A_ptr, object_B_ptr, request, result);

  data.exist = result.isCollision();
  return data.exist;
}

}  // namespace collisions_exist
}  // namespace internal
}  // namespace geometry
}  // namespace drake
