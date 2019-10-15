#pragma once

#include <vector>

#include <fcl/fcl.h>

#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {

// TODO(SeanCurtis-TRI): Template this and include X_WGs when this query
//  supports AutoDiff scalars.
/** Supporting data for the detecting collision between geometries and reporting
 them as a pair of points (see PenetrationAsPointPair). It includes:

    - A collision filter instance.
    - An fcl collision request.
    - A vector of point pairs -- one instance of PenetrationAsPointPair for
      every supported, unfiltered penetrating pair.  */
struct CallbackData {
  CallbackData(
      const CollisionFilterLegacy* collision_filter_in,
      std::vector<PenetrationAsPointPair<double>>* point_pairs_in)
  : collision_filter(*collision_filter_in),
    point_pairs(*point_pairs_in) {
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(point_pairs_in);
    request.num_max_contacts = 1;
    request.enable_contact = true;
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

  /** The results of the collision query.  */
  std::vector<PenetrationAsPointPair<double>>& point_pairs;
};

// Callback function for FCL's collide() function for retrieving a *single*
// contact.
bool Callback(fcl::CollisionObjectd* fcl_object_A_ptr,
              fcl::CollisionObjectd* fcl_object_B_ptr, void* callback_data);

}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
