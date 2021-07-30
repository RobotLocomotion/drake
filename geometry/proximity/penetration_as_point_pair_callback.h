#pragma once

#include <unordered_map>
#include <vector>

#include <fcl/fcl.h>

#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {

/* Supporting data for the detecting collision between geometries and reporting
 them as a pair of points (see PenetrationAsPointPair). It includes:

    - A collision filter instance. Aliased.
    - An fcl collision request. Aliased.
    - The poses. Aliased.
    - A vector of point pairs -- one instance of PenetrationAsPointPair for
      every supported, unfiltered penetrating pair. Aliased. */
template <typename T>
struct CallbackData {
  CallbackData(
      const CollisionFilter* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      std::vector<PenetrationAsPointPair<T>>* point_pairs_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        point_pairs(*point_pairs_in) {
    DRAKE_DEMAND(collision_filter_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(point_pairs_in != nullptr);
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

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The parameters for the fcl object-object collision function.  */
  fcl::CollisionRequestd request;

  /** The pose of each geometry in the scene. */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The results of the collision query.  */
  std::vector<PenetrationAsPointPair<T>>& point_pairs;
};

/* Callback function for FCL's collide() function for retrieving a *single*
 contact. As documented by QueryObject::ComputePointPairPenetration(), the
 result added to the output data is the same, regardless of the order of
 the two fcl objects.  */
template <typename T>
bool Callback(fcl::CollisionObjectd* fcl_object_A_ptr,
              fcl::CollisionObjectd* fcl_object_B_ptr, void* callback_data);

}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
