#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_export.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair DRAKE_NO_EXPORT {

/* Supporting data for the detecting collision between geometries and reporting
 them as a pair of points (see PenetrationAsPointPair). It includes:

    - A collision filter instance. Aliased.
    - A collision request. Aliased.
    - The poses. Aliased.
    - A vector of point pairs -- one instance of PenetrationAsPointPair for
      every supported, unfiltered penetrating pair. Aliased. */
template <typename T, typename Backend>
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
    Backend::ConfigurePenetrationRequest(&request);
  }

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The parameters for the backend's object-object collision function.  */
  typename Backend::CollisionRequest request;

  /** The pose of each geometry in the scene. */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The results of the collision query.  */
  std::vector<PenetrationAsPointPair<T>>& point_pairs;
};

/* Callback function for the backend's collide() function for retrieving a
 *single* contact. As documented by
 QueryObject::ComputePointPairPenetration(), the result added to the output
 data is the same, regardless of the order of the two objects.  */
template <typename T, typename Backend>
bool Callback(typename Backend::Object* object_A_ptr,
              typename Backend::Object* object_B_ptr, void* callback_data);

/* Given two objects that are candidates for a collision, returns the
 point-pair contact result. If the penetration depth turns out to be negative
 (no collision), returns nullopt. */
template <typename T, typename Backend>
std::optional<PenetrationAsPointPair<T>> MaybeMakePointPair(
    typename Backend::Object* object_A_ptr,
    typename Backend::Object* object_B_ptr,
    const CallbackData<T, Backend>& data);

// clang-format off
}  // namespace penetration_as_point_pair
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
