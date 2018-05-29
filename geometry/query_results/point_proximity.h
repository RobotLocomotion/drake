#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the geometry nearest a query point. The struct does
 not store the value of the query point, but assumes the user will correlate the
 result with the previously provided query point.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PointProximity {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointProximity)

  PointProximity() {}
  /** Constructor
   @param id              The id of the near geometry (named "A").
   @param local_point     The point on A's surface near the query point (the
                          proximal point), in A's frame.
   @param world_point     The proximal point, in the world frame.
   @param world_dir       A unit-length vector pointing from the proximal
                          point to the query point, in the world frame.
   @param dist            The distance from the query point to the proximal
                          point. */
  PointProximity(GeometryId id, const Vector3<T>& local_point,
                 const Vector3<T>& world_point, const Vector3<T>& world_dir,
                 T dist)
      : id_A(id),
        p_AQa(local_point),
        p_WQa(world_point),
        rhat_CaQ_W(world_dir),
        distance(dist) {}
  /** The id of the near geometry, named "A". */
  GeometryId id_A;
  /** The point on A's surface nearest the query point, in A's frame. */
  Vector3<T> p_AQa;
  /** The point on A's surface nearest the query point, in the world frame. */
  Vector3<T> p_WQa;
  /** An _optional_ unit-length vector indicating the direction from the point
   on A's surface to the query point Q (measured and expressed in the world
   frame). This only contains a vector if the query point does _not_ lie on the
   surface (i.e., |distance| > epsilon). */
  optional<Vector3<T>> rhat_CaQ_W;
  /** The *signed* distance between p_ACa and the query point (negative values
   imply the point lies *inside* the surface). */
  T distance{};
};
}  // namespace geometry
}  // namespace drake
