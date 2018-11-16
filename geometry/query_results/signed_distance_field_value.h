#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance from a query point to a geometry.
  Reports the result of a signed distance query between a query point Q and
  geometry G (with frame G). This includes geometry G's id, the
  signed distance, the nearest point N on geometry G to point Q, the
  gradient of the signed distance with respect to the position of Q
  expressed in the same frame as Q.

  @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct SignedDistanceFieldValue{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceFieldValue)

  SignedDistanceFieldValue() = default;

  /** Constructs SignedDistanceFieldValue struct from calculated results.
   @param id_G_in    The id of the geometry G to which we measure distance from
                     the query point Q.
   @param p_GN_in    The position of the nearest point N on G's surface from
                     the query point Q, expressed in G's frame.
   @param distance_in  The signed distance from the query point Q to the nearest
                     point N. It is positive if the query point Q is outside the
                     geometry G. It is negative if the query point is inside the
                     geometry G. It is zero if the query point is on the
                     boundary of the geometry G.
   @param grad_W_in  The gradient vector of the distance function with respect
                     to the query point Q, expressed in world frame W.
   @pre Assume grad_W_in is not NaN.
   */
  SignedDistanceFieldValue(GeometryId id_G_in, const Vector3<T>& p_GN_in,
                           T distance_in, const Vector3<T>& grad_W_in)
  : id_G(id_G_in), p_GN(p_GN_in), distance(distance_in), grad_W(grad_W_in) {
    using std::isnan;
    DRAKE_ASSERT(!(isnan(grad_W(0))||
                   isnan(grad_W(1))||
                   isnan(grad_W(2))))
  }

  GeometryId id_G;
  Vector3<T> p_GN;
  T distance{};
  Vector3<T> grad_W;
};

}  // namespace geometry
}  // namespace drake
