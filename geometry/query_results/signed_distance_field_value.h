#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance from a query point to a geometry.
  Reports the result of a signed distance query between a query point Q and
  geometry G. This includes geometry G's id, the signed distance, the nearest
  point N on the surface of geometry G, the gradient of the signed distance
  with respect to the position of Q. If the gradient vector is not well defined
  at a query point, we may report an arbitrary vector.

  @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct SignedDistanceFieldValue{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceFieldValue)

  SignedDistanceFieldValue() = default;

  /** Constructs SignedDistanceFieldValue struct from calculated results.
   @param id_G_i     The id of the geometry G to which we measure distance from
                     the query point Q.
   @param p_GN_i     The position of the nearest point N on G's surface from
                     the query point Q, expressed in G's frame.
   @param distance_i The signed distance from the query point Q to the nearest
                     point N on the surface of geometry G. It is positive if
                     Q is outside G. It is negative if Q is inside G. It is zero
                     if Q is on the boundary of G.
   @param grad_W_i   The gradient vector of the distance function with respect
                     to the query point Q, expressed in world frame W.
   @pre grad_W_i must not contain NaN.
   */
  SignedDistanceFieldValue(GeometryId id_G_i, const Vector3<T>& p_GN_i,
                           T distance_i, const Vector3<T>& grad_W_i)
  : id_G(id_G_i), p_GN(p_GN_i), distance(distance_i), grad_W(grad_W_i) {
    using std::isnan;
    DRAKE_ASSERT(!(isnan(grad_W(0))||
                   isnan(grad_W(1))||
                   isnan(grad_W(2))));
  }

  GeometryId id_G;
  Vector3<T> p_GN;
  T distance{};
  Vector3<T> grad_W;
};

}  // namespace geometry
}  // namespace drake
