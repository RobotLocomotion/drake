#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance from a query point to a geometry.
  Reports the result of a signed distance query between a query point Q and
  geometry G. This includes G's id, the signed distance, the nearest point N
  on the surface of G, and the gradient of the signed distance with respect to
  the position of Q. Generally, the gradient of the signed distance function is
  not defined everywhere. The value reported in this struct depends on the
  query function returning it. Refer to the query function's documentation
  for what value it will report for otherwise undefined gradient values.

  @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct SignedDistanceToPoint{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceToPoint)

  SignedDistanceToPoint() = default;

  /** Constructs SignedDistanceToPoint struct from calculated results.
   @param id_G_in     The id of the geometry G to which we measure distance from
                      the query point Q.
   @param p_GN_in     The position of the nearest point N on G's surface from
                      the query point Q, expressed in G's frame.
   @param distance_in The signed distance from the query point Q to the nearest
                      point N on the surface of geometry G. It is positive if
                      Q is outside G. It is negative if Q is inside G. It is
                      zero if Q is on the boundary of G.
   @param grad_W_in   The gradient vector of the distance function with respect
                      to the query point Q, expressed in world frame W.
   @param is_grad_W_unique_in  True if grad_W is unique, false otherwise.

   @note grad_W is not well defined everywhere. For example, when computing the
         distance from a point to a sphere, and the point coincides with the
         center of the sphere, grad_W is not well defined (as it can be computed
         as p_GQ / |p_GQ|, but the denominator is 0). When grad_W is not
         well defined, and we instantiate SignedDistanceToPoint<T> with T being
         an AutoDiffScalar (like AutoDiffXd), the gradient of the query result
         is not well defined either, so the user should use the gradient in
         p_GN, distance and grad_W with caution.
   @pre grad_W_in must not contain NaN.
   */
  SignedDistanceToPoint(GeometryId id_G_in, const Vector3<T>& p_GN_in,
                        T distance_in, const Vector3<T>& grad_W_in,
                        bool is_grad_W_unique_in)
      : id_G(id_G_in),
        p_GN(p_GN_in),
        distance(distance_in),
        grad_W(grad_W_in),
        is_grad_W_unique(is_grad_W_unique_in) {
    using std::isnan;
    DRAKE_ASSERT(!(isnan(grad_W(0)) || isnan(grad_W(1)) || isnan(grad_W(2))));
  }

  /** The id of the geometry G to which we measure distance from the query
      point Q. */
  GeometryId id_G;
  /** The position of the nearest point N on G's surface from the query
      point Q, expressed in G's frame. */
  Vector3<T> p_GN;
  /** The signed distance from the query point Q to the nearest point N on the
      surface of geometry G. It is positive if Q is outside G. It is negative
      if Q is inside G. It is zero if Q is on the boundary of G. */
  T distance{};
  /** The gradient vector of the distance function with respect to the query
      point Q, expressed in world frame W. */
  Vector3<T> grad_W;

  /** Whether grad_W is well defined.
   * Ref to the constructor SignedDistanceToPoint() for an explanation.
   */
  bool is_grad_W_unique;
};

}  // namespace geometry
}  // namespace drake
