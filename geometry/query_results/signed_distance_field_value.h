#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance from a query point to a geometry.
 * @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */

template <typename T>
struct SignedDistanceFieldValue{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceFieldValue)

  SignedDistanceFieldValue() = default;

  /** Constructor
   * @param id_G_in  The id of the geometry G to which we measure distance from
   *                 the query point.
   * @param p_GN_in  The position of the nearest point N on G's surface from
   *                 the query point, espressed in G's frame.
   * @param dist     The distance from the query point Q to the nearest point N.
   * @param v_QN_in  The gradient vector of the distance function with
   *                 respect to the query point, expressed in Q's frame.
   */
  SignedDistanceFieldValue(GeometryId id_G_in, const Vector3<T>& p_GN_in,
                           T dist, const Vector3<T>& v_QN_in)
  : id_G(id_G_in), p_GN(p_GN_in), distance(dist), v_QN(v_QN_in) {}

  /** The id of the geometry G to which we measure distance from the query
   * point */
  GeometryId id_G;
  /** The position of the point N on the geometry G's surface nearest to the
   * query point, expressed in G's frame. */
  Vector3<T> p_GN;
  /** The distance from the query point to the nearest point N. It is
   * positive if the query point Q is outside the geometry G. It is zero if
   * the query point is on the boundary of the geometry G. It is negative if
   * the query point is inside the geometry G. */
  T distance{};
  /** The gradient vector ∂d/∂q of the distance function with respect to the
   * query point Q, expressed in Q's frame */
  Vector3<T> v_QN;
};

}  // namespace geometry
}  // namespace drake
