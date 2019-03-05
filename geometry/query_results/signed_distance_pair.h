#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance between two geometries, A and B.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct SignedDistancePair{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistancePair)

  SignedDistancePair() {}

  /** Constructor
   @param a       The id of the first geometry (A).
   @param b       The id of the second geometry (B).
   @param p_A     The witness point on geometry A's surface, in A's frame.
   @param p_B     The witness point on geometry B's surface, in B's frame.
   @param dist    The signed distance between p_A and p_B. When A and B are
   separated, dist > 0; when A and B are touching or penetrating, dist <= 0.
   @param n_WAB   The unit normal vector between witness points from A to B,
                  in world frame.*/
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_A,
                     const Vector3<T>& p_B, T dist, const Vector3<T>& n_WAB)
      : id_A(a),
        id_B(b),
        p_ACa(p_A),
        p_BCb(p_B),
        distance(dist),
        normal_WAB(n_WAB) {}

  /** The id of the first geometry in the pair. */
  GeometryId id_A;
  /** The id of the second geometry in the pair. */
  GeometryId id_B;
  // TODO(SeanCurtis-TRI): Determine if this is the *right* API. Should we
  // really be returning the points in geometry frame and not the world frame?
  //  1. Penetration as point pair returns the points in world frame.
  //  2. FCL computes it in world frame.
  //  3. Generally, MBP would want the points in *body* frame. MBP has access
  //     to X_WB but does *not* generally have access to X_BG (it would have to
  //     query QueryObject for that information). Although, such a query can
  //     easily be provided.
  /** The witness point on geometry A's surface, expressed in A's frame. */
  Vector3<T> p_ACa;
  /** The witness point on geometry B's surface, expressed in B's frame. */
  Vector3<T> p_BCb;
  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  T distance{};
  /** The unit normal vector from Ca to Cb, expressed in world frame,
   *  normal_WAB = (p_WCb - p_WCa) / |p_WCb - p_WCa|.
   *  @note When A and B are touching, defined as the signed distance = 0, we
   *  define normal_WAB as the direction through which moving B will maximize
   *  the distance. It is the outward unit normal vector to the surface of A
   *  at Ca, if the surface is smooth at Ca.
   *    */
  Vector3<T> normal_WAB;
};

}  // namespace geometry
}  // namespace drake
