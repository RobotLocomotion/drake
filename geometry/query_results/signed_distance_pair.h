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
   @param n_BA_W  The unit direction of the gradient of the signed distance
                  field of B evaluated at p_B, expressed in world frame. By
                  definition, the unit direction of the gradient of the signed
                  distance field of A evaluated at Ca must be in the opposite
                  direction.*/
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_A,
                     const Vector3<T>& p_B, T dist, const Vector3<T>& n_BA_W)
      : id_A(a),
        id_B(b),
        p_ACa(p_A),
        p_BCb(p_B),
        distance(dist),
        nhat_BA_W(n_BA_W) {}

  // TODO(DamrongGuoy): Remove this constructor when it's not needed.  Right
  //   now the unit tests need it.
  /** Constructor.
   We keep this constructor tempoarily for backward compatibility.
   @param a       The id of the first geometry (A).
   @param b       The id of the second geometry (B).
   @param p_A     The witness point on geometry A's surface, in A's frame.
   @param p_B     The witness point on geometry B's surface, in B's frame.
   @param dist    The signed distance between p_A and p_B. When A and B are
   separated, dist > 0; when A and B are touching or penetrating, dist <= 0.*/
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_A,
                     const Vector3<T>& p_B, T dist)
      : id_A(a),
        id_B(b),
        p_ACa(p_A),
        p_BCb(p_B),
        distance(dist),
        nhat_BA_W(Vector3<T>::Zero()) {}


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
  /** The signed distance between p_ACa and p_BCb (measured in a common frame).
   When A and B are separated, distance > 0; when A and B are touching or
   penetrating, distance <= 0.
   */
  T distance{};
  /** The unit direction of the gradient of the signed distance field of B
   evaluated at Cb, expressed in world frame. By definition, the unit direction
   of the gradient of the signed distance field of A evaluated at Ca must be
   in the opposite direction.
   @note For non-touching objects, nhat_BA_W points from Cb to Ca, i.e.,
   nhat_BA_W = (p_WCa - p_WCb) / (signed)distance.
   @note Guaranteed to be defined, even when the objects are touching.
   @note It is always an outward unit normal vector to the surface of B
   at Cb whether A and B are separated, touching, or penetrating.
   @note If the normal vector to the surface of B at Cb is not unique but the
   outward normal vector n_A to the surface of A at Ca is unique, nhat_BA_W
   will be in the opposite direction of n_A.
   @note If both the normal vector to the surface of B at Cb and that of A at
   Ca are not unique, nhat_BA_W will be one of the valid outward unit normal
   vector to the surface of B at Cb.
   */
  Vector3<T> nhat_BA_W;
};

}  // namespace geometry
}  // namespace drake
