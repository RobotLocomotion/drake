#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance between two geometries, A and B.
 It provides the id's of the two geometries, the witness points Ca and Cb on
 the surfaces of A and B, the signed distance, and the unit direction nhat_BA_W
 of the gradient of the signed distance field of B evaluated at Cb (always
 pointing outward from B's surface).
     When A and B are separated, distance > 0; when A and B are touching or
 penetrating, distance <= 0.
     By definition, the unit direction of the gradient of the signed distance
 field of A evaluated at Ca must be in the opposite direction of nhat_BA_W.
 @note For non-touching objects, nhat_BA_W points from Cb to Ca, i.e.,
       nhat_BA_W = (p_WCa - p_WCb) / (signed)distance.
 @note For two touching objects, consider the case when the normal vector to
       the surface of B at Cb is not unique but the outward normal vector
       n_A to the surface of A at Ca is unique. For example, a corner of a
       box B touches a sphere A, or the corner of a box B touches a planar
       side of a box A. Then, nhat_BA_W will be in the opposite direction of
       the unique n_A.
                 __                      __
                |  | box B     box B /\ |  |
              __|__|                /  \|  |
             /  \                   \  /|  | box A
    sphere A \__/                    \/ |__|

 @note Consider the case when both the normal vectors to the surface of B at
       Cb and that of A at Ca are not unique.  For example, a corner of a box
       B touches a corner of a box A. Then, nhat_BA_W will be one of the
       valid outward unit normal vector to the surface of B at Cb.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct SignedDistancePair{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistancePair)

  SignedDistancePair() {}

  /** Constructor
   @param a         The id of the first geometry (A).
   @param b         The id of the second geometry (B).
   @param p_ACa_in  The witness point on geometry A's surface, in A's frame.
   @param p_BCb_in  The witness point on geometry B's surface, in B's frame.
   @param dist      The signed distance between p_A and p_B.
   @param nhat_BA_W_in  The unit direction of the gradient of the signed
                        distance field of B evaluated at p_B, expressed in
                        the world frame.*/
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_ACa_in,
                     const Vector3<T>& p_BCb_in, T dist,
                     const Vector3<T>& nhat_BA_W_in)
      : id_A(a),
        id_B(b),
        p_ACa(p_ACa_in),
        p_BCb(p_BCb_in),
        distance(dist),
        nhat_BA_W(nhat_BA_W_in)
  // TODO(DamrongGuoy): When we have a full implementation of computing
  //  nhat_BA_W in ComputeSignedDistancePairwiseClosestPoints, add document:
  //      @pre nhat_BA_W_in is unit-length.
  //  and check a condition like this (within epsilon):
  //      DRAKE_DEMAND(nhat_BA_W.norm() == T(1.));
  {}

  // TODO(DamrongGuoy): Remove this constructor when we have a full
  //  implementation of computing nhat_BA_W in
  //  ComputeSignedDistancePairwiseClosestPoints.  Right now many unit tests
  //  need it.  The downside is that Python binder calls the doc for the
  //  above constructor ctor.doc_6args and this one ctor.doc_5args.
  /** Constructor.
   We keep this constructor temporarily for backward compatibility.
   @param a       The id of the first geometry (A).
   @param b       The id of the second geometry (B).
   @param p_ACa_in  The witness point on geometry A's surface, in A's frame.
   @param p_BCb_in  The witness point on geometry B's surface, in B's frame.
   @param dist    The signed distance between p_A and p_B.*/
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_ACa_in,
                     const Vector3<T>& p_BCb_in, T dist)
      : id_A(a),
        id_B(b),
        p_ACa(p_ACa_in),
        p_BCb(p_BCb_in),
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
  /** The signed distance between p_ACa and p_BCb. */
  T distance{};
  /** The unit direction of the gradient of the signed distance field of B
   evaluated at Cb, expressed in the world frame. */
  Vector3<T> nhat_BA_W;
};

}  // namespace geometry
}  // namespace drake
