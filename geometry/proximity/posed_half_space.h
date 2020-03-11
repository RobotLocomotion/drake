#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/plane.h"

namespace drake {
namespace geometry {
namespace internal {

/** The definition of a half space, posed in an arbitrary frame. It is
 defined by its boundary plane such that a point's signed distance relative to
 the half space is equal to its height relative to the boundary plane (see
 Plane for details).

 The %PosedHalfSpace class is distinct from the HalfSpace class in that the
 HalfSpace class defines the half space in its own canonical frame. The posed
 half space can be defined in an arbitrary frame.

 The signed distance of a point Q can only be meaningfully evaluated if the
 point is measured and expressed in the same frame as the half space:

 ```
 const Vector3<T> nhat_F = ...;
 const Vector3<T> p_FP = ...;  // P is a point on the half space boundary.
 const PosedHalfSpace<T> half_space_F(nhat_F, p_FP);  // Half space in frame F.
 const double distance_Q = half_space_F.CalcSignedDistance(p_FQ);  // valid!
 const double distance_R = half_space_F.CalcSignedDistance(p_GR);  // invalid!
 ```
 */
template <typename T>
class PosedHalfSpace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PosedHalfSpace)

  /** Constructs a PosedHalfSpace in frame F from a normal to the half space's
   boundary plane and a point on that plane.
   @param nhat_F
       A unit-length vector perpendicular to the boundary plane expressed in
       Frame F. The normal points _out_ of the half space.
   @param p_FP
       A point on the boundary plane measured and expressed in Frame F.
   @pre
       ‖nhat_F‖₂ = 1.   */
  PosedHalfSpace(const Vector3<T>& nhat_F, const Vector3<T>& p_FP)
      : plane_(nhat_F, p_FP) {}

  /** Computes the signed distance to the point Q (measured and expressed in
   Frame F). If Q's signed distance is positive, it lies outside the half space.
   If it is negative, it lies inside. If it is zero, it lies on the surface of
   the half space.  */
  T CalcSignedDistance(const Vector3<T>& p_FQ) const {
    return plane_.CalcHeight(p_FQ);
  }

  /** Gets the normal expressed in Frame F. */
  const Vector3<T>& normal() const { return plane_.normal(); }

  /** Gets the boundary plane of `this` posed half space.  */
  const Plane<T>& boundary_plane() const { return plane_; }

 private:
  Plane<T> plane_;
};
}  // namespace internal
}  // namespace geometry
}  // namespace drake
