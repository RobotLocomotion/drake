#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/geometry/proximity/plane.h"

namespace drake {
namespace geometry {
namespace internal {

/* The definition of a half space, posed in an arbitrary frame. It is
 defined by its boundary plane such that a point's signed distance relative to
 the half space is equal to its height relative to the boundary plane (see
 Plane for details). The boundary plane's normal points outside the half space.

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

  /* Constructs a half space in frame F in terms of its boundary plane. The
   parameters define the boundary plane (see Plane for details). The boundary
   plane's normal points _outside_ the half space.  */
  PosedHalfSpace(const Vector3<T>& nhat_F, const Vector3<T>& p_FP,
                 bool already_normalized = false)
      : plane_(nhat_F, p_FP, already_normalized) {}

  /* Computes the signed distance to the point Q (measured and expressed in
   Frame F). If Q's signed distance is positive, it lies outside the half space.
   If it is negative, it lies inside. If it is zero, it lies on the boundary
   plane of the half space.

   The return type depends on both the half space's scalar type `T` and the
   given query point's scalar type `U`. See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.
   */
  template <typename U = T>
  promoted_numerical_t<U, T> CalcSignedDistance(const Vector3<U>& p_FQ) const {
    return plane_.CalcHeight(p_FQ);
  }

  /* Gets the normal expressed in Frame F. */
  const Vector3<T>& normal() const { return plane_.normal(); }

  /* Gets the boundary plane of `this` posed half space.  */
  const Plane<T>& boundary_plane() const { return plane_; }

 private:
  Plane<T> plane_;
};
}  // namespace internal
}  // namespace geometry
}  // namespace drake
