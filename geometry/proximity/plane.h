#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
/** Definition of a plane. It is defined by the implicit equation
 `P(x⃗) = n̂⋅x⃗ + d = 0`. A particular instance is measured and expressed in a
 particular frame, such that only points measured and expressed in that same
 frame can be meaningfully compared to the plane. E.g.,

 ```
 const Vector3<T> nhat_F = ...;
 const Vector3<T> p_FP = ...;  // P is a point on the plane.
 const Plane<T> plane_F(nhat_F, p_FP);  // A plane in frame F.
 const double distance_Q = plane_F.CalcSignedDistance(p_FQ);  // valid!
 const double distance_R = plane_F.CalcSignedDistance(p_GR);  // invalid!
 ```
 */
template <typename T>
class Plane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Plane)

  /** Constructs a Plane in frame F from a normal and a point on the plane.
   @param nhat_F
       A unit-length vector perpendicular to the plane expressed in Frame F
       (the `n̂` in the implicit equation).
   @param p_FP
       A point on the plane measured and expressed in Frame F. The `d` in the
       implicit equation is derived from this quantity.
   @pre
       ‖nhat_F‖₂ = 1.
   */
  Plane(const Vector3<T>& nhat_F, const Vector3<T>& p_FP)
      : nhat_F_(nhat_F), displacement_(nhat_F.dot(p_FP)) {
    using std::abs;
    // Note: This may *seem* like a very tight threshold for determining if a
    // vector is unit length. However, empirical evidence suggests that in
    // double precision, normalizing a vector generally makes a vector whose
    // evaluated magnitude is within epsilon of one. There may be some
    // unconsidered value that disproves this -- at that point, adapt the
    // tolerance here and add it to the unit test.
    DRAKE_THROW_UNLESS(abs(nhat_F_.norm() - 1.0) <=
        std::numeric_limits<double>::epsilon());
  }

  /** Computes the signed distance to the point Q (measured and expressed in
   Frame F). A positive signed distance implies that the point lies on the side
   of the plane in the normal direction. A zero implies Q lies on the plane.
   A negative signed distance lies on the opposite side of the plane (away from
   the normal direction).
   */
  T CalcSignedDistance(const Vector3<T>& p_FQ) const {
    return nhat_F_.dot(p_FQ) - displacement_;
  }

  /** Gets the normal expressed in frame F. */
  const Vector3<T>& normal() const { return nhat_F_; }

 private:
  Vector3<T> nhat_F_;
  T displacement_{};
};
}  // namespace internal
}  // namespace geometry
}  // namespace drake
