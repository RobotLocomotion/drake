#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
/** Definition of a half space. It is defined by the implicit equation
 `H(x⃗) = n̂⋅x⃗ - d <= 0`. A particular instance is defined by a boundary plane in
 a particular frame F with its normal pointing out of the half space such that
 `H(p_QF) > 0` if the point Q (measured and expressed in F) is outside the
 half space.
 */
template <typename T>
class Plane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Plane)

  /** Constructs a Plane in frame F.
   @param nhat_F
       A unit-length vector perpendicular to the half space's planar boundary
       expressed in frame F (the `n̂` in the implicit equation).
   @param displacement
       The signed distance from F's origin to the half space boundary (the `d`
       term in the implicit equation).
   @pre
       ‖nhat_F‖₂ = 1.
   */
  Plane(const Vector3<T>& nhat_F, const T& displacement)
      : nhat_F_(nhat_F), displacement_(displacement) {
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
   frame F). The point is strictly inside, on the boundary, or outside based on
   the return value being negative, zero, or positive, respectively.
   */
  T CalcSignedDistance(const Vector3<T>& p_FQ) const {
    return nhat_F_.dot(p_FQ) - displacement_;
  }

  /** Reports true if the point Q (measured and expressed in frame F),
   strictly lies outside this half space.
   */
  bool PointIsOutside(const Vector3<T>& p_FQ) const {
    return CalcSignedDistance(p_FQ) > 0;
  }

  /** Gets the normal expressed in frame F. */
  const Vector3<T>& normal() const { return nhat_F_; }

  /** Gets the displacement constant. */
  const T& displacement() const { return displacement_; }

 private:
  Vector3<T> nhat_F_;
  T displacement_{};
};
}  // namespace internal
}  // namespace geometry
}  // namespace drake
