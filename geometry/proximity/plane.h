#pragma once

#include <limits>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

/** The definition of a plane in ℜ³, posed in an arbitrary frame. The plane
 normal implicitly defines "above" and "below" directions relative to the plane.
 The "height" of a point relative to the plane can be queried.

 It is defined with the implicit equation: `P(x⃗) = n̂⋅x⃗ - d = 0`. A particular
 instance is measured and expressed in a particular frame, such that only points
 measured and expressed in that same frame can be meaningfully compared to the
 plane. E.g.,

 ```
 const Vector3<T> nhat_F = ...;
 const Vector3<T> p_FP = ...;  // P is a point on the plane.
 const Plane<T> plane_F(nhat_F, p_FP);  // Plane in frame F.
 const double distance_Q = plane_F.CalcHeight(p_FQ);  // valid!
 const double distance_R = plane_F.CalcHeight(p_GR);  // invalid!
 ```
 */
template <typename T>
class Plane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Plane)

  /** Constructs a %Plane in frame F which is normal to `n_F` and passes
   through the point `p_FP`.
   @param n_F
       A (possibly unit-length) vector perpendicular to the plane expressed in
       Frame F (the `n̂` in the implicit equation). By default, the vector will
       be normalized before being stored (see below).
   @param p_FP
       A point on the plane measured and expressed in Frame F. The `d` in the
       implicit equation is derived from this quantity.
   @param needs_normalization
       If `false`, the `n_F` will be treated as if it has already been
       normalized by the caller. This is an _advanced_ feature. Providing
       `false` incorrectly will cause height calculations to be scaled by the
       _actual_ normal magnitude.
   @pre If `needs_normalization` is `true`, `n_F` must have magnitude ≥ 1e-10.
   */
  Plane(const Vector3<T>& n_F, const Vector3<T>& p_FP,
        bool needs_normalization = true) {
    if (needs_normalization) {
      const T magnitude = n_F.norm();
      if (magnitude < 1e-10) {
        throw std::runtime_error(
            fmt::format("Cannot instantiate plane from normal n_F = [{}]; its "
                        "magnitude is too small: {}",
                        n_F.transpose(), magnitude));
      }
      nhat_F_ = n_F / magnitude;
    } else {
      nhat_F_ = n_F;
    }
    displacement_ = nhat_F_.dot(p_FP);
  }

  /** Computes the height of Point Q relative to the plane. A positive height
   indicates the point lies _above_ the plane; negative height indicates
   _below_. The point must be measured and expressed in the same frame as the
   plane.   */
  T CalcHeight(const Vector3<T>& p_FQ) const {
    return nhat_F_.dot(p_FQ) - displacement_;
  }

  /** Gets the plane's normal expressed in frame F. */
  const Vector3<T>& normal() const { return nhat_F_; }

 private:
  Vector3<T> nhat_F_;
  T displacement_{};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
