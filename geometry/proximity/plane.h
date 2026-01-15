#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {

/** The definition of a plane in ℜ³, posed in an arbitrary frame. The plane
 normal implicitly defines "above" and "below" directions relative to the plane.
 The "height" of a point relative to the plane can be queried.

 It is defined with the implicit equation: `P(x⃗ = n̂⋅x⃗- d = 0`. A particular
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Plane);

  /** Constructs a %Plane in frame F which is normal to `normal` and passes
   through the point `point_on_plane`.
   @param normal
       A (possibly unit-length) vector perpendicular to the plane expressed in
       Frame F (the `n̂` in the implicit equation). By default, the vector will
       be normalized before being stored (see below), becoming the nhat_F
       documented above.
   @param point_on_plane
       A point on the plane measured and expressed in Frame F, p_FP. The `d` in
       the implicit equation is derived from this quantity.
   @param already_normalized
       (Advanced) If `true`, the `normal` will be treated as if it has already
       been normalized by the caller. It should still essentially have unit
       length. This function reserves the right to validate this property in
       debug build up to an arbitrary tolerance. When in doubt, allow the plane
       to normalize the normal vector.
   @pre If `already_normalized` is `false`, `normal` must have magnitude ≥
       1e-10.
   */
  Plane(const Vector3<T>& normal, const Vector3<T>& point_on_plane,
        bool already_normalized = false);

  /** Computes the height of Point Q relative to the plane. A positive height
   indicates the point lies _above_ the plane; negative height indicates
   _below_. The point must be measured and expressed in the same frame as the
   plane.

   The return type depends on both the plane's scalar type `T` and the given
   query point's scalar type `U`. See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.

   @param point  The quantity p_FQ (query point Q measured and expressed in the
                 plane's frame F).
   */
  template <typename U = T>
  promoted_numerical_t<U, T> CalcHeight(const Vector3<U>& point) const {
    return nhat_F_.dot(point) - displacement_;
  }

  /** Gets the plane's unit normal expressed in frame F. */
  const Vector3<T>& unit_normal() const { return nhat_F_; }

  /** Returns a point on the plane, measured and expressed in frame F. This is
   not necessarily the same point used to construct the plane. */
  const Vector3<T> reference_point() const { return nhat_F_ * displacement_; }

  /** Reports if the given box intersects this plane. The plane is specified in
   a frame P (the plane normal is not necessarily aligned with Pz). The box is
   specified in generic terms. It is an box whose axes are aligned to frame B,
   centered on Bo, and posed in the plane's frame P.

   @param half_width                The half-width extents of the box along its
                                    local axes.
   @param box_center_in_plane       The center of the box measured and expressed
                                    in the plane's frame: p_PBo.
   @param box_orientation_in_plane  The orientation of the box expressed in the
                                    plane's frame. The ith column goes with the
                                    ith half width: R_PB.
   */
  bool BoxOverlaps(const Eigen::Vector3d& half_width,
                   const Eigen::Vector3d& box_center_in_plane,
                   const math::RotationMatrixd& box_orientation_in_plane) const;

 private:
  // Used to validate the "already normalized" plane normal in debug mode.
  static void ThrowIfInsufficientlyNormal(const Vector3<T>& n);

  Vector3<T> nhat_F_;
  T displacement_{};
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class Plane);

}  // namespace geometry
}  // namespace drake
