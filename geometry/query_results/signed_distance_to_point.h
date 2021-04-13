#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the signed distance from a query point to a geometry.
  Reports the result of a signed distance query between a query point Q and
  geometry G. This includes G's id, the signed distance, the nearest point N
  on the surface of G, and the gradient of the signed distance with respect to
  the position of Q. Generally, the gradient of the signed distance function is
  not defined everywhere. The value reported in this struct depends on the
  query function returning it. Refer to the query function's documentation
  for what value it will report for otherwise undefined gradient values.

  @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
struct SignedDistanceToPoint{
  // 2021-08-01 deprecation note: While deprecating the is_grad_W_unique, we
  // cannot use the standard Drake mechanism for declaring copy and move
  // semantics. The default implementations insist on writing to the deprecated
  // member and the macro rebels against being enclosed in the
  // deprecation-suppressing pragma. Once deprecation is complete restore the
  // standard boilerplate:
  //
  //  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceToPoint)
  //  SignedDistanceToPoint() = default;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  SignedDistanceToPoint() = default;
  /** @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
   MoveAssignable */
  //@{
  SignedDistanceToPoint(const SignedDistanceToPoint& other) = default;
  SignedDistanceToPoint(SignedDistanceToPoint&& other) = default;
  SignedDistanceToPoint& operator=(const SignedDistanceToPoint& other) =
      default;
  SignedDistanceToPoint& operator=(SignedDistanceToPoint&& other) =
      default;
  //@}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/* 2021-08-01 deprecation note: This constructor is wrapped in a deprecation
 silencer because gcc will implicitly try to write to the deprecated member and
 throw a build error. When the deprecated member is removed, we *keep* this
 constructor, but lose the pragma manipulations (in contrast to the other
 constructors which will just be deleted as noted). */
  /** Constructs SignedDistanceToPoint struct from calculated results.
   @param id_G_in     The id of the geometry G to which we measure distance from
                      the query point Q.
   @param p_GN_in     The position of the nearest point N on G's surface from
                      the query point Q, expressed in G's frame.
   @param distance_in The signed distance from the query point Q to the nearest
                      point N on the surface of geometry G. It is positive if
                      Q is outside G. It is negative if Q is inside G. It is
                      zero if Q is on the boundary of G.
   @param grad_W_in   The gradient vector of the distance function with respect
                      to the query point Q, expressed in world frame W.

   @note `grad_W` is not well defined everywhere. For example, when computing
         the distance from a point to a sphere, and the point coincides with the
         center of the sphere, grad_W is not well defined (as it can be computed
         as p_GQ / |p_GQ|, but the denominator is 0). When grad_W is not
         well defined, and we instantiate SignedDistanceToPoint<T> with T being
         an AutoDiffScalar (like AutoDiffXd), the gradient of the query result
         is not well defined either, so the user should use the gradient in
         p_GN, distance and grad_W with caution.
   @pre grad_W_in must not contain NaN.
   */
  SignedDistanceToPoint(GeometryId id_G_in, const Vector3<T>& p_GN_in,
                        T distance_in, const Vector3<T>& grad_W_in)
      : id_G(id_G_in),
        p_GN(p_GN_in),
        distance(distance_in),
        grad_W(grad_W_in) {
    using std::isnan;
    DRAKE_ASSERT(!(isnan(grad_W(0)) || isnan(grad_W(1)) || isnan(grad_W(2))));
  }
#pragma GCC diagnostic pop

  /** Constructs SignedDistanceToPoint struct from calculated results.
   @param id_G_in     The id of the geometry G to which we measure distance from
                      the query point Q.
   @param p_GN_in     The position of the nearest point N on G's surface from
                      the query point Q, expressed in G's frame.
   @param distance_in The signed distance from the query point Q to the nearest
                      point N on the surface of geometry G. It is positive if
                      Q is outside G. It is negative if Q is inside G. It is
                      zero if Q is on the boundary of G.
   @param grad_W_in   The gradient vector of the distance function with respect
                      to the query point Q, expressed in world frame W.
   @param is_grad_W_unique_in  True if grad_W is unique, false otherwise.

   @note grad_W is not well defined everywhere. For example, when computing the
         distance from a point to a sphere, and the point coincides with the
         center of the sphere, grad_W is not well defined (as it can be computed
         as p_GQ / |p_GQ|, but the denominator is 0). When grad_W is not
         well defined, and we instantiate SignedDistanceToPoint<T> with T being
         an AutoDiffScalar (like AutoDiffXd), the gradient of the query result
         is not well defined either, so the user should use the gradient in
         p_GN, distance and grad_W with caution.
   @pre grad_W_in must not contain NaN.
   */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_DEPRECATED("2021-08-01",
                   "SignedDistanceToPoint will no longer report uniqueness. "
                   "If you require knowledge of uniqueness, please contact the "
                   "Drake team.")
  SignedDistanceToPoint(GeometryId id_G_in, const Vector3<T>& p_GN_in,
                        T distance_in, const Vector3<T>& grad_W_in,
                        bool is_grad_W_unique_in)
      : id_G(id_G_in),
        p_GN(p_GN_in),
        distance(distance_in),
        grad_W(grad_W_in),
        is_grad_W_unique(is_grad_W_unique_in) {
    using std::isnan;
    DRAKE_ASSERT(!(isnan(grad_W(0)) || isnan(grad_W(1)) || isnan(grad_W(2))));
  }
#pragma GCC diagnostic pop

  /** The id of the geometry G to which we measure distance from the query
      point Q. */
  GeometryId id_G;
  /** The position of the nearest point N on G's surface from the query
      point Q, expressed in G's frame. */
  Vector3<T> p_GN;
  /** The signed distance from the query point Q to the nearest point N on the
      surface of geometry G. It is positive if Q is outside G. It is negative
      if Q is inside G. It is zero if Q is on the boundary of G. */
  T distance{};
  /** The gradient vector of the distance function with respect to the query
      point Q, expressed in world frame W. */
  Vector3<T> grad_W;

  // TODO(SeanCurtis-TRI) When it comes time to deprecate (2021-08-01), ping the
  //  listed author. Deprecation in this case entails more than simply removing
  //  this field; it includes changing code in distance_to_point_callback.h,
  //  its unit tests, and in distance_to_shape_callback.h.
  /** Whether grad_W is well defined.
   * Ref to the constructor SignedDistanceToPoint() for an explanation.
   */
  DRAKE_DEPRECATED("2021-08-01",
                   "SignedDistanceToPoint will no longer report uniqueness. "
                   "If you require knowledge of uniqueness, please contact the "
                   "Drake team.")
  bool is_grad_W_unique{false};
};

}  // namespace geometry
}  // namespace drake
