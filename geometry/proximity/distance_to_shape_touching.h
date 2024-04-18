#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

/* @file
 Support distance queries between shapes when they barely touch (have zero
 signed distance) by providing a reasonable distance gradient in fcl fallback
 CalcDistanceFallback() in distance_to_shape_callback.cc.
 */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

// clang-format off
/* When two objects touch (have 0 or almost 0 signed distance), computes
 the signed-distance gradient nhat_BA_W, which is a unit vector in the
 direction of fastest increasing distance pointing outward from object `b`
 into object `a`, expressed in World frame.

 The following table describes the supported geometry pairs.  For the
 unsupported cases, it will return a Vector3d of NaN.  The upper triangular
 part of the table is intentionally left out because it is the symmetric case
 of the lower triangular part.

  |           | %Box | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
  | --------: | :--: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
  | Box       | Ok   |  ░░░░░░  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
  | Capsule   | NaN  |  NaN     |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
  | Convex    | NaN  |  NaN     |  NaN    |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
  | Cylinder  | NaN  |  NaN     |  NaN    |  NaN      |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
  | Ellipsoid | NaN  |  NaN     |  NaN    |  NaN      |   NaN      |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
  | HalfSpace | NaN  |  NaN     |  NaN    |  NaN      |   NaN      |   NaN      |  ░░░░░  |  ░░░░░  |
  | Mesh      | NaN  |  NaN     |  NaN    |  NaN      |   NaN      |   NaN      |  NaN    |  ░░░░░  |
  | Sphere    | Okᵃ  |  Okᵃ     |  Okᵃ    |  Okᵃ      |   Okᵃ      |   Okᵃ      |  Okᵃ    |  Okᵇ    |

 - ᵃ Return a Vector3d of NaN if the sphere has zero radius.
 - ᵇ Return a Vector3d of NaN if both spheres have zero radii.

 @param p_ACa  Witness point of object `a` measured and expressed in frame A.
 @param p_BCb  Witness point of object `b` measured and expressed in frame B.

 @note  This function is used only in fcl fallback CalcDistanceFallback() in
        distance_to_shape_callback.cc.  */
// clang-format on
Eigen::Vector3d CalcGradientWhenTouching(const fcl::CollisionObjectd& a,
                                         const math::RigidTransformd& X_WA,
                                         const fcl::CollisionObjectd& b,
                                         const math::RigidTransformd& X_WB,
                                         const Eigen::Vector3d& p_ACa,
                                         const Eigen::Vector3d& p_BCb);

/* Helper method to determine whether a point `p_BQ` lies approximately (to
 internal tolerance) on either a face, edge or vertex of `box_B`.

 @returns An encoding v, such that:
 v[0] = 1 if p_BQ is on one of the two faces normal to Bx axis, otherwise 0.
 v[1] = 1 if p_BQ is on one of the two faces normal to By axis, otherwise 0.
 v[2] = 1 if p_BQ is on one of the two faces normal to Bz axis, otherwise 0.
 The quantity v.sum() encodes whether p_BQ lies stricly on a face (1), edge (2),
 vertex(3), or it's not on the surface (0).

 @note Since we use an internal tolerance, the classification of the location
 of `p_BQ` (face, edge, or vertex of `box_B`) depends on its precision.
 For example, if p_BQ is on a face normal to Bx axis, p_BQ.x() should be
 within the internal tolerance from box_B.side.x()/2.  */
Eigen::Vector3d PointOnBoxSurfaceHelper(const Eigen::Vector3d& p_BQ,
                                        const fcl::Boxd& box_B);

/* Returns the projected interval [min, max] of a box on a line through
 World's origin. The line is defined by the parametric function s(t) = û_W⋅t
 such that s(0) is located at the world origin.

 @param box_A  The box expressed in frame A.
 @param X_WA   The pose of the box in World frame.
 @param unit_vector_W   The unit vector of the direction of the line.

 @note  Assume the caller already normalized the unit_vector_W. This function
        will give wrong answer if it's not a unit vector. */
std::pair<double, double> ProjectedMinMax(const fcl::Boxd& box_A,
                                          const math::RigidTransformd& X_WA,
                                          const Eigen::Vector3d& unit_vector_W);

/* Returns the unit vector nhat_BA_W--out of box_B into box_A expressed in
 World frame--parallel to a vector in `v_Ws` that can make a separating axis
 between two touching boxes. If no vector in `v_Ws` can make it, returns
 std::nullopt.

 See https://en.wikipedia.org/wiki/Hyperplane_separation_theorem for the
 definition of separating axis.

 @note If there are multiple vectors in `v_Ws` that can make a separating
       axis, we use the first one in the list.

 @note A vector in `v_Ws` doesn't have to be a unit vector. It can even be a
       zero (or near-zero) vector, which will be ignored.

 @pre The signed distance between the two boxes is zero (or within an
      internal tolerance), i.e., the two boxes barely touch.  */
std::optional<Eigen::Vector3d> MaybeMakeSeparatingVector(
    const fcl::Boxd& box_A, const fcl::Boxd& box_B,
    const math::RigidTransformd& X_WA, const math::RigidTransformd& X_WB,
    const std::vector<Eigen::Vector3d>& v_Ws);

/* Computes the signed-distance gradient between two touching boxes.

 @param box_A  The first box expressed in frame A.
 @param box_B  The second box expressed in frame B.
 @param X_WA   The pose of box_A in World frame.
 @param X_WB   The pose of box_B in World frame.
 @param p_ACa  The witness point of box_A, measured and expressed in frame A.
 @param p_BCb  The witness point of box_B, measured and expressed in frame B.

 @return nhat_BA_W a unit vector in the direction of fastest increasing
         distance pointing outward from box_B into box_A.

 @note In some cases, the gradient is not unique, and we choose arbitrarily.

 @pre The signed distance between the two boxes is zero (or within an
      internal tolerance), i.e., the two witness points are at the same
      location (or within an internal tolerance) in World frame. */
Eigen::Vector3d BoxBoxGradient(const fcl::Boxd& box_A, const fcl::Boxd& box_B,
                               const math::RigidTransformd& X_WA,
                               const math::RigidTransformd& X_WB,
                               const Eigen::Vector3d& p_ACa,
                               const Eigen::Vector3d& p_BCb);

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
