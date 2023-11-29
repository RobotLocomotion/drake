#pragma once

#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

/* @file Support distance queries between shapes when they barely touch (have
 zero signed distance) by providing a reasonable distance gradient in fcl
 fallback CalcDistanceFallback() in distance_to_shape_callback.cc.
 */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

/* When two objects touch (have 0 or almost 0 signed distance), computes
 the signed-distance gradient nhat_BA_W, which is a unit vector in the
 direction of fastest increasing distance pointing outward from object `b`
 into object `a`, expressed in World frame.

 @param p_ACa  Witness point of object `a` measured and expressed in frame A.
 @param p_BCb  Witness point of object `b` measured and expressed in frame B.

 @pre  The fcl::CollisionObject `a` and `b` contain their poses X_WA and X_WB
       in World frame.

 @note  This function is used only in fcl fallback CalcDistanceFallback() in
        distance_to_shape_callback.cc.  */
Eigen::Vector3d CalcGradientWhenTouching(const fcl::CollisionObjectd& a,
                                         const fcl::CollisionObjectd& b,
                                         const Eigen::Vector3d& p_ACa,
                                         const Eigen::Vector3d& p_BCb);

/* Returns an outward face normal of box_B if the query point Q is strictly
 on a face; otherwise, returns a Vector3d of NaN. Q is strictly on a face
 when it is on the face but not on an edge or a vertex. It uses an internal
 tolerance.

 @param p_BQ   The position of the query point Q measured and expressed in
               frame B of the box.
 @param box_B  The box expressed in frame B.

 @return An outward face normal of the box expressed in frame B of the box,
         which could be ±Vector3d::UnitX(), ±Vector3d::UnitY(), or
         ±Vector3d::UnitZ(). Furthermore, it could be Vector3d of NaN if
         the query point is not strictly on a face of the box. */
Eigen::Vector3d FaceNormalIfStrictlyOnFace(const Eigen::Vector3d& p_BQ,
                                           const fcl::Boxd& box_B);

/* Returns an axis index i (0, 1, or 2 for X, Y, or Z) of box_B if the query
 point Q is strictly on an edge of box_B parallel to the i-th axis; otherwise,
 returns -1.  Q is strictly on an edge when it is on the edge but not at the
 end points of the edge.  It uses an internal tolerance.

 @param p_BQ   The position of the query point Q measured and expressed in
               frame B of the box.
 @param box_B  The box expressed in frame B. */
int AxisIndexIfStrictlyOnEdge(const Eigen::Vector3d& p_BQ,
                              const fcl::Boxd& box_B);

/* Returns true if the query point Q is at a vertex of the box.  It uses an
 internal tolerance.

 @param p_BQ   The position of the query point Q measured and expressed in
               frame B of the box.
 @param box_B  The box expressed in frame B. */
bool IsAtVertex(const Eigen::Vector3d& p_BQ, const fcl::Boxd& box_B);

/* Returns the projected interval [min, max] of a box on a line through
 World's origin.

 @param box_A  The box expressed in frame A.
 @param X_WA   The pose of the box in World frame.
 @param unit_vector_W   The unit vector of the direction of the line.  */
std::pair<double, double> ProjectedMinMax(const fcl::Boxd& box_A,
                                          const math::RigidTransformd& X_WA,
                                          const Eigen::Vector3d& unit_vector_W);

/* Returns the unit vector nhat_BA_W--out of box_B into box_A expressed in
 World frame--parallel to a vector in `v_Ws` that can make a separating axis
 between two touching boxes. If no vector in `v_Ws` can make it, returns
 Vector3d of NaN.

 See https://en.wikipedia.org/wiki/Hyperplane_separation_theorem for the
 definition of separating axis.

 @note If there are multiple vectors in `v_Ws` that can make a separating
       axis, we use the first one in the list.

 @note A vector in `v_Ws` doesn't have to be a unit vector.

 @pre The signed distance between the two boxes is zero (or within an
      internal tolerance), i.e., the two boxes barely touch.  */
Eigen::Vector3d MakeSeparatingVector(const fcl::Boxd& box_A,
                                     const fcl::Boxd& box_B,
                                     const math::RigidTransformd& X_WA,
                                     const math::RigidTransformd& X_WB,
                                     const std::vector<Eigen::Vector3d>& v_Ws);

/* Computes the signed-distance gradient between two touching boxes.

 @param box_A  The first box expressed in frame A.
 @param box_B  The second box expressed in frame B.
 @param X_WA   The pose of the first box in World frame.
 @param X_WB   The pose of the second box in World frame.
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
