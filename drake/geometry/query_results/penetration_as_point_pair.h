#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A characterization of the intersection of two penetrating geometries. The
 characterization consists of a pair of points and a normal. The points
 represent a point on each geometry that most deeply penetrates the other
 geometry (in the normal direction). For convenience, the penetration depth
 is provided and is equal to:

     depth = ‖ (p_WCa - p_WCb) ⋅ nhat_AB_W ‖₂.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PenetrationAsPointPair {
  /** The id of the first geometry in the contact. */
  GeometryId id_A;
  /** The id of the second geometry in the contact. */
  GeometryId id_B;
  /** The point on A that most deeply penetrates B, measured and expressed in
   the world frame. */
  Vector3<T> p_WCa;
  /** The point on B that most deeply penetrates A, measured and expressed in
   the world frame. */
  Vector3<T> p_WCb;
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry A to geometry B, measured and expressed in the world frame. */
  Vector3<T> nhat_AB_W;
  /** The penetration depth. */
  T depth{-1.0};
};

}  // namespace geometry
}  // namespace drake
