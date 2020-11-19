#pragma once

#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A characterization of the intersection of two penetrating geometries. The
 characterization consists of a pair of points and a normal. The points
 represent a point on each geometry that most deeply penetrates the other
 geometry (in the normal direction). For convenience, the penetration depth
 is provided and is equal to:

     depth = `(p_WCb - p_WCa) ⋅ nhat_BA_W`

 (`depth` is strictly positive when there is penetration and otherwise not
 defined.)

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct PenetrationAsPointPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PenetrationAsPointPair)
  PenetrationAsPointPair() = default;

  /** Swaps the interpretation of geometries A and B. */
  void SwapAAndB() {
    // Leave depth unchanged.
    std::swap(id_A, id_B);
    std::swap(p_WCa, p_WCb);
    nhat_BA_W = -nhat_BA_W;
  }

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
   from geometry B into geometry A, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  Vector3<T> nhat_BA_W;
  /** The penetration depth. */
  T depth{-1.0};
};

}  // namespace geometry
}  // namespace drake
