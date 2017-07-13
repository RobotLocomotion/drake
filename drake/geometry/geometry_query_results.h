#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for a single contact between two bodies/elements.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct Contact {
  /** The id of the first geometry in the contact. */
  GeometryId id_A;
  /** The id of the second geometry in the contact. */
  GeometryId id_B;
  /** The point on A that most deeply penetrates B, measured and exprssed in
   the world frame. */
  Vector3<T> p_WCa;
  /** The point on A that most deeply penetrates B, measured and exprssed in
   the world frame. */
  Vector3<T> p_WCb;
  /** The contact normal, pointing from geometry A to geometry B, measured and
   expressed in the world frame. */
  Vector3<T> nhat_AcBc_W;
  /** The penetration depth. */
  T depth{};
};

}  // namespace geometry
}  // namespace drake
