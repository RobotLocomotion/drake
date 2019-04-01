#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
/**
 * Same as SignedDistanceToPoint, except this class also contains the gradient
 * w.r.t p_GQ_G (the position of the query point Q measured and expressed in
 * object G's frame).
 */
struct SignedDistanceToPointWithGradient {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceToPointWithGradient)

  SignedDistanceToPointWithGradient() = default;

  /**
   * Same as SignedDistanceToPoint::id_G. The id of the geometry G to which we
   * measure distance from the query point Q.
   */
  GeometryId id_G;
  /**
   * Same as SignedDistanceToPoint::p_GN. The position of the nearest point N
   * on G's surface to the query point Q, expressed in G's frame.
   */
  Eigen::Vector3d p_GN;
  /**
   * The gradient of p_GN w.r.t p_GQ (the position of query point Q measured and
   * expressed in object G's frame).
   */
  Eigen::Matrix3d dp_GN_dp_GQ;
  /**
   * Same as SignedDistanceToPoint::distance.The signed distance from the query
   * point Q to the nearest point N on the surface of geometry G. It is positive
   * if Q is outside G. It is negative if Q is inside G. It is zero if Q is on
   * the boundary of G.
   */
  double distance{};
  /** The gradient of the distance w.r.t p_GQ. */
  Eigen::RowVector3d ddistance_dp_GQ;
  /**
   * Same as SignedDistanceToPoint::grad_W. The unit length gradient vector of
   * the distance function with respect to the query point Q, expressed in world
   * frame W.
   */
  Eigen::Vector3d grad_W;
  /** The gradient of grad_W w.r.t p_GQ. */
  Eigen::Matrix3d dgrad_W_dp_GQ;
};
}  // namespace geometry
}  // namespace drake
