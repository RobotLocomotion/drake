#pragma once

#include <cmath>
#include <utility>

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

  SignedDistanceToPointWithGradient(GeometryId id_G_in, Eigen::Vector3d p_GN_in,
                                    Eigen::Matrix3d dp_GN_dp_GQ_in,
                                    double distance_in,
                                    Eigen::RowVector3d ddistance_dp_GQ_in,
                                    Eigen::Vector3d grad_W_in,
                                    Eigen::Matrix3d dgrad_W_dp_GQ_in,
                                    bool is_grad_W_well_defined_in)
      : id_G(std::move(id_G_in)),
        p_GN(std::move(p_GN_in)),
        dp_GN_dp_GQ(std::move(dp_GN_dp_GQ_in)),
        distance(distance_in),
        ddistance_dp_GQ(std::move(ddistance_dp_GQ_in)),
        grad_W(std::move(grad_W_in)),
        dgrad_W_dp_GQ(std::move(dgrad_W_dp_GQ_in)),
        is_grad_W_well_defined{is_grad_W_well_defined_in} {}

  /**
   * Same as SignedDistanceToPoint::id_G.
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
   * point Q to the nearest point N on the surface of geometry G.
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
  /** The derivative of grad_W w.r.t p_GQ. */
  Eigen::Matrix3d dgrad_W_dp_GQ;

  /** Same as SignedDistanceToPoint::is_grad_W_well_defined. */
  bool is_grad_W_well_defined;
};
}  // namespace geometry
}  // namespace drake
