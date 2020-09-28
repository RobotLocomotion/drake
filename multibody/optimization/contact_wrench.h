#pragma once

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
/**
 * Stores the contact wrench (spatial force) from Body A to Body B applied at
 * point Cb.
 */
struct ContactWrench {
  /**
   * Refer to the documentation for each attribute.
   */
  ContactWrench(BodyIndex bodyA_index_in, BodyIndex bodyB_index_in,
                Eigen::Vector3d p_WCb_W_in, SpatialForce<double> F_Cb_W_in)
      : bodyA_index(bodyA_index_in),
        bodyB_index(bodyB_index_in),
        p_WCb_W(std::move(p_WCb_W_in)),
        F_Cb_W(std::move(F_Cb_W_in)) {}

  /** The index of Body A.*/
  BodyIndex bodyA_index;
  /** The index of Body B.*/
  BodyIndex bodyB_index;
  /** The position of the point Cb (where the wrench is applied) expressed in
   * the world frame W.
   */
  Eigen::Vector3d p_WCb_W;
  /** F_Cb_W_in The wrench (spatial force) applied at point Cb from Body A
   * to Body B, measured in the world frame.
   */
  SpatialForce<double> F_Cb_W;
};
}  // namespace multibody
}  // namespace drake
