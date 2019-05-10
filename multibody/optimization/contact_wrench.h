#pragma once

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
/**
 * Stores the contact wrench (spatial force) from Body A to Body B applied at
 * point Cb.
 */
struct ContactWrench {
  /**
   * @param bodyA_index_in The index of Body A.
   * @param bodyB_index_in The index of Body B.
   * @param p_WCb_W_in The position of the point Cb (where the wrench is
   * applied) expressed in the world frame W.
   * @param F_Cb_W_in The wrench (spatial force) applied at point Cb from Body A
   * to Body B, measured in the world frame.
   */
  ContactWrench(BodyIndex bodyA_index_in, BodyIndex bodyB_index_in,
                Eigen::Vector3d p_WCb_W_in, SpatialForce<double> F_Cb_W_in)
      : bodyA_index(bodyA_index_in),
        bodyB_index(bodyB_index_in),
        p_WCb_W(std::move(p_WCb_W_in)),
        F_Cb_W(std::move(F_Cb_W_in)) {}
  BodyIndex bodyA_index;
  BodyIndex bodyB_index;
  Eigen::Vector3d p_WCb_W;
  SpatialForce<double> F_Cb_W;
};
}  // namespace multibody
}  // namespace drake
