#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/**
 A class containing information regarding contact response between two bodies
 including:
    - The pair of bodies that are contacting, referenced by their BodyIndex.
    - A resultant contact force.
    - A contact point.
    - Separation velocity.
    - Slip velocity.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
    - double
    - AutoDiffXd
 */
template <typename T>
class PointPairContactInfo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointPairContactInfo)
  /**
   Initialize the contact information for a given pair of two colliding bodies.
   @param bodyA_index
     Index that references a body, body A, in the MultibodyPlant model for
     `this` contact pair.
   @param bodyB_index
     Index that references a body, body B, in the MultibodyPlant model for
     `this` contact pair.
   @param f_Bc_W
     Force on body B applied at contact point C, expressed in the world frame W.
   @param p_WC
     Position of the contact point C in the world frame W.
   @param separation_velocity
     Separation velocity along the normal direction between body A and body B,
     in meters per second. A positive value indicates bodies are moving apart.
   @param slip
     Slip velocity in meters per second.
   @param point_pair
     Additional point pair information for `this` contact info. Refer to the
     documentation for PenetrationAsPointPair for further details.
   */
  PointPairContactInfo(
      BodyIndex bodyA_index, BodyIndex bodyB_index,
      const Vector3<T>& f_Bc_W, const Vector3<T>& p_WC,
      const T& separation_velocity, const T& slip,
      const drake::geometry::PenetrationAsPointPair<T>& point_pair);

  /// Returns the index of body A in the contact pair.
  BodyIndex bodyA_index() const { return bodyA_index_; }

  /// Returns the index of body B in the contact pair.
  BodyIndex bodyB_index() const { return bodyB_index_; }

  /// Returns the contact force `f_Bc_W` on B at contact point C expressed in
  /// the world frame W.
  const Vector3<T>& contact_force() const { return f_Bc_W_; }

  /// Returns the position `p_WC` of the contact point C in the world frame W.
  const Vector3<T>& contact_point() const { return p_WC_; }

  /// Returns the slip velocity between body A and B at contact point C.
  const T& slip_speed() const { return slip_; }

  /// Returns the separation velocity between body A and B along the normal
  /// direction (see PenetrationAsPointPair::nhat_BA_W). It is defined positive
  /// for bodies moving apart in the normal direction.
  const T& separation_velocity() const { return separation_velocity_; }

  /// Returns additional information for the geometric contact query for `this`
  /// pair as a PenetrationAsPointPair.
  const drake::geometry::PenetrationAsPointPair<T>& point_pair() const {
    return point_pair_;
  }

 private:
  // Point pair containing iformation regarding the geometric query for this
  // contact pair.
  drake::geometry::PenetrationAsPointPair<T> point_pair_;
  // Body associated with the geometry with identifier point_pair_.id_A.
  BodyIndex bodyA_index_;
  // Body associated with the geometry with identifier point_pair_.id_B.
  BodyIndex bodyB_index_;
  // Contact force on body B applied at the contact point C, expressed in the
  // world frame W.
  Vector3<T> f_Bc_W_;
  // Position of the contact point in the world frame.
  Vector3<T> p_WC_;
  // The separation velocity, in m/s, of the contact pair. That is, the rate of
  // change of the signed distance function. That is, separation_velocity_ > 0
  // when bodies are moving away from each other.
  T separation_velocity_;
  // Sliding speed, the norm of the tangential velocity.
  T slip_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
