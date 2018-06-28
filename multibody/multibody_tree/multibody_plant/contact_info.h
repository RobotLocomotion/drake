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
   */
  PointPairContactInfo(
      BodyIndex bodyA_index, BodyIndex bodyB_index,
      const Vector3<T>& f_Bc_W, const Vector3<T>& p_WC,
      const T& separation_velocity, const T& slip,
      const drake::geometry::PenetrationAsPointPair<T>& point_pair);

  BodyIndex bodyA_index() const { return bodyA_index_; }
  BodyIndex bodyB_index() const { return bodyB_index_; }

  const Vector3<T>& contact_force() const { return f_Bc_W_; }

  const Vector3<T>& contact_point() const { return p_WC_; }

  const T& slip_speed() const { return slip_; }

  const T& separation_velocity() const { return separation_velocity_; }

  const drake::geometry::PenetrationAsPointPair<T>& point_pair() const {
    return point_pair_;
  }

 private:
  drake::geometry::PenetrationAsPointPair<T> point_pair_;
  // Body associated with the geometry with identifier point_pair_.id_A.
  BodyIndex bodyA_index_;
  // Body associated with the geometry with identifier point_pair_.id_B.
  BodyIndex bodyB_index_;
  // Contact force on body B applied at the contact point C, expressed in the
  // world frame W.
  Vector3<T> f_Bc_W_;
  // Contact point.
  Vector3<T> p_WC_;
  // The separation velocity, in m/s, of the contact pair. That is, the rate of
  // change of the signed distance function. That is, separation_velocity_ > 0
  // when bodies are moving away from each other.
  T separation_velocity_;
  // Sliding speed, the norm of the sliding velocity.
  T slip_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
