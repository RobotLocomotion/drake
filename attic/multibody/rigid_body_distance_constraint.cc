#include "drake/multibody/rigid_body_distance_constraint.h"

RigidBodyDistanceConstraint::RigidBodyDistanceConstraint(
    int bodyA_index_in, const Eigen::Vector3d& r_AP_in, int bodyB_index_in,
    const Eigen::Vector3d& r_BQ_in, double distance_in)
    : bodyA_index(bodyA_index_in),
      bodyB_index(bodyB_index_in),
      r_AP(r_AP_in),
      r_BQ(r_BQ_in),
      distance(distance_in) {}
