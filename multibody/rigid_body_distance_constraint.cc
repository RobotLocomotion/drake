#include "drake/multibody/rigid_body_distance_constraint.h"

RigidBodyDistanceConstraint::RigidBodyDistanceConstraint(
    int bodyA_index, const Eigen::Vector3d& r_AP, int bodyB_index,
    const Eigen::Vector3d& r_BQ, double distance)
    : bodyA_index_(bodyA_index),
      bodyB_index_(bodyB_index),
      r_AP_(r_AP),
      r_BQ_(r_BQ),
      distance_(distance) {}
