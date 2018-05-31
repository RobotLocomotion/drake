#include "drake/multibody/rigid_body_distance_constraint.h"

#include "drake/common/text_logging.h"

RigidBodyDistCon::RigidBodyDistCon(int bodyA_index, const Eigen::Vector3d& pointA,
                                int bodyB_index, const Eigen::Vector3d& pointB,
                                double dist)
    : from_body(bodyA_index), to_body(bodyB_index),
      from_point(pointA), to_point(pointB), distance(dist) {}

