#include "drake/multibody/rigid_body_distance_constraint.h"

#include "drake/common/text_logging.h"

RigidBodyDistCon::RigidBodyDistCon(int bodyA_ind, const Eigen::Vector3d& pointA,
                                int bodyB_ind, const Eigen::Vector3d& pointB,
                                double dist)
    : from_body(bodyA_ind), to_body(bodyB_ind),
      from_point(pointA), to_point(pointB), distance(dist) {}

// std::ostream& operator<<(std::ostream& os, const RigidBodyDistCon& obj) {
//   os << "loop connects pt "
//      << obj.frameA_->get_transform_to_body().matrix().topRightCorner(3, 1)
//          .transpose()
//      << " on " << obj.frameA_->get_rigid_body().get_name() << " to pt "
//      << obj.frameB_->get_transform_to_body().matrix().topRightCorner(3, 1)
//          .transpose()
//      << " on " << obj.frameB_->get_rigid_body().get_name() << std::endl;
//   return os;
// }

