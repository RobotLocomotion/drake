#include "drake/multibody/rigid_body_loop.h"

#include "drake/common/eigen_autodiff_types.h"

template <typename T>
RigidBodyLoop<T>::RigidBodyLoop(std::shared_ptr<RigidBodyFrame<T>> frameA,
                                std::shared_ptr<RigidBodyFrame<T>> frameB,
                                const Eigen::Vector3d& axis)
    : frameA_(frameA), frameB_(frameB), axis_(axis) {}

std::ostream& operator<<(std::ostream& os, const RigidBodyLoop<double>& obj) {
  os << "loop connects pt "
     << obj.frameA_->get_transform_to_body().matrix().topRightCorner(3, 1)
         .transpose()
     << " on " << obj.frameA_->get_rigid_body().get_name() << " to pt "
     << obj.frameB_->get_transform_to_body().matrix().topRightCorner(3, 1)
         .transpose()
     << " on " << obj.frameB_->get_rigid_body().get_name() << std::endl;
  return os;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyLoop<double>;
template class RigidBodyLoop<drake::AutoDiffXd>;
template class RigidBodyLoop<drake::AutoDiffUpTo73d>;
