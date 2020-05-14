#include "drake/multibody/joints/helical_joint.h"

#include <memory>
#include <utility>

#include "drake/common/text_logging.h"

using Eigen::Vector3d;

drake::TwistVector<double> HelicalJoint::spatialJointAxis(const Vector3d& axis,
                                                          double pitch) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}

std::unique_ptr<DrakeJoint> HelicalJoint::DoClone() const {
  auto joint = std::make_unique<HelicalJoint>(get_name(),
                                              get_transform_to_parent_body(),
                                              axis_, pitch_);
  return joint;
}
