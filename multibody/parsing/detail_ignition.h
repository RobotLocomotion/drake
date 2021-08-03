#pragma once

#include <ignition/math/Pose3.hh>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper function to express an ignition::math::Vector3d instance as
// a Vector3d instance.
Eigen::Vector3d ToVector3(const ignition::math::Vector3d& vector);

// Helper function to express an ignition::math::Pose3d instance as
// a RigidTransform instance.
math::RigidTransformd ToRigidTransform(const ignition::math::Pose3d& pose);

// Helper function to express a RigidTransform instance as an
// ignition::math::Pose3d instance.
ignition::math::Pose3d ToIgnitionPose3d(const math::RigidTransformd& pose);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
