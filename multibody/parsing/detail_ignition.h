#pragma once

#include <ignition/math/Pose3.hh>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace detail {

/// Helper function to express an ignition::math::Vector3d instance as
/// a Vector3d instance.
Eigen::Vector3d ToVector3(const ignition::math::Vector3d& vector);

/// Helper function to express an ignition::math::Pose3d instance as
/// a RigidTransform instance.
math::RigidTransformd ToRigidTransform(const ignition::math::Pose3d& pose);

}  // namespace detail
}  // namespace multibody
}  // namespace drake
